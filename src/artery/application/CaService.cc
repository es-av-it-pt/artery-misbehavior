/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2019 Raphael Riebl et al.
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/CaObject.h"
#include "artery/application/CaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>

namespace artery
{

using namespace omnetpp;

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
	static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
	static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

	SpeedValue_t speed = SpeedValue_unavailable;
	if (v >= upper) {
		speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
	} else if (v >= lower) {
		speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	}
	return speed;
}

Define_Module(CaService)

CaService::CaService() :
		mGenCamMin { 100, SIMTIME_MS },
		mGenCamMax { 1000, SIMTIME_MS },
		mGenCam(mGenCamMax),
		mGenCamLowDynamicsCounter(0),
		mGenCamLowDynamicsLimit(3)
{
}

void CaService::initialize()
{
	ItsG5BaseService::initialize();
	mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mTimer = &getFacilities().get_const<Timer>();
	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastCamTimestamp = simTime();

	// first generated CAM shall include the low frequency container
	mLastLowCamTimestamp = mLastCamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

	// generation rate boundaries
	mGenCamMin = par("minInterval");
	mGenCamMax = par("maxInterval");
	mGenCam = mGenCamMax;

	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");

	// look up primary channel for CA
	mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);

	// MISBEHAVIOR INIT
	mIsAttacker = par("isAttacker");
	if (mIsAttacker) {
		misbehavior::MisbehaviorConfig config;
		config.attackType = static_cast<misbehavior::AttackType>(par("misbehaviorType").intValue());
		config.falsificationOffset = par("falsificationOffset").doubleValue() * vanetza::units::si::meter;
		config.falsificationSpeedOffset = par("falsificationSpeedOffset").doubleValue() * vanetza::units::si::meter_per_second;
		config.doSProbability = par("doSProbability");
		config.parVar = par("parVar");
		config.dosMultipleFreq = par("dosMultipleFreq");
		config.replaySeqNum = par("replaySeqNum");
		config.victimSelection = static_cast<misbehavior::VictimSelection>(par("victimSelection").intValue());
		config.sybilVehNumber = par("sybilVehNum");
		config.startAttack = par("startAttack");
		config.attackProbability = par("attackProbability");

		mMdAttack.init(config, *mVehicleDataProvider, this);

		EV_INFO << "CaService initialized as ATTACKER for Station " << mVehicleDataProvider->getStationId() 
				<< " Type=" << misbehavior::attackTypeName(config.attackType) << "\n";
	} else {
		EV_INFO << "CaService initialized for Station " << mVehicleDataProvider->getStationId() << " (Honest)\n";
	}
}

void CaService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}

void CaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

	Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
	const vanetza::asn1::Cam* cam = boost::apply_visitor(visitor, *packet);
	if (cam && cam->validate()) {
		EV_INFO << "Received CAM from Station ID: " << (*cam)->header.stationID << "\n";
		
		// Pass to Misbehavior Module (for neighbor table / replay attacks)
		if (mIsAttacker && mMdAttack.needsNeighborData()) {
			mMdAttack.onNeighborCam(*cam, simTime());
		}

		CaObject obj = visitor.shared_wrapper;
		emit(scSignalCamReceived, &obj);
		mLocalDynamicMap->updateAwareness(obj);
	}
}

void CaService::checkTriggeringConditions(const SimTime& T_now)
{
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenCam = mGenCam;
	const SimTime& T_GenCamMin = mGenCamMin;
	const SimTime& T_GenCamMax = mGenCamMax;
	const SimTime T_GenCamDcc = mDccRestriction ? genCamDcc() : T_GenCamMin;
	const SimTime T_elapsed = T_now - mLastCamTimestamp;

	// For DoS attacks: if we are attacking (and active), force trigger regardless of interval?
	// But F2MD logic usually triggers on normal intervals, just modifies content or sends bursts.
	// We stick to standard trigger logic, but sendCam handles bursts.

	if (T_elapsed >= T_GenCamDcc) {
		if (mFixedRate) {
			sendCam(T_now);
		} else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
			sendCam(T_now);
			T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
			mGenCamLowDynamicsCounter = 0;
		} else if (T_elapsed >= T_GenCam) {
			sendCam(T_now);
			if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
				T_GenCam = T_GenCamMax;
			}
		}
	}
}

bool CaService::checkHeadingDelta() const
{
	return !vanetza::facilities::similar_heading(mLastCamHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool CaService::checkPositionDelta() const
{
	return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool CaService::checkSpeedDelta() const
{
	return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void CaService::sendCam(const SimTime& T_now)
{
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	
	misbehavior::AttackResult result;
	if (mIsAttacker) {
		result = mMdAttack.launchAttack(*mVehicleDataProvider, T_now, genDeltaTimeMod);
		if (result.dropMessage) {
            EV_INFO << "ATTACKER: Dropping CAM (DoS drop)\n";
			return; // DoS Random drop
		}
	}

	// Prepare list of CAMs to send (usually 1, unless DoS or Sybil)
	std::vector<vanetza::asn1::Cam> cams_to_send;

	// 1. Primary Identity CAM
	//    - Check for Frozen (EventualStop) or Replay (DataReplay) overrides
	if (result.useFrozenMessage && mMdAttack.hasFrozenCam()) {
		cams_to_send.push_back(mMdAttack.getFrozenCam());
	} else if (result.useReplayMessage && mMdAttack.hasReplayCam()) {
		cams_to_send.push_back(mMdAttack.getReplayCam());
	} else {
		// Normal generation (possibly with Falsified position/speed in result)
		auto cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTimeMod, result);
		cams_to_send.push_back(cam);
	}
    
    // 2. DoS Duplicates
    if (result.duplicateCount > 0) {
        // Add extra copies of the primary CAM
        for (int i=0; i < result.duplicateCount; ++i) {
            cams_to_send.push_back(cams_to_send.front());
        }
    }

	// 3. Sybil Identities
	for (const auto& sybil : result.sybilIdentities) {
		// Create a separate message for Sybil
		// We can reuse createCooperativeAwarenessMessage but we need to trick it?
		// Or just manually construct. Reusing is better but requires VDP.
		// Construct manually using the Sybil data:
        vanetza::asn1::Cam sybilCam = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTimeMod); // Base honest CAM
        
        // Override with Sybil data
        sybilCam->header.stationID = sybil.stationId;
        
        // Position
        sybilCam->cam.camParameters.basicContainer.referencePosition.latitude = 
            round(sybil.latitude, microdegree) * Longitude_oneMicrodegreeEast; // Typo in original code? No: Longitude..
        // Wait, standard code uses:
        // latitude = round(..., microdegree) * Latitude_oneMicrodegreeNorth
        sybilCam->cam.camParameters.basicContainer.referencePosition.latitude = 
            round(sybil.latitude, microdegree) * Latitude_oneMicrodegreeNorth;
        sybilCam->cam.camParameters.basicContainer.referencePosition.longitude = 
            round(sybil.longitude, microdegree) * Longitude_oneMicrodegreeEast;
            
        // Speed
        sybilCam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = 
            buildSpeedValue(sybil.speed);
            
        cams_to_send.push_back(sybilCam);
	}

    // LOGGING
    if (mIsAttacker) {
        EV_INFO << "ATTACKER - " << misbehavior::attackTypeName(mMdAttack.getAttackType()) 
                << " [Type " << (int)mMdAttack.getAttackType() << "] (ID: " << mVehicleDataProvider->getStationId() 
                << "): Generating " << cams_to_send.size() << " CAM(s) at T=" << T_now << "\n"
                << "  [Ground Truth / What is true]\n"
                << "    Real - Pos: (" << mVehicleDataProvider->longitude().value() << ", " << mVehicleDataProvider->latitude().value() << ")\n"
                << "    Real - Spd: " << mVehicleDataProvider->speed().value() << "\n"
                << "    Real - Hdg: " << mVehicleDataProvider->heading().value() << "\n";

        if (!cams_to_send.empty()) {
            // Log first CAM details (primary falsification)
            const auto& cam = cams_to_send.front();
            double sent_lat = (double)cam->cam.camParameters.basicContainer.referencePosition.latitude / 10000000.0;
            double sent_lon = (double)cam->cam.camParameters.basicContainer.referencePosition.longitude / 10000000.0;
            
            // Speed extraction is complex due to encoding, simplifying for log:
            double sent_spd = -1.0;
            if (result.speed) sent_spd = result.speed->value();
            else sent_spd = mVehicleDataProvider->speed().value(); // Close enough for honest part

            EV_INFO << "  [Sent / What was claimed (Primary Identity)]\n"
                    << "    Sent - Pos: (" << sent_lon << ", " << sent_lat << ")\n"
                    << "    Sent - Spd: " << sent_spd << "\n";
                    
            if (result.useFrozenMessage) EV_INFO << "    Sent - Hdg: FROZEN\n";
            else EV_INFO << "    Sent - Hdg: " << mVehicleDataProvider->heading().value() << "\n";
        }
    } else {
        EV_INFO << "HONEST (ID: " << mVehicleDataProvider->getStationId() << ")\n";
    }

    // SEND LOOP
	mLastCamPosition = mVehicleDataProvider->position();
	mLastCamSpeed = mVehicleDataProvider->speed();
	mLastCamHeading = mVehicleDataProvider->heading();
	mLastCamTimestamp = T_now;

    using namespace vanetza;
    
	for (auto& msg : cams_to_send) {
        // Low Frequency Container logic (only for primary, ideally)
        // But let's apply to all for now or just primary.
        // Applying to primary only:
        if (&msg == &cams_to_send.front() && T_now - mLastLowCamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
            addLowFrequencyContainer(msg, par("pathHistoryLength"));
            // Update timestamp only once
            mLastLowCamTimestamp = T_now; 
        }

        btp::DataRequestB request;
        request.destination_port = btp::ports::CAM;
        request.gn.its_aid = aid::CA;
        request.gn.transport_type = geonet::TransportType::SHB;
        request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
        request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
        request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

        CaObject obj(std::move(msg));
        emit(scSignalCamSent, &obj);

        using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
        std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
        std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
        payload->layer(OsiLayer::Application) = std::move(buffer);
        this->request(request, std::move(payload));
    }
}

SimTime CaService::genCamDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(ca_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
}

vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime)
{
    misbehavior::AttackResult emptyResult;
    return createCooperativeAwarenessMessage(vdp, genDeltaTime, emptyResult);
}

vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime, const misbehavior::AttackResult& result)
{
	vanetza::asn1::Cam message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_cam;
	header.stationID = vdp.station_id();

	CoopAwareness_t& cam = (*message).cam;
	cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = cam.camParameters.basicContainer;
	HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	
    // Longitude
    auto lon = result.longitude ? *result.longitude : vdp.longitude();
    basic.referencePosition.longitude = round(lon, microdegree) * Longitude_oneMicrodegreeEast;

    // Latitude
    auto lat = result.latitude ? *result.latitude : vdp.latitude();
	basic.referencePosition.latitude = round(lat, microdegree) * Latitude_oneMicrodegreeNorth;

	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
	bvc.heading.headingValue = round(vdp.heading(), decidegree);
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	
    // Speed
    auto speed = result.speed ? *result.speed : vdp.speed();
    bvc.speed.speedValue = buildSpeedValue(speed);
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
    
	bvc.driveDirection = speed.value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	bvc.yawRate.yawRateValue = round(vdp.yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (bvc.yawRate.yawRateValue < -32766 || bvc.yawRate.yawRateValue > 32766) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}
	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	bvc.vehicleWidth = VehicleWidth_unavailable;

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
	}

	return message;
}

void addLowFrequencyContainer(vanetza::asn1::Cam& message, unsigned pathHistoryLength)
{
	if (pathHistoryLength > 40) {
		EV_WARN << "path history can contain 40 elements at maximum";
		pathHistoryLength = 40;
	}

	LowFrequencyContainer_t*& lfc = message->cam.camParameters.lowFrequencyContainer;
	lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
	lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
	BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
	bvc.vehicleRole = VehicleRole_default;
	bvc.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	assert(nullptr != bvc.exteriorLights.buf);
	bvc.exteriorLights.size = 1;
	bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);

	for (unsigned i = 0; i < pathHistoryLength; ++i) {
		PathPoint* pathPoint = vanetza::asn1::allocate<PathPoint>();
		pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
		*(pathPoint->pathDeltaTime) = (i + 1) * PathDeltaTime_tenMilliSecondsInPast * 10;
		pathPoint->pathPosition.deltaLatitude = DeltaLatitude_unavailable;
		pathPoint->pathPosition.deltaLongitude = DeltaLongitude_unavailable;
		pathPoint->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
		ASN_SEQUENCE_ADD(&bvc.pathHistory, pathPoint);
	}

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
	}
}

} // namespace artery
