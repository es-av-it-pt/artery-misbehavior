/*
* Artery V2X Simulation Framework
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CpObject.h"
#include "artery/application/CpService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/envmod/sensor/FovSensor.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <chrono>


namespace artery
{

using namespace omnetpp;

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}


Define_Module(CpService)

CpService::CpService() :
		mGenCpmMin { 100, SIMTIME_MS },
		mGenCpmMax { 1000, SIMTIME_MS },
		mGenCpm(mGenCpmMax)
{
}

void CpService::initialize()
{
	ItsG5BaseService::initialize();
	mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mTimer = &getFacilities().get_const<Timer>();
	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();
    mLocalEnvironmentModel = &getFacilities().get_mutable<LocalEnvironmentModel>();

	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastCpmTimestamp = simTime();

    mLastSensorInformationTimestamp = mLastCpmTimestamp;

	// generation rate boundaries
	mGenCpmMin = par("minInterval");
	mGenCpmMax = par("maxInterval");
	mGenCpm = mGenCpmMax;

    mAddSensorInformation = par("addSensorInformation");
    mObjectInclusionConfig = par("objectInclusionConfig");

    mDccRestriction = par("withDccRestriction");

	// look up primary channel for CP
	mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CP);
}

void CpService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}

void CpService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

	Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
	const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);
	if (cpm && cpm->validate()) {
		CpObject obj = visitor.shared_wrapper;
		emit(scSignalCpmReceived, &obj);
        
        // DEBUG: print received CPM
		std::cout << mVehicleDataProvider->getStationId() << " received a CPM: " << std::endl;
		asn_fprint(nullptr, &asn_DEF_Vanetza_ITS2_ItsPduHeader, &(*cpm)->header);
		asn_fprint(nullptr, &asn_DEF_Vanetza_ITS2_CpmPayload, &(*cpm)->payload);

	}
}

void CpService::checkTriggeringConditions(const SimTime& T_now)
{
	SimTime& T_GenCam = mGenCpm;
	const SimTime& T_GenCpmMin = mGenCpmMin;
	const SimTime& T_GenCpmMax = mGenCpmMax;
    const SimTime T_GenCpmDcc = mDccRestriction ? genCpmDcc() : T_GenCpmMin;
	const SimTime T_elapsed = T_now - mLastCpmTimestamp;

	if (T_elapsed >= T_GenCpmDcc) {
        sendCpm(T_now);
	}
}

bool CpService::checkSensorInformationTrigger(const SimTime& T_now)
{
	const SimTime& T_AddSensorInformation = mAddSensorInformation;
	return (T_now - mLastSensorInformationTimestamp >= T_AddSensorInformation);
}

bool CpService::checkPerceptionRegionTrigger(const SimTime& T_now)
{
	// The currently available sensors on artery do not provide dynamic perception regions
    // to check differences on shape, confidence and shadowing.
    // This container is never added to the CPM.
	return false;
}

bool CpService::checkPerceivedObjectTrigger(const SimTime& T_now)
{
    if (mObjectInclusionConfig == 0) {
        // no rules for object inclusion
    }
	else if (mObjectInclusionConfig != 1) EV_ERROR << "Invalid object inclusion configuration: " << mObjectInclusionConfig << endl;

    return true;
}


void CpService::sendCpm(const SimTime& T_now)
{
	// uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    uint64_t referenceTime = countTaiMilliseconds(mTimer->getTimeFor(T_now));
	auto cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, referenceTime);

    if (mVehicleDataProvider->getStationType() == vanetza::geonet::StationType::RSU) {
		addOriginatingRsuContainer(cpm);
    } else {
        addOriginatingVehicleContainer(cpm, *mVehicleDataProvider);
    }
    if (checkSensorInformationTrigger(T_now)) {
		addSensorInformationContainer(cpm, *mLocalEnvironmentModel, mLocalEnvironmentModel->getSensors());
		mLastSensorInformationTimestamp = T_now;
	}
	if (checkPerceptionRegionTrigger(T_now)) {
		addPerceptionRegionContainer(cpm);
	}
	if (checkPerceivedObjectTrigger(T_now)) {
        addPerceivedObjectContainer(cpm, *mVehicleDataProvider, mLocalEnvironmentModel->allObjects());
    }

    std::string error;
	if (!cpm.validate(error)) {
		throw cRuntimeError("Invalid CPM: %s", error.c_str());
	}

	mLastCpmTimestamp = T_now;

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CPM;
	request.gn.its_aid = aid::CP;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	CpObject obj(std::move(cpm));
	emit(scSignalCpmSent, &obj);

	// DEBUG: print sent CPM
	std::cout << mVehicleDataProvider->getStationId() << " sent a CPM: " << std::endl;

	using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CpmByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));
}

SimTime CpService::genCpmDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CP's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite cp_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(cp_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenCpmMax, std::max(mGenCpmMin, dcc));
}

vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider& vdp, uint16_t referenceTime)
{
	vanetza::asn1::Cpm message;

    Vanetza_ITS2_ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageId = Vanetza_ITS2_MessageId_cpm;
	header.stationId = vdp.station_id();

    Vanetza_ITS2_CpmPayload_t& payload = (*message).payload;

	Vanetza_ITS2_CPM_PDU_Descriptions_ManagementContainer_t& mc = payload.managementContainer;

	int ret = asn_uint642INTEGER(&mc.referenceTime, referenceTime);
    assert(ret == 0);

	mc.referencePosition.latitude = round(vdp.latitude(), vanetza::units::degree * boost::units::si::micro) * 10;
	mc.referencePosition.longitude = round(vdp.longitude(), vanetza::units::degree * boost::units::si::micro) * 10;
	mc.referencePosition.positionConfidenceEllipse.semiMajorConfidence = Vanetza_ITS2_SemiAxisLength_unavailable;
	mc.referencePosition.positionConfidenceEllipse.semiMinorConfidence = Vanetza_ITS2_SemiAxisLength_unavailable;
	mc.referencePosition.positionConfidenceEllipse.semiMajorOrientation = Vanetza_ITS2_HeadingValue_unavailable;
	mc.referencePosition.altitude.altitudeValue = Vanetza_ITS2_AltitudeValue_unavailable;
	mc.referencePosition.altitude.altitudeConfidence = Vanetza_ITS2_AltitudeConfidence_unavailable;

	// TODO: add optional values

	return message;
}

void addOriginatingVehicleContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp)
{
	Vanetza_ITS2_WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer_t>();
	wcc->containerId = Vanetza_ITS2_CpmContainerId_originatingVehicleContainer;
	wcc->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_OriginatingVehicleContainer;

	Vanetza_ITS2_OriginatingVehicleContainer_t& ovc = wcc->containerData.choice.OriginatingVehicleContainer;
	ovc.orientationAngle.value = round(vdp.heading(), vanetza::units::degree * boost::units::si::deci);
	ovc.orientationAngle.confidence = Vanetza_ITS2_Wgs84AngleConfidence_unavailable;

	// TODO: add optional values

	int ret = ASN_SEQUENCE_ADD(&(message->payload.cpmContainers.list), wcc);
	assert(ret == 0);

    std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Originating Vehicle Container: %s", error.c_str());
	}
}

void addOriginatingRsuContainer(vanetza::asn1::Cpm& message)
{
	Vanetza_ITS2_WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer_t>();
	wcc->containerId = Vanetza_ITS2_CpmContainerId_originatingRsuContainer;
	wcc->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_OriginatingRsuContainer;

	Vanetza_ITS2_OriginatingRsuContainer_t& orc = wcc->containerData.choice.OriginatingRsuContainer;

	// TODO: add optional values

	int ret = ASN_SEQUENCE_ADD(&(message->payload.cpmContainers.list), wcc);
	assert(ret == 0);

    std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Originating Rsu Container: %s", error.c_str());
	}
}

void addSensorInformationContainer(vanetza::asn1::Cpm& message, const LocalEnvironmentModel& lem, const std::vector<Sensor*>& sensors)
{
	Vanetza_ITS2_WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer_t>();
	wcc->containerId = Vanetza_ITS2_CpmContainerId_sensorInformationContainer;
	wcc->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_SensorInformationContainer;

	Vanetza_ITS2_SensorInformationContainer_t& sic = wcc->containerData.choice.SensorInformationContainer;
	
	sic.list.array = vanetza::asn1::allocate<Vanetza_ITS2_SensorInformation_t*>();
	Vanetza_ITS2_SensorInformation_t* si = vanetza::asn1::allocate<Vanetza_ITS2_SensorInformation_t>();
	si->sensorId = 0;
	si->sensorType = Vanetza_ITS2_SensorType_radar;
	// TODO: add optional values
	si->shadowingApplies = false;
	ASN_SEQUENCE_ADD(&sic.list, si);

	int ret = ASN_SEQUENCE_ADD(&(message->payload.cpmContainers.list), wcc);
	assert(ret == 0);

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Sensor Information Container: %s", error.c_str());
	}
}

void addPerceptionRegionContainer(vanetza::asn1::Cpm& message)
{
	// Vanetza_ITS2_WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer_t>();
	// wcc->containerId = Vanetza_ITS2_CpmContainerId_perceptionRegionContainer;
	// wcc->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_PerceptionRegionContainer;

	// Vanetza_ITS2_PerceptionRegionContainer_t& prc = wcc->containerData.choice.PerceptionRegionContainer;

	// ...
}

template<typename T, typename U>
static long roundToUnit(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

void addPerceivedObjectContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp, const LocalEnvironmentModel::TrackedObjects& objects)
{
	Vanetza_ITS2_WrappedCpmContainer_t* wcc = vanetza::asn1::allocate<Vanetza_ITS2_WrappedCpmContainer_t>();
	wcc->containerId = Vanetza_ITS2_CpmContainerId_perceivedObjectContainer;
	wcc->containerData.present = Vanetza_ITS2_WrappedCpmContainer__containerData_PR_PerceivedObjectContainer;

	Vanetza_ITS2_PerceivedObjectContainer_t& poc = wcc->containerData.choice.PerceivedObjectContainer;
	poc.numberOfPerceivedObjects = objects.size();

	// Ego vehicle position
    const Position& egoPos = vdp.position();

	// get timestampIts from management container.
    const Vanetza_ITS2_CPM_PDU_Descriptions_ManagementContainer_t& mc = (*message).payload.managementContainer;
    const INTEGER_t& timestampIts = mc.referenceTime;
    int64_t referenceTime;
    asn_INTEGER2imax(&timestampIts, &referenceTime);

	for (LocalEnvironmentModel::TrackedObject obj : objects) {
		
		Vanetza_ITS2_PerceivedObject_t* po = vanetza::asn1::allocate<Vanetza_ITS2_PerceivedObject_t>();
		
		const LocalEnvironmentModel::Tracking& tracking = obj.second;

		po->objectId = vanetza::asn1::allocate<Vanetza_ITS2_Identifier2B_t>();
		*(po->objectId) = tracking.id();

		// Get last detected time
        SimTime lastDetected = SimTime::ZERO;
        for (const auto& sensor : tracking.sensors()) {
            if(sensor.first->getSensorCategory() == "FoV") {
                if(lastDetected < sensor.second.last()) {
                    lastDetected = sensor.second.last();
                }
            }
        }

		// a measurement delta time shall be provided for each object as the time difference for the provided measurement
        // information with respect to the reference time of type TimestampIts stated in the Management Container
        // time unit -> milliseconds
        int64_t deltaTime = lastDetected.inUnit(SIMTIME_MS) - referenceTime;
        // clamp measurementDeltaTime according to cdd documentation
        if(deltaTime < -2048) deltaTime = -2048;
        if(deltaTime > 2046) deltaTime = 2047;
        po->measurementDeltaTime = deltaTime;

		auto objPtr = obj.first.lock();
        if (!objPtr) {
            // Object no longer exists, skip or handle error
            std::cout << "Object with ID " << tracking.id() << " no longer exists." << std::endl;
            continue;
        }
        const Position& objectPos = objPtr->getCentrePoint();
        Vanetza_ITS2_CartesianPosition3dWithConfidence_t& position = po->position;

		position.xCoordinate.value = roundToUnit(objectPos.x - egoPos.x, vanetza::units::si::meter * boost::units::si::centi);
        if(position.xCoordinate.value < Vanetza_ITS2_CartesianCoordinateLarge_negativeOutOfRange)
            position.xCoordinate.value = Vanetza_ITS2_CartesianCoordinateLarge_negativeOutOfRange;
        if(position.xCoordinate.value > Vanetza_ITS2_CartesianCoordinateLarge_positiveOutOfRange)
            position.xCoordinate.value = Vanetza_ITS2_CartesianCoordinateLarge_positiveOutOfRange;
        
		position.xCoordinate.confidence = Vanetza_ITS2_CoordinateConfidence_unavailable;
        if(position.xCoordinate.confidence > 4094)
            position.xCoordinate.confidence = Vanetza_ITS2_CoordinateConfidence_outOfRange;

        position.yCoordinate.value = roundToUnit(objectPos.y - egoPos.y, vanetza::units::si::meter * boost::units::si::centi);
        if(position.yCoordinate.value < Vanetza_ITS2_CartesianCoordinateLarge_negativeOutOfRange)
            position.yCoordinate.value = Vanetza_ITS2_CartesianCoordinateLarge_negativeOutOfRange;
        if(position.yCoordinate.value > Vanetza_ITS2_CartesianCoordinateLarge_positiveOutOfRange)
            position.yCoordinate.value = Vanetza_ITS2_CartesianCoordinateLarge_positiveOutOfRange;
        
		position.yCoordinate.confidence = Vanetza_ITS2_CoordinateConfidence_unavailable;
        if(position.yCoordinate.confidence > 4094)
            position.yCoordinate.confidence = Vanetza_ITS2_CoordinateConfidence_outOfRange;

		int ret = ASN_SEQUENCE_ADD(&poc.perceivedObjects.list, po);
		assert(ret == 0);

		// TODO: add optional values
	}

	int ret = ASN_SEQUENCE_ADD(&(message->payload.cpmContainers.list), wcc);
	assert(ret == 0);

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Perceived Object Container: %s", error.c_str());
	}
}


} // namespace artery
