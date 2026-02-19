/*
* Artery V2X Simulation Framework
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_CPSERVICE_H_
#define ARTERY_CPSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace artery
{

class NetworkInterfaceTable;
class Timer;
class VehicleDataProvider;

class CpService : public ItsG5BaseService
{
	public:
		CpService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;

	private:
		void checkTriggeringConditions(const omnetpp::SimTime&);
        bool checkSensorInformationTrigger(const omnetpp::SimTime&);
		bool checkPerceptionRegionTrigger(const omnetpp::SimTime&);
		bool checkPerceivedObjectTrigger(const omnetpp::SimTime&);
		void sendCpm(const omnetpp::SimTime&);
		omnetpp::SimTime genCpmDcc();

		ChannelNumber mPrimaryChannel = channel::CCH;
		const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
		const VehicleDataProvider* mVehicleDataProvider = nullptr;
		const Timer* mTimer = nullptr;
		LocalDynamicMap* mLocalDynamicMap = nullptr;
        LocalEnvironmentModel* mLocalEnvironmentModel = nullptr;

		omnetpp::SimTime mGenCpmMin;
		omnetpp::SimTime mGenCpmMax;
		omnetpp::SimTime mGenCpm;
		omnetpp::SimTime mLastCpmTimestamp;
		bool mDccRestriction;
        omnetpp::SimTime mAddSensorInformation;
        omnetpp::SimTime mLastSensorInformationTimestamp;
        int mObjectInclusionConfig;

};

vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider&, uint16_t referenceTime);
void addOriginatingVehicleContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp);
void addOriginatingRsuContainer(vanetza::asn1::Cpm& message);
void addSensorInformationContainer(vanetza::asn1::Cpm& message, const LocalEnvironmentModel& lem, const std::vector<Sensor*>& sensors);
void addPerceptionRegionContainer(vanetza::asn1::Cpm& message);
void addPerceivedObjectContainer(vanetza::asn1::Cpm& message, const VehicleDataProvider& vdp, const LocalEnvironmentModel::TrackedObjects& objects);


} // namespace artery

#endif /* ARTERY_CPSERVICE_H_ */
