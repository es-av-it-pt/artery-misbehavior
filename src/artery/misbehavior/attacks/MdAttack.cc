/*
 * Artery V2X Simulation Framework — Misbehavior Module
 * Attack engine implementation — extracted from CaService::sendCam()
 *
 * Contains the attack switch statement that computes falsified values.
 * Each case produces an AttackResult; the calling service (CaService)
 * maps it to the specific message format.
 */

#include "artery/misbehavior/attacks/MdAttack.h"
#include "artery/application/VehicleDataProvider.h"
#include <omnetpp/ccomponent.h>
#include <omnetpp/crng.h>
#include <omnetpp/csimulation.h>
#include <vanetza/units/angle.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <cmath>
#include <iostream>

namespace artery
{
namespace misbehavior
{

static auto microdegree = vanetza::units::degree * boost::units::si::micro;

void MdAttack::init(const MisbehaviorConfig& config, const VehicleDataProvider& vdp,
                    omnetpp::cComponent* rng)
{
    mConfig = config;
    mStationId = vdp.getStationId();
    mRng = rng;

    // Apply per-attacker variation (parVar)
    double parVar = mConfig.parVar;
    if (parVar < 0) {
        parVar = dblrand(); // Randomize per node
    }
    mConfig.parVar = parVar;

    mScaledOffset = mConfig.falsificationOffset * parVar;
    mScaledSpeedOffset = mConfig.falsificationSpeedOffset * parVar;

    // Pre-compute constants for attacks that need them
    AttackType type = mConfig.attackType;
    
    // Explicit types for clarity and unit safety
    vanetza::units::GeoAngle delta_lat_angle = 0.0 * vanetza::units::degree;
    vanetza::units::GeoAngle delta_lon_angle = 0.0 * vanetza::units::degree;
    
    if (type == AttackType::ConstPos || type == AttackType::ConstPosOffset) {
        double dx = (dblrand() - 0.5) * 2.0 * mScaledOffset.value();
        double dy = (dblrand() - 0.5) * 2.0 * mScaledOffset.value();
        
        double R = 6371000.0;
        double delta_lat = (dy / R) * (180.0 / M_PI);
        double delta_lon = (dx / (R * cos(vdp.latitude().value() * M_PI / 180.0))) * (180.0 / M_PI);
        
        delta_lat_angle = delta_lat * vanetza::units::degree;
        delta_lon_angle = delta_lon * vanetza::units::degree;
        
        if (type == AttackType::ConstPos) {
            mConstPos.longitude = vdp.longitude() + delta_lon_angle;
            mConstPos.latitude = vdp.latitude() + delta_lat_angle;
        } else {
            // ConstPosOffset: store the delta as relative values (Angle)
            mConstPosOffset.longitude = delta_lon_angle;
            mConstPosOffset.latitude = delta_lat_angle;
        }
    }
    
    if (type == AttackType::ConstSpeed || type == AttackType::ConstSpeedOffset) {
        mConstSpeed = (dblrand() - 0.5) * 2.0 * mScaledSpeedOffset;
        if (type == AttackType::ConstSpeed) {
            mConstSpeed = dblrand() * 40.0 * vanetza::units::si::meter_per_second;
        }
    }

    std::cout << "[MdAttack] Initialized: type=" << attackTypeName(type)
              << " parVar=" << parVar
              << " stationId=" << mStationId << std::endl;
}

AttackResult MdAttack::launchAttack(const VehicleDataProvider& vdp, omnetpp::SimTime currentTime,
                                     uint16_t genDeltaTimeMod)
{
    AttackResult result;
    AttackType type = mConfig.attackType;

    if (!isActive(currentTime)) {
        return result; // No overrides during warm-up
    }

    // Probabilistic attack behavior (F2MD alignment)
    if (mConfig.attackProbability < 1.0) {
        if (dblrand() > mConfig.attackProbability) {
            return result; // Send benign CAM (honest behavior)
        }
    }

    double R = 6371000.0;
    double lat_val = vdp.latitude().value();
    double cos_lat = cos(lat_val * M_PI / 180.0);

    switch (type) {
        case AttackType::ConstPos:
            result.latitude = mConstPos.latitude;
            result.longitude = mConstPos.longitude;
            break;

        case AttackType::ConstPosOffset:
            result.latitude = vdp.latitude() + mConstPosOffset.latitude;
            result.longitude = vdp.longitude() + mConstPosOffset.longitude;
            break;

        case AttackType::RandomPos: {
            double dx = (dblrand() - 0.5) * 2000.0;
            double dy = (dblrand() - 0.5) * 2000.0;
            double d_lat = (dy / R) * (180.0 / M_PI);
            double d_lon = (dx / (R * cos_lat)) * (180.0 / M_PI);
            result.latitude = vdp.latitude() + d_lat * vanetza::units::degree;
            result.longitude = vdp.longitude() + d_lon * vanetza::units::degree;
            break;
        }

        case AttackType::RandomPosOffset: {
            double dx = (dblrand() - 0.5) * 2.0 * mScaledOffset.value();
            double dy = (dblrand() - 0.5) * 2.0 * mScaledOffset.value();
            double d_lat = (dy / R) * (180.0 / M_PI);
            double d_lon = (dx / (R * cos_lat)) * (180.0 / M_PI);
            result.latitude = vdp.latitude() + d_lat * vanetza::units::degree;
            result.longitude = vdp.longitude() + d_lon * vanetza::units::degree;
            break;
        }

        case AttackType::ConstSpeed:
            result.speed = mConstSpeed;
            break;

        case AttackType::ConstSpeedOffset:
            result.speed = vdp.speed() + mConstSpeed;
            if (result.speed->value() < 0) result.speed = 0.0 * vanetza::units::si::meter_per_second;
            break;

        case AttackType::RandomSpeed:
            result.speed = dblrand() * 40.0 * vanetza::units::si::meter_per_second;
            break;

        case AttackType::RandomSpeedOffset: {
            double d_speed = (dblrand() - 0.5) * 2.0 * mScaledSpeedOffset.value();
            result.speed = vdp.speed() + d_speed * vanetza::units::si::meter_per_second;
            if (result.speed->value() < 0) result.speed = 0.0 * vanetza::units::si::meter_per_second;
            break;
        }

        case AttackType::EventualStop:
        case AttackType::StaleMessages:
            if (!mHasFrozenCam) {
                mHasFrozenCam = true;
                // CaService will create the frozen CAM logic or we just flag it here
            }
            result.useFrozenMessage = true;
            break;

        case AttackType::Disruptive: {
            double dx = (dblrand() - 0.5) * 1000.0;
            double dy = (dblrand() - 0.5) * 1000.0;
            double d_lat = (dy / R) * (180.0 / M_PI);
            double d_lon = (dx / (R * cos_lat)) * (180.0 / M_PI);
            result.latitude = vdp.latitude() + d_lat * vanetza::units::degree;
            result.longitude = vdp.longitude() + d_lon * vanetza::units::degree;
            result.speed = dblrand() * 60.0 * vanetza::units::si::meter_per_second;
            break;
        }

        case AttackType::DataReplay:
            if (!mNeighbors.empty()) {
                if (mReplaySeq >= mConfig.replaySeqNum) {
                    mCurrentVictimId = 0;
                    mReplaySeq = 0;
                }
                
                if (mCurrentVictimId == 0) {
                    const NeighborRecord* victim = nullptr;
                    if (mConfig.victimSelection == VictimSelection::Closest) {
                        victim = mNeighbors.getClosestNeighbor(vdp.latitude(), vdp.longitude());
                    } else {
                        victim = mNeighbors.getNextVictim(mCurrentVictimId);
                    }

                    if (victim) {
                        mReplayCam = victim->cam;
                        mCurrentVictimId = victim->stationId;
                        mHasReplayCam = true;
                        std::cout << "[MdAttack] DataReplay: Selected new victim Station ID " 
                                  << mCurrentVictimId << std::endl;
                    }
                }
                
                if (mHasReplayCam) {
                    result.useReplayMessage = true;
                    mReplaySeq++;
                    std::cout << "[MdAttack] DataReplay: Replaying victim " << mCurrentVictimId 
                              << " (count " << mReplaySeq << "/" << mConfig.replaySeqNum << ")" << std::endl;
                }
            }
            break;

        case AttackType::DoS:
        case AttackType::DoSRandom:
        case AttackType::DoSDisruptive:
            if (!mDoSInitiated) {
                mDoSInitiated = true;
                std::cout << "[MdAttack] " << attackTypeName(type) << " Attack Initiated: Frequency increased by " 
                          << mConfig.dosMultipleFreq << "x" << std::endl;
            }
            result.duplicateCount = mConfig.dosMultipleFreq;
            
            if (type == AttackType::DoSRandom) {
                // Falsify payload as RandomPos
                double dx = (dblrand() - 0.5) * 2000.0;
                double dy = (dblrand() - 0.5) * 2000.0;
                double d_lat = (dy / R) * (180.0 / M_PI);
                double d_lon = (dx / (R * cos_lat)) * (180.0 / M_PI);
                result.latitude = vdp.latitude() + d_lat * vanetza::units::degree;
                result.longitude = vdp.longitude() + d_lon * vanetza::units::degree;
            } else if (type == AttackType::DoSDisruptive) {
                // Falsify payload as Disruptive
                double dx = (dblrand() - 0.5) * 1000.0;
                double dy = (dblrand() - 0.5) * 1000.0;
                double d_lat = (dy / R) * (180.0 / M_PI);
                double d_lon = (dx / (R * cos_lat)) * (180.0 / M_PI);
                result.latitude = vdp.latitude() + d_lat * vanetza::units::degree;
                result.longitude = vdp.longitude() + d_lon * vanetza::units::degree;
                result.speed = dblrand() * 60.0 * vanetza::units::si::meter_per_second;
            }
            break;

        case AttackType::GridSybil:
        case AttackType::DataReplaySybil:
        case AttackType::DoSRandomSybil:
        case AttackType::DoSDisruptiveSybil: {
            // Generate multiple Sybil identities
            uint32_t sybilCount = mConfig.sybilVehNumber;
            
            for (uint32_t i = 0; i < sybilCount; ++i) {
                SybilIdentity sybil;
                sybil.stationId = mStationId + 1000 + i + (uint32_t)(currentTime.dbl() * 10);
                
                // Base position relative to attacker
                double offsetLat = 0.0, offsetLon = 0.0;
                
                if (type == AttackType::GridSybil) {
                    // Grid formation
                    int row = i / 2;
                    int col = i % 2;
                    offsetLat = (row + 1) * 0.0001; 
                    offsetLon = (col + 1) * 0.0001;
                    sybil.latitude = vdp.latitude() + offsetLat * vanetza::units::degree;
                    sybil.longitude = vdp.longitude() + offsetLon * vanetza::units::degree;
                    sybil.speed = vdp.speed();
                } else if (type == AttackType::DataReplaySybil) {
                    // Replay existing data? Needs more complex logic.
                    // Simplified: just offset
                    sybil.latitude = vdp.latitude() + 0.001 * vanetza::units::degree;
                    sybil.longitude = vdp.longitude() + 0.001 * vanetza::units::degree;
                    sybil.speed = vdp.speed();
                } else {
                    // Random / Disruptive
                    double dx = (dblrand() - 0.5) * 2000.0;
                    double dy = (dblrand() - 0.5) * 2000.0;
                    double d_lat = (dy / R) * (180.0 / M_PI);
                    double d_lon = (dx / (R * cos_lat)) * (180.0 / M_PI);
                    sybil.latitude = vdp.latitude() + d_lat * vanetza::units::degree;
                    sybil.longitude = vdp.longitude() + d_lon * vanetza::units::degree;
                    sybil.speed = dblrand() * 40.0 * vanetza::units::si::meter_per_second;
                }
                
                result.sybilIdentities.push_back(sybil);
            }
            break;
        }

        default:
            break;
    }

    return result;
}

bool MdAttack::isActive(omnetpp::SimTime currentTime) const
{
    // Attack starts after warm-up period
    return currentTime.dbl() >= mConfig.startAttack;
}

bool MdAttack::needsNeighborData() const
{
    return mConfig.attackType == AttackType::DataReplay ||
           mConfig.attackType == AttackType::Disruptive || /* Maybe? */
           mConfig.attackType == AttackType::DataReplaySybil;
}

void MdAttack::onNeighborCam(const vanetza::asn1::Cam& cam, omnetpp::SimTime t)
{
    NeighborRecord record;
    // Access header with pointer syntax since wrapper behaves like pointer
    record.stationId = cam->header.stationID;
    
    // Copy the CAM (wrapper copies pointer/data)
    record.cam = cam;
    record.receiveTime = t;
    mNeighbors.addNeighbor(record);
}

double MdAttack::dblrand() const
{
    if (mRng) {
        // Use component's RNG - getRNG(0) returns cRNG*
        return omnetpp::uniform(mRng->getRNG(0), 0.0, 1.0);
    }
    return 0.5;
}

int MdAttack::intuniform(int a, int b) const
{
    if (mRng) {
        // Use component's RNG
        return omnetpp::intuniform(mRng->getRNG(0), a, b);
    }
    return a;
}

} // namespace misbehavior
} // namespace artery
