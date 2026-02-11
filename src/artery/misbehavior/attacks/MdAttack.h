/*
 * Artery V2X Simulation Framework — Misbehavior Module
 * MdAttack Class
 *
 * Core attack engine that computes falsified values based on current state.
 * Implemented as a plain C++ class (not an OMNeT++ module) for logic separation.
 */

#ifndef ARTERY_MISBEHAVIOR_MD_ATTACK_H_
#define ARTERY_MISBEHAVIOR_MD_ATTACK_H_

#include "artery/misbehavior/support/MisbehaviorConfig.h"
#include "artery/misbehavior/attacks/AttackResult.h"
#include "artery/misbehavior/base/NeighborTable.h"
#include <omnetpp/simtime.h>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vector>

// Forward declaration
namespace omnetpp { class cComponent; class cRNG; }

namespace artery
{
class VehicleDataProvider; // Forward declaration

namespace misbehavior
{

class MdAttack
{
public:
    MdAttack() = default;

    /**
     * Initialize the attack engine with config and context.
     * @param config Configuration parameters
     * @param vdp Vehicle data provider
     * @param rng Component to access RNG from
     */
    void init(const MisbehaviorConfig& config, const VehicleDataProvider& vdp,
              omnetpp::cComponent* rng);

    /**
     * Launch the attack logic for the current message generation event.
     * @param vdp Current vehicle state
     * @param currentTime Current simulation time
     * @param genDeltaTimeMod Generation delta time (ms)
     * @return AttackResult structure with overrides or flags
     */
    AttackResult launchAttack(const VehicleDataProvider& vdp, omnetpp::SimTime currentTime,
                              uint16_t genDeltaTimeMod);

    bool isActive(omnetpp::SimTime currentTime) const;

    // Neighbor management methods
    bool needsNeighborData() const;
    void onNeighborCam(const vanetza::asn1::Cam& cam, omnetpp::SimTime t);

    // Getters for CaService logging
    AttackType getAttackType() const { return mConfig.attackType; }
    
    // Access methods for complex attacks
    bool hasFrozenCam() const { return mHasFrozenCam; }
    const vanetza::asn1::Cam& getFrozenCam() const { return mFrozenCam; }

    bool hasReplayCam() const { return mHasReplayCam; }
    const vanetza::asn1::Cam& getReplayCam() const { return mReplayCam; }

private:
    MisbehaviorConfig mConfig;
    uint32_t mStationId = 0;
    omnetpp::cComponent* mRng = nullptr;

    // Persistent attack state (const positions/speeds)
    // Absolute position (ConstPos)
    struct {
        vanetza::units::GeoAngle latitude;
        vanetza::units::GeoAngle longitude;
    } mConstPos;

    // Offset (ConstPosOffset)
    struct {
        vanetza::units::GeoAngle latitude;
        vanetza::units::GeoAngle longitude;
    } mConstPosOffset;
    vanetza::units::Velocity mConstSpeed;

    // Derived parameters
    vanetza::units::Length mScaledOffset;
    vanetza::units::Velocity mScaledSpeedOffset;

    // Attack state — EventualStop / StaleMessages
    bool mHasFrozenCam = false;
    vanetza::asn1::Cam mFrozenCam;

    // Attack state — DataReplay
    bool mHasReplayCam = false;
    vanetza::asn1::Cam mReplayCam;
    int mReplaySeq = 0;
    uint32_t mCurrentVictimId = 0;

    // Attack state — DoS
    bool mDoSInitiated = false;

    // Neighbor storage (for replay/disruptive attacks)
    NeighborTable mNeighbors;

    // Helper: random number generation (uses the cComponent's RNG)
    double dblrand() const;
    int intuniform(int a, int b) const;
};

} // namespace misbehavior
} // namespace artery

#endif /* ARTERY_MISBEHAVIOR_MD_ATTACK_H_ */
