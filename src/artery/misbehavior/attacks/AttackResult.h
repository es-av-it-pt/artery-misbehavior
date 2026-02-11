/*
 * Artery V2X Simulation Framework — Misbehavior Module
 * Attack Result structure
 *
 * Defines the outcome of an attack calculation.
 * This struct is message-agnostic (holds physical values, not encoded bytes).
 * The calling service maps these values to the specific message format (CAM, CPM, DENM).
 */

#ifndef ARTERY_MISBEHAVIOR_ATTACK_RESULT_H_
#define ARTERY_MISBEHAVIOR_ATTACK_RESULT_H_

#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>
#include <boost/optional.hpp>
#include <vector>
#include <cstdint> // for uint32_t

namespace artery
{
namespace misbehavior
{

/**
 * Represents a single Sybil identity's state for the current message
 */
struct SybilIdentity
{
    uint32_t stationId;
    vanetza::units::GeoAngle latitude;
    vanetza::units::GeoAngle longitude;
    vanetza::units::Velocity speed;
    // Add heading/yaw rate if needed
};

/**
 * Output of MdAttack::launchAttack()
 * Contains optional overrides for physical values and flags for special behaviors.
 */
struct AttackResult
{
    // Physical value overrides (if set, use these instead of real values)
    boost::optional<vanetza::units::GeoAngle> latitude;
    boost::optional<vanetza::units::GeoAngle> longitude;
    boost::optional<vanetza::units::Velocity> speed;
    
    // Attack flags
    bool useFrozenMessage = false;   // For EventualStop / StaleMessages
    bool useReplayMessage = false;   // For DataReplay
    
    // Denial of Service control
    bool dropMessage = false;        // For probabilistic drops (DoSRandom)
    int duplicateCount = 0;          // How many EXTRA copies to send (0 = normal 1 copy)
    
    // Sybil identities to generate (in addition to primary identity)
    std::vector<SybilIdentity> sybilIdentities;
};

} // namespace misbehavior
} // namespace artery

#endif /* ARTERY_MISBEHAVIOR_ATTACK_RESULT_H_ */
