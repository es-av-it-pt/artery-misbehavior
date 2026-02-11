/*
 * Artery V2X Simulation Framework — Misbehavior Module
 * Enumeration of attack types and strategies
 */

#ifndef ARTERY_MISBEHAVIOR_TYPES_H_
#define ARTERY_MISBEHAVIOR_TYPES_H_

#include <string>

namespace artery
{
namespace misbehavior
{

enum class AttackType
{
    None = 0,               // Honest behavior

    // Position Falsification
    ConstPos = 1,           // Fixed absolute position
    ConstPosOffset = 2,     // Fixed offset from real position
    RandomPos = 3,          // Random absolute position
    RandomPosOffset = 4,    // Random offset from real position

    // Speed Falsification
    ConstSpeed = 5,         // Fixed absolute speed
    ConstSpeedOffset = 6,   // Fixed offset from real speed
    RandomSpeed = 7,        // Random absolute speed
    RandomSpeedOffset = 8,  // Random offset from real speed

    // Message Manipulation
    EventualStop = 9,       // Simulate a stopped vehicle (frozen data)
    Disruptive = 10,        // Randomized position/speed per message
    DataReplay = 11,        // Replay neighbor CAMs
    StaleMessages = 12,     // Replay old self-messages (frozen)

    // Denial of Service
    DoS = 13,               // Malicious — honest data at high frequency
    DoSRandom = 14,         // Malicious — random data at high frequency
    DoSDisruptive = 15,     // Malicious — victim data at high frequency

    // Sybil Attacks
    GridSybil = 16,         // Multiple identities in grid formation
    DataReplaySybil = 17,   // Replay existing data with Sybil IDs
    DoSRandomSybil = 18,    // DoS with random Sybil IDs
    DoSDisruptiveSybil = 19 // DoS with disruptive Sybil IDs
};

/**
 * Strategy for selecting victims (DataReplay)
 */
enum class VictimSelection
{
    Random = 0,
    Closest = 1
};

inline std::string attackTypeName(AttackType type)
{
    switch(type) {
        case AttackType::None: return "None";
        case AttackType::ConstPos: return "ConstPos";
        case AttackType::ConstPosOffset: return "ConstPosOffset";
        case AttackType::RandomPos: return "RandomPos";
        case AttackType::RandomPosOffset: return "RandomPosOffset";
        case AttackType::ConstSpeed: return "ConstSpeed";
        case AttackType::ConstSpeedOffset: return "ConstSpeedOffset";
        case AttackType::RandomSpeed: return "RandomSpeed";
        case AttackType::RandomSpeedOffset: return "RandomSpeedOffset";
        case AttackType::EventualStop: return "EventualStop";
        case AttackType::Disruptive: return "Disruptive";
        case AttackType::DataReplay: return "DataReplay";
        case AttackType::StaleMessages: return "StaleMessages";
        case AttackType::DoS: return "DoS";
        case AttackType::DoSRandom: return "DoSRandom";
        case AttackType::DoSDisruptive: return "DoSDisruptive";
        case AttackType::GridSybil: return "GridSybil";
        case AttackType::DataReplaySybil: return "DataReplaySybil";
        case AttackType::DoSRandomSybil: return "DoSRandomSybil";
        case AttackType::DoSDisruptiveSybil: return "DoSDisruptiveSybil";
        default: return "Unknown";
    }
}

} // namespace misbehavior
} // namespace artery

#endif /* ARTERY_MISBEHAVIOR_TYPES_H_ */
