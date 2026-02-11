/*
 * Artery V2X Simulation Framework — Misbehavior Module
 * Centralized configuration for misbehavior parameters
 * Adapted from F2MD's F2MDParameters
 */

#ifndef ARTERY_MISBEHAVIOR_CONFIG_H_
#define ARTERY_MISBEHAVIOR_CONFIG_H_

#include "artery/misbehavior/enumTypes/MisbehaviorTypes.h"
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>

namespace artery
{
namespace misbehavior
{

/**
 * Centralized configuration struct for all misbehavior-related parameters.
 * Equivalent to F2MD's F2MDParameters class.
 * 
 * Values are read from NED parameters during CaService::initialize()
 * and passed to MdAttack::init().
 */
struct MisbehaviorConfig
{
    // Attack identity
    AttackType attackType = AttackType::None;

    // Timing
    double startAttack = 10.0;  // seconds — warm-up period before attacks begin

    // Position falsification
    vanetza::units::Length falsificationOffset = 10.0 * boost::units::si::meter;

    // Speed falsification
    vanetza::units::Velocity falsificationSpeedOffset = 0.0 * boost::units::si::meter_per_second;

    // Per-attacker variation (F2MD's parVar)
    double parVar = -1.0;  // -1.0 = randomize per node

    // DoS parameters
    int dosMultipleFreq = 4;    // Frequency multiplier for DoS attacks
    double doSProbability = 0.5; // Drop probability for DoSRandomSybil

    // Attack probability (F2MD alignment)
    double attackProbability = 1.0; // 1.0 = Always attack after warmup

    // DataReplay parameters
    int replaySeqNum = 6;  // Number of times to replay same victim before switching
    VictimSelection victimSelection = VictimSelection::Random;

    // Sybil parameters
    int sybilVehNumber = 4;  // Number of Sybil ghost vehicles
};

} // namespace misbehavior
} // namespace artery

#endif /* ARTERY_MISBEHAVIOR_CONFIG_H_ */
