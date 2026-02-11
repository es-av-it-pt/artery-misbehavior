/*
 * Artery V2X Simulation Framework — Misbehavior Module
 * Neighbor Table: Stores received CAMs for replay and disruptive attacks
 */

#ifndef ARTERY_NEIGHBOR_TABLE_H_
#define ARTERY_NEIGHBOR_TABLE_H_

#include "artery/misbehavior/enumTypes/MisbehaviorTypes.h"
#include <vanetza/units/length.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/asn1/cam.hpp>
#include <omnetpp/simtime.h>
#include <vanetza/units/angle.hpp>
#include <vanetza/common/clock.hpp>
#include <map>
#include <vector>
#include <cstdint>

namespace artery
{
namespace misbehavior
{

struct NeighborRecord
{
    uint32_t stationId;
    vanetza::asn1::Cam cam;
    omnetpp::SimTime receiveTime;
    vanetza::Clock::time_point generationTime;
};

/**
 * Manages list of neighboring vehicles and their latest CAMs
 * Used by MdAttack for DataReplay, Disruptive, and Sybil attacks.
 */
class NeighborTable
{
public:
    NeighborTable() = default;

    /** Add or update a neighbor record */
    void addNeighbor(const NeighborRecord& record);

    /** Get a random neighbor record. Returns nullptr if table is empty. */
    const NeighborRecord* getRandomNeighbor() const;
    
    /** Get the full CAM of a random neighbor. Returns nullptr if table is empty. */
    const vanetza::asn1::Cam* getRandomCam() const;
    
    /**
     * Get data from the closest neighbor
     * @param lat Attacker's latitude
     * @param lon Attacker's longitude
     * @return Pointer to neighbor record, or nullptr if table is empty
     */
    const NeighborRecord* getClosestNeighbor(vanetza::units::GeoAngle lat, vanetza::units::GeoAngle lon) const;

    /** 
     * Get data from the next sequential victim (round-robin).
     * Picks a random neighbor different from lastVictimId if possible.
     */
    const NeighborRecord* getNextVictim(uint32_t lastVictimId) const;
    
    int size() const { return mNeighbors.size(); }
    bool empty() const { return mNeighbors.empty(); }
    void clear() { mNeighbors.clear(); }

private:
    static constexpr size_t MAX_RECORDS = 50;
    std::vector<NeighborRecord> mNeighbors;
};

} // namespace misbehavior
} // namespace artery

#endif /* ARTERY_NEIGHBOR_TABLE_H_ */
