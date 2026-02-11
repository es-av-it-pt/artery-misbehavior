/*
 * Artery V2X Simulation Framework — Misbehavior Module
 * Neighbor table implementation
 */

#include "artery/misbehavior/base/NeighborTable.h"
#include <omnetpp/csimulation.h>
#include <omnetpp/crng.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace artery
{
namespace misbehavior
{

void NeighborTable::addNeighbor(const NeighborRecord& record)
{
    // Update existing record or add new one
    for (auto& existing : mNeighbors) {
        if (existing.stationId == record.stationId) {
            existing = record;
            return;
        }
    }
    
    // Limit buffer size
    if (mNeighbors.size() >= MAX_RECORDS) {
        mNeighbors.erase(mNeighbors.begin());
    }
    mNeighbors.push_back(record);
}

const NeighborRecord* NeighborTable::getRandomNeighbor() const
{
    if (mNeighbors.empty()) return nullptr;
    int idx = omnetpp::intuniform(omnetpp::cSimulation::getActiveSimulation()->getContext()->getRNG(0), 
                                   0, mNeighbors.size() - 1);
    return &mNeighbors[idx];
}

const vanetza::asn1::Cam* NeighborTable::getRandomCam() const
{
    const NeighborRecord* record = getRandomNeighbor();
    if (record) return &record->cam;
    return nullptr;
}

const NeighborRecord* NeighborTable::getNextVictim(uint32_t lastVictimId) const
{
    if (mNeighbors.empty()) return nullptr;
    
    // Try to find a different victim first
    if (mNeighbors.size() > 1) {
        auto* rng = omnetpp::cSimulation::getActiveSimulation()->getContext()->getRNG(0);
        for (int attempt = 0; attempt < 5; ++attempt) {
            int idx = omnetpp::intuniform(rng, 0, mNeighbors.size() - 1);
            if (mNeighbors[idx].stationId != lastVictimId) {
                return &mNeighbors[idx];
            }
        }
    }
    
    // Default: just pick random
    return getRandomNeighbor();
}

const NeighborRecord* NeighborTable::getClosestNeighbor(vanetza::units::GeoAngle lat, vanetza::units::GeoAngle lon) const
{
    if (mNeighbors.empty()) return nullptr;

    const NeighborRecord* closest = nullptr;
    double minDistSq = std::numeric_limits<double>::max();
    double myLatVal = lat.value();
    double myLonVal = lon.value();

    // Simple equirectangular approximation for comparison
    // M_PI is typically defined in <cmath> or <math.h>
    double cosLat = std::cos(myLatVal * M_PI / 180.0);

    for (const auto& record : mNeighbors) {
        const auto& basic = record.cam->cam.camParameters.basicContainer;
        // CAM positions are in 1/10 microdegree
        double theirLat = (double)basic.referencePosition.latitude / 10000000.0;
        double theirLon = (double)basic.referencePosition.longitude / 10000000.0;

        double dLat = myLatVal - theirLat;
        double dLon = (myLonVal - theirLon) * cosLat;
        double distSq = dLat*dLat + dLon*dLon;

        if (distSq < minDistSq) {
            minDistSq = distSq;
            closest = &record;
        }
    }
    return closest;
}

} // namespace misbehavior
} // namespace artery
