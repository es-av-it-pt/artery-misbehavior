//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

simple CaService like ItsG5Service
{
    parameters:
        @signal[CamReceived](type=CaObject);
        @signal[CamSent](type=CaObject);

        @statistic[reception](source=CamReceived;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);
        @statistic[transmission](source=CamSent;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);

        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundaries
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering CAM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(23);

        // Position falsification attack configuration
        bool isAttacker = default(false);
        int misbehaviorType = default(0); // 0=None, 1=ConstPos, etc.
        double falsificationOffset @unit(m) = default(10.0m);
        double falsificationSpeedOffset @unit(mps) = default(0.0mps);
        double doSProbability = default(0.5);
        double parVar = default(-1.0); // -1.0 means randomized per node
        int dosMultipleFreq = default(4); // F2MD default: 4x frequency for DoS
        int replaySeqNum = default(6); // F2MD default: replay same victim 6 times
        int victimSelection = default(0); // 0: Random, 1: Closest
        int sybilVehNum = default(4); // F2MD default: 4 ghost vehicles
        double attackProbability = default(1.0); // 1.0 = Always attach, 0.5 = 50% chance
        double startAttack = default(10.0); // F2MD default: start attack at T=10s
}
