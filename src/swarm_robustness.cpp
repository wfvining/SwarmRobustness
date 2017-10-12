#include "swarm_robustness.h"
#include <argos3/core/utility/logging/argos_log.h>

SwarmRobustness::SwarmRobustness() {}

void SwarmRobustness::Init(TConfigurationNode& t_node)
{
   // TODO
}

void SwarmRobustness::ControlStep()
{
   BeaconInSight();
}

void SwarmRobustness::BeaconInSight()
{
   const std::vector<Real> readings = m_pcLight->GetReadings();
   argos::LOG << "readings[" << GetId() << "]:";
   for(auto reading : readings)
   {
      argos::LOG << " " << reading;
   }
   argos::LOG << std::endl;
}
