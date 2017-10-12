#include "swarm_robustness.h"

SwarmRobustness::SwarmRobustness() {}

SwarmRobustness::~SwarmRobustness() {}

void SwarmRobustness::ControlStep()
{}

void SwarmRobustness::Destroy()
{}

void SwarmRobustness::Init(TConfigurationNode& t_node)
{
   // TODO
}

void SwarmRobustness::BeaconInSight()
{
   Real total_reading = 0.0;
   const std::vector<Real> readings = m_pcLight->GetReadings();
   for(Real reading : readings)
   {
      total_reading += reading;
   }

   // a robot can see the light if at least 1/3 of its light sensor
   // readings are 1
   if(total_reading / readings.size() >= 0.33)
   {
      beacon_detected = true;
   }
   else
   {
      beacon_detected = false;
   }
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(SwarmRobustness, "swarm_robustness_controller")


