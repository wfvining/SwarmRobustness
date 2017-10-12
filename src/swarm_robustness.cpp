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

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(SwarmRobustness, "swarm_robustness_controller")


