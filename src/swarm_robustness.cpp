#include "swarm_robustness.h"

SwarmRobustness::SwarmRobustness() {}

SwarmRobustness::~SwarmRobustness() {}

void SwarmRobustness::ControlStep()
{
   SwarmDirection();
}

void SwarmRobustness::Destroy()
{}

void SwarmRobustness::Init(TConfigurationNode& t_node)
{
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity");
   m_pcLight     = GetSensor  <CCI_LightSensor                 >("light");
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing");
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing");
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

CRadians SwarmRobustness::SwarmDirection()
{
   const CCI_RangeAndBearingSensor::TReadings& readings = m_pcRABS->GetReadings();

   if(readings.empty()) {
      // If there are no readings, then just keep driving.
      // (Should maybe turn a random direction instead).
      argos::LOG << "[" << GetId() << "]: got no range and bearing readings" << std::endl;
      return CRadians(0.0);
   }
   
   CRadians sum(0.0);
   for( auto packet : readings )
   {
      sum += packet.HorizontalBearing;
   }

   // return the average bearing to all readings
   return sum / readings.size();
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(SwarmRobustness, "swarm_robustness_controller")


