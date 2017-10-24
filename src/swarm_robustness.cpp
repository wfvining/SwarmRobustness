#include "swarm_robustness.h"

SwarmRobustness::SwarmRobustness()
{
    //Constructor
}

SwarmRobustness::~SwarmRobustness()
{
    //Destructor
}

void SwarmRobustness::ControlStep()
{
    //Create Wheel Data Object
    DATA wheeldata;

/*   BeaconInSight();
   if(beacon_detected)
   {
      // TODO: SetCollisionAvoidanceRadius(high)
   }
   else
   {
      // TODO: SetCollisionAvoidanceRaduius(low)
   }

   if(TimeSinceLastAvoidanceCall() > turn_threshold)
   {
      CRadians swarm_bearing = GetSwarmBearing();
      // TODO: turn towards swarm bearing
   }
   else
   {
      // TODO: Drive while avoiding obstacles
   }
*/

    //Obstacle Avoidance

    //Set Variables to 0;
    obstsense.distance = m_pcProximity->GetReadings()[0];
    obstsense.sensorID = 0;

    if(obstsense.distance < m_pcProximity->GetReadings()[1])
    {
       obstsense.distance = m_pcProximity->GetReadings()[1];
       obstsense.sensorID = 1;
    }
    if(obstsense.distance < m_pcProximity->GetReadings()[7])
    {
       obstsense.distance = m_pcProximity->GetReadings()[7];
       obstsense.sensorID = 7;
    }
    if(obstsense.distance < m_pcProximity->GetReadings()[6])
    {
       obstsense.distance = m_pcProximity->GetReadings()[6];
       obstsense.sensorID = 6;
    }

    //Create ObstController
    Obstacle obstController;

    if(obstController.ShouldAvoid(&obstsense))
    {
        time_since_collision = 0;
        if(obstsense.turnRight)
        {
            wheeldata.rwVel = 2.5;
            wheeldata.lwVel = 0;
        }
        else
        {
            wheeldata.rwVel = 0;
            wheeldata.lwVel = 2.5;
        }
    }
    else
    {
        time_since_collision++;
        // Check to see if we need to flock back

        /*-----FLOCKING CODE-----*/

        float tolerance = 5;

        // if dist >= tolerance
        if(flocking)
        {

            //argos::LOG << "FLOCKING" << std::endl;

            // Get Error
            CRadians errorBackToSwarm = GetSwarmBearing();
            argos::LOG << GetId() << ": swarm bearing " << GetSwarmBearing() << std::endl;

            // Determine if we are turning right or left
            if(errorBackToSwarm > CRadians(0.0))
            {
                // RIGHT
                wheeldata.lwVel = 0.0;
                wheeldata.rwVel = 2.5;
            }
            else
            {
                // LEFT
                wheeldata.lwVel = 0.0;
                wheeldata.rwVel = 2.5;
            }

            // check error, if below, turn flocking behavior off
            if(errorBackToSwarm <= CRadians(0.05) && errorBackToSwarm >= CRadians(-0.05))
            {
                argos::LOG << "done flocking" << std::endl;
                flocking = false;
                wheeldata.lwVel = 2.5;
                wheeldata.rwVel = 2.5;
            }

        }
        else if(time_since_collision >= tolerance)
        {
            flocking = true;
        }
        //otherwise drive straight
        else
        {
            wheeldata.lwVel = 2.5;
            wheeldata.rwVel = 2.5;
        }
    }

                                   //rightwheel, leftwheel
    m_pcWheels->SetLinearVelocity(wheeldata.rwVel, wheeldata.lwVel);

//    Used to debug wheel speed
//    argos::LOG << "Wheel Speeds Left:" << wheeldata.lwVel << "Right:  " << wheeldata.rwVel << std::endl;

}

void SwarmRobustness::Destroy() {   }

void SwarmRobustness::Init(TConfigurationNode& t_node)
{
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity");
   m_pcLight     = GetSensor  <CCI_LightSensor                 >("light");
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing");
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing");

   // get the failure mode from the config.
   int fm;
   GetNodeAttributeOrDefault(t_node, "failure_mode", fm, 0);
   if(fm == 0)
   {
      failure_mode = NO_FAILURE;
      argos::LOG << "failure mode: NO_FAILURE" << std::endl;
   }
   else if(fm == 1)
   {
      failure_mode = MOTOR_FAILURE;
      argos::LOG << "failure mode: MOTOR_FAILURE" << std::endl;
   }
   else if(fm == 2)
   {
      failure_mode = POWER_FAILURE;
      argos::LOG << "failure mode: POWER_FAILURE" << std::endl;
   }
   else if(fm == 3)
   {
      failure_mode = SENSOR_FAILURE;
      argos::LOG << "failure mode: SENSOR_FAILURE" << std::endl;
   }
}

int SwarmRobustness::TimeSinceLastAvoidanceCall()
{
   return 10; // TODO: actually return something true
}

void SwarmRobustness::BeaconInSight()
{
   if(failed == SENSOR_FAILURE)
   {
      beacon_detected = false;
      return;
   }
   
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

CRadians SwarmRobustness::GetSwarmBearing()
{
   const CCI_RangeAndBearingSensor::TReadings& readings = m_pcRABS->GetReadings();

   if(failed == SENSOR_FAILURE)
   {
      // ignore all readings.
      return CRadians(0.0);
   }

   if(readings.size() == 0)
   {
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

void SwarmRobustness::SensorFailure()
{
   failed = SENSOR_FAILURE;
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(SwarmRobustness, "swarm_robustness_controller")


