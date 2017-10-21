#include "swarm_robustness.h"
#include <stdlib.h>
#include <time.h>

 
 
// Feel free to tune the following variables
int failureType = 1; // 0 = no failure, 1 = case 1 failure as explained in paper, case 2 etc.
int failureThreshold = 30; // 10 = 10% chance of failure, 50 = 50% chance of failure
int maxFail = 165; /* maxFail represents the latest that a robot is allowed to die at. 
  * Scale is n = n*10 steps (ex. 165 = step 1650 is latest that robot can die at) */


// Values that do not require tuning.
int hasScrambled = false;
int maxBots = 120;
int numTicks;
bool robotFailList[120]; // Boolean value if corresponding robot is going to eventually fail
int robotFailTime[120]; // Value of time that corresponding robot is supposed to fail
int robotMaxTime = 99999999; // Constant used for robots that are not supposed to fail

SwarmRobustness::SwarmRobustness()
{
    //Constructor
}


SwarmRobustness::~SwarmRobustness() {}


bool robotIsFailed(int robotId, int numTicks) {
   if (robotFailList[robotId] == true && robotFailTime[robotId] <= (numTicks/10)) {
      return true;
   }
   return false;
}

void SwarmRobustness::ControlStep()
{
   // myId and numTicks++ lines must stay as first in order to work.
   int myId = std::stoi (GetId().substr(2),nullptr);
   if (myId == 0) numTicks ++;

   //Create Wheel Data Object
   DATA wheeldata;

   // When a robot fails perform its corresponding failure case.
   if (robotIsFailed(myId, numTicks)) {
      switch(failureType)
      {
      case 1 : // Complete failure of individual robot
         m_pcWheels->SetLinearVelocity(0, 0);
         return;
         break;
      case 2 : // Failure of a robot's IR sensors. Wanders around lost
         break;
      case 3 : // Failure of a robot's motors only
         m_pcWheels->SetLinearVelocity(0, 0);
         return;
         break;
      default : // Nothing
         break;
      }
   }


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

    //Create ObstController
    Obstacle obstController;

    if(obstController.ShouldAvoid(&obstsense))
    {
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
        wheeldata.lwVel = 2.5;
        wheeldata.rwVel = 2.5;
    }

                                   //rightwheel, leftwheel
    m_pcWheels->SetLinearVelocity(wheeldata.rwVel, wheeldata.lwVel);

    // argos::LOG << "Wheel Speeds Left:" << wheeldata.lwVel << "Right:  " << wheeldata.rwVel << std::endl;
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

   int i = 0;
   int time = -1;
   if (!hasScrambled) {
      while(i < maxBots) {
         int fail = (rand() % 100) + 1;
         // argos::LOG << "i: " << i << " fail: " << fail << std::endl;

         // If true then robot will fail. Now determine when to fail.
         if (fail <= failureThreshold) {
            time = (rand() % maxFail) + 1;
            // argos::LOG << "time: " << time << std::endl;
            robotFailTime[i] = time;
            robotFailList[i] = true;
         } else {
            robotFailTime[i] = robotMaxTime;
            robotFailList[i] = false;
         }
         // argos::LOG << "robot: " << i << " failTime: " << robotFailTime[i] << std::endl;
         i++;
      }
      hasScrambled = true;
   }
   numTicks = -1; // Must be set to -1 because how it is incremented in ControlStep
   // argos::LOG << "FINISHED INIT" << std::endl;
}

int SwarmRobustness::TimeSinceLastAvoidanceCall()
{
   return 10; // TODO: actually return something true
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

CRadians SwarmRobustness::GetSwarmBearing()
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


