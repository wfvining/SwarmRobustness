#include "swarm_robustness.h"
#include <stdlib.h>
#include <time.h>

bool isFailed = false;
int failureType = 0; // 0 = no failure, 1 = case 1, 2 = case 2, 3 = case 3 as explained in paper
int maxTimeToFail = 100; // ticks
int scheduledFailTime = 99999999; 
int failureThreshold = 30; // 10 = 10% chance of failure, 50 = 50% chance of failure

bool isSet = false;
int maxFail = 2000; // When's the latest that a bot is allowed to fail
int failPercentage = 0; // The chance that this bot is going to fail
int failTime = 99999999; // Time that the bot is supposed to fail

int hasScrambled = false;
int maxBots = 5;
int failBots[5];
int failTimes[5];
int failIndex = 0;
int numTicks;

SwarmRobustness::SwarmRobustness()
{
    //Constructor
   // std::string test = GetId().substr(2);
   // srand(time(NULL));
   // argos::LOG << "failPercentage: " << failPercentage << std::endl;
   // // Decide whether robot will fail. If so determine when
   // double failure = (rand()*1.0f/RAND_MAX) * 100;
   // if (failure <= failureThreshold) {
   //    isFailed = true;
   //    scheduledFailTime = (rand()*1.0f/RAND_MAX) * maxTimeToFail;
   // } else {
   //    isFailed = false;
   // }
}


SwarmRobustness::~SwarmRobustness() {}

// void configureRobot(int myId) {
//    argos::LOG << "MY ID: " << myId << std::endl;
//    int i = 0;
//    while(i++ < (myId+1)) {
//       failPercentage = (rand() % 100) + 1;
//       failTime = (rand() % maxFail) + 1;
//       argos::LOG << "r: " << failPercentage << " m: " << failTime << std::endl;
//    }
// }

bool robotIsFailed(int robotID) {
   int i = 0;
   while (i < maxBots) {
      // Check robot is eventually going to fail
      if(failBots[i] != -1) {
         if(robotID == failBots[i]) {
            // Check if robot is to be failed now
            if(failTimes[i] <= numTicks) {
               return true;
            } else {
               return false;
            }
         }
      }
   }
   return false;
}

void SwarmRobustness::ControlStep()
{
   int myId = std::stoi (GetId().substr(2),nullptr);
   if (robotIsFailed(myId)) {
      m_pcWheels->SetLinearVelocity(0, 0);
      return;
   }
   // if (!isSet) {
   //    argos::LOG << "MY ID: " << myId << std::endl;
   //    int i = 0;
   //    while(i++ < (myId+1)) {
   //       failPercentage = (rand() % 100) + 1;
   //       failTime = (rand() % maxFail) + 1;
   //       argos::LOG << "r: " << failPercentage << " m: " << failTime << std::endl;
   //    }
   //    // configureRobot(myId);
   //    isSet = true;
   // }

   //Create Wheel Data Object
   DATA wheeldata;
   // argos::LOG << "Fail time: " << scheduledFailTime << " ID: " << GetId() << " w: " << (rand()*1.0f/RAND_MAX) * 100 << std::endl;
   argos::LOG << "ID: " << GetId() << "failTime: " << failTime << std::endl;

   // // Logic if this robot is failed
   // if (isFailed) {
   //    switch(failureType)
   //    {
   //    case 1 : // Complete failures of individual robot
   //       // Shut off wheels
   //       m_pcWheels->SetLinearVelocity(0, 0);
   //       argos::LOG << "Wheel Speeds Left:" << wheeldata.lwVel << "Right:  " << wheeldata.rwVel << std::endl;

   //       // Shut off LED emitter

   //       // Once everything is turned off return
   //       return;
   //       break;
   //    case 2 : // Failure of a robot's IR sensors. Wanders around lost
   //       break;
   //    case 3 : // Failure of a robot's motors only
   //       m_pcWheels->SetLinearVelocity(0, 0);
   //       argos::LOG << "Wheel Speeds Left:" << wheeldata.lwVel << "Right:  " << wheeldata.rwVel << std::endl;
   //       return;
   //       break;
   //    default : // Nothing
   //       break;
   //    }
   // }


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

    argos::LOG << "Wheel Speeds Left:" << wheeldata.lwVel << "Right:  " << wheeldata.rwVel << std::endl;
    if (myId == 0) numTicks ++;
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

   // All bots split the same rand generator. While loop adds diversity to rand generator output.
   // int myId = std::stoi (GetId().substr(2),nullptr);
   // argos::LOG << "MY ID: " << myId << std::endl;
   // int i = 0;
   // while(i++ < (myId+1)) {
   //    failPercentage = (rand() % 100) + 1;
   //    failTime = (rand() % maxFail) + 1;
   //    argos::LOG << "r: " << failPercentage << " m: " << failTime << std::endl;
   // }

   // TODO: Make list of who fails and when they fail
   // Then in the control step if statement will fail based if their id is within that list
   int i = 0;
   int failPercentage = 50;
   int maxFail = 165;
   int time = -1;
   // srand(time(NULL));
   if (!hasScrambled) {
      while(i < maxBots) {
         int fail = (rand() % 100) + 1;
         argos::LOG << "i: " << i << " fail: " << fail << std::endl;

         // If true then robot will fail. Now determine when to fail.
         if (fail <= failureThreshold) {
            time = (rand() % maxFail) + 1;
            failBots[i] = i;
            failTimes[i] = time;
         } else {
            failBots[i] = -1;
            failTimes[i] = 99999999;
         }
         argos::LOG << "failBots: " << failBots[i] << " failTime: " << failTimes[i] << std::endl;
         i++;
         // failIndex ++;
      }
      hasScrambled = true;
   }
   numTicks = 0;
   argos::LOG << "FINISHED INIT" << std::endl;
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


