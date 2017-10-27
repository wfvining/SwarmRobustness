#include "swarm_robustness.h"
#include <stdlib.h>
#include <math.h>
#include <argos3/core/utility/math/rng.h>
 
 
// Values that do not require tuning.
int hasScrambled = false;
int numTicks;
bool robotFailList[120]; // Boolean value if corresponding robot is going to eventually fail
int robotFailTime[120]; // Value of time that corresponding robot is supposed to fail
int robotMaxTime = 99999999; // Constant used for robots that are not supposed to fail
int logFrequency = 0;

#include <math.h>

SwarmRobustness::SwarmRobustness()
{
    //Constructor
}

SwarmRobustness::~SwarmRobustness()
{
    //Destructor
}

// Is from the init function to print when robots will fail
void logMySchedule(int id, int time, bool willFail) {
  if (willFail) {
    argos::LOG << "robotId = " << id << "    time = " << time << " ticks" << std::endl;
  }
}

void logProgress(int numTicks, int myId, float x, float y) {
  if(myId == 0) {
    argos::LOG << "" << std::endl;
    argos::LOG << "****************" << std::endl;
    argos::LOG << "*** TICK = " << numTicks << " ***" << std::endl;
    argos::LOG << "****************" << std::endl;
  }
  argos::LOG << "myId[" << myId << "] x: " << x << " y: " << y << std:: endl;
}

bool robotIsFailed(int robotId, int numTicks) {
   if (robotFailList[robotId] == true && robotFailTime[robotId] <= numTicks) {
      return true;
   }
   return false;
}

void SwarmRobustness::ControlStep()
{
  // myId and numTicks++ lines must stay as first in order to work.
  int myId = std::stoi (GetId().substr(2),nullptr);
  if (myId == 0) numTicks ++;

  // If applicable, log robot positioning.
  if (numTicks % logFrequency == 0) {
    float x = m_pcPosSens->GetReading().Position.GetX();
    float y = m_pcPosSens->GetReading().Position.GetY();
    logProgress(numTicks, myId, x, y);
  }
   //Create Wheel Data Object
   DATA wheeldata;
   // When a robot fails perform its corresponding failure case.
   if (robotIsFailed(myId, numTicks)) {
      switch(failure_mode)
      {
      case MOTOR_FAILURE : // Failure of a robot's motors only
         m_pcWheels->SetLinearVelocity(0, 0);
         return;
         break;
      case POWER_FAILURE : // Complete failure of individual robot
         m_pcWheels->SetLinearVelocity(0, 0);
         RABAFailure();
         return;
         break;
      case SENSOR_FAILURE : // Failure of a robot's IR sensors. Wanders around lost
         SensorFailure();
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
    obstsense.distance = m_pcProximity->GetReadings()[0]+rand->Gaussian(0.0, 0.1);
    obstsense.sensorID = 0;

    //Determine which sensor has a closer obstacle
    if(obstsense.distance < m_pcProximity->GetReadings()[1]+rand->Gaussian(0.0, 0.1))
    {
       obstsense.distance = m_pcProximity->GetReadings()[1];
       obstsense.sensorID = 1;
    }

    if(obstsense.distance < m_pcProximity->GetReadings()[7]+rand->Gaussian(0.0, 0.1))
    {
       obstsense.distance = m_pcProximity->GetReadings()[7];
       obstsense.sensorID = 7;
    }

    if(obstsense.distance < m_pcProximity->GetReadings()[6]+rand->Gaussian(0.0, 0.1))
    {
       obstsense.distance = m_pcProximity->GetReadings()[6];
       obstsense.sensorID = 6;
    }

    //Create ObstController
    Obstacle obstController;
    obstController.SetAvoidanceRadius(BeaconInSight());

    //React to obstacle
    if(obstController.ShouldAvoid(&obstsense))
    {

        time_since_collision = 0;

        if(obstsense.turnRight)
        {
            wheeldata.rwVel = 5.0;
            wheeldata.lwVel = 0;
        }
        else
        {
            wheeldata.rwVel = 0;
            wheeldata.lwVel = 5.0;
        }
    }
    else
    {
        time_since_collision++;
        // Check to see if we need to flock back

        /*-----FLOCKING CODE-----*/

        float tolerance = 50;

        // if dist >= tolerance
        if(flocking)
        {

            //argos::LOG << "FLOCKING" << std::endl;

            // Get Error
            CRadians errorBackToSwarm = GetSwarmBearing();

            // Determine if we are turning right or left
            if(!pickedDirection)
            {
                if(errorBackToSwarm > CRadians(0.0))
                {
                    turningRight = true;
                }
                else
                {
                    turningRight = false;
                }

                pickedDirection = true;
            }

            if(pickedDirection)
            {
                if(turningRight)
                {
                    // RIGHT
                    wheeldata.rwVel = 5.0;
                    wheeldata.lwVel = 0.0;
                }
                else
                {
                    // LEFT
                    wheeldata.lwVel = 0.0;
                    wheeldata.rwVel = 5.0;
                }
            }

            // check error, if below, turn flocking behavior off
            if(errorBackToSwarm <= CRadians(0.05) && errorBackToSwarm >= CRadians(-0.05))
            {
                // Stops Flocking Behavior
                flocking = false;

                // Resets code to choose which direction to turn
                pickedDirection = false;

                wheeldata.lwVel = 5.0;
                wheeldata.rwVel = 5.0;

                time_since_collision = 0;
            }

        }
        else if(time_since_collision >= tolerance)
        {
            flocking = true;
        }

        //otherwise drive straight
        else
        {
            wheeldata.lwVel = 5.0;
            wheeldata.rwVel = 5.0;
        }
    }

                                   //rightwheel, leftwheel
    m_pcWheels->SetLinearVelocity(wheeldata.rwVel, wheeldata.lwVel);
}

void SwarmRobustness::Destroy() {   }

void SwarmRobustness::Init(TConfigurationNode& t_node)
{
  // m_pcPosAct    = GetActuator<CCI_QuadRotorPositionActuator>("quadrotor_position");
  m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity");
  m_pcLight     = GetSensor  <CCI_LightSensor                 >("light");
  m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing");
  m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing");
  m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"       );
  // CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

  // m_cFootBoot
  // argos::LOG << "pos: " << GetEmbodiedEntity().GetOriginAnchor().Position.GetX() << std::endl;
  rand = argos::CRandom::CreateRNG("argos");
  int failureRate;
  int numRobots;
  int maxFail;
  GetNodeAttributeOrDefault(t_node, "failure_rate", failureRate, 0);
  GetNodeAttributeOrDefault(t_node, "num_robots", numRobots, 0);
  GetNodeAttributeOrDefault(t_node, "max_fail", maxFail, 0);
  GetNodeAttributeOrDefault(t_node, "log_frequency", logFrequency, 0);

  // Assign which robots will fail and when if applicable
  if (!hasScrambled) {
    int i = 0;
    int time = -1;
    double numToFail = numRobots * ((double)failureRate/100);
    
    if (numToFail == 0) argos::LOG << "No robots selected to fail." << std::endl;
    else {
      argos::LOG << "numToFail: " << numToFail << std::endl;
      argos::LOG << "printing list of which and when robots will fail" << std::endl;
    }

    // Edge case if user selected 5 robots with 10% failure rate
    if(fmod(numToFail,1) != 0) {
      // argos::LOG << "modulus: " << fmod(numToFail,1)  << std::endl;
      int fail = rand->Uniform(argos::CRange<int>(0,100));
      if (fail <= (int)fmod(numToFail,1)*100) {
        time = rand->Uniform(argos::CRange<int>(0,maxFail));
        robotFailTime[i] = time;
        robotFailList[i] = true;
      }
      logMySchedule(i, robotFailTime[i], robotFailList[i]);
      i++;
    }

    // Set fail time for applicable robots
    while(i < numToFail) {
      time = rand->Uniform(argos::CRange<int>(0,maxFail));
      robotFailTime[i] = time;
      robotFailList[i] = true;
      logMySchedule(i, time, true);
      i++;
    }

    // Continue initializing other bots
    while(i<numRobots){
      robotFailTime[i] = robotMaxTime;
      robotFailList[i] = false;
      logMySchedule(i, robotMaxTime, false);
      i++;
    }
    if (numToFail > 0) argos::LOG << "finished printing list" << std::endl;
    hasScrambled = true;
  } //end (if (!hasScrambled))
  numTicks = 0; // Must be set to 0 because how it is incremented in ControlStep

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

bool SwarmRobustness::BeaconInSight()
{
   if(failed == SENSOR_FAILURE)
   {
      return false;
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
      return true;
   }
   else
   {
      return false;
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
      if(packet.Data[0] == POWER_FAILURE)
      {
         continue; // skip robots that are showing power failure.
      }
      sum += packet.HorizontalBearing;
   }

   // return the average bearing to all readings
   return sum / readings.size();
}

void SwarmRobustness::SensorFailure()
{
   failed = SENSOR_FAILURE;
}

void SwarmRobustness::RABAFailure()
{
   m_pcRABA->SetData(0, POWER_FAILURE);
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(SwarmRobustness, "swarm_robustness_controller")
