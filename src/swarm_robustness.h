#ifndef SWARM_ROBUSTNESS_H
#define SWARM_ROBUSTNESS_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
/* Definition of the light sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
/* Argos RNG */
#include <argos3/core/utility/math/rng.h>

#include "data.h"
#include "obstacle.h"

using namespace argos;

class SwarmRobustness : public CCI_Controller
{
public:
   SwarmRobustness();
   virtual ~SwarmRobustness();
   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_brownian_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy();

private:

   enum FailureMode
   {
      NO_FAILURE = 0, /* Robots magically do not fail      */
      MOTOR_FAILURE,  /* failure of a robot's motors only  */
      POWER_FAILURE,  /* complete failure of a robot       */
      SENSOR_FAILURE  /* failure of a robot's sensors only */
   } failure_mode;
   FailureMode failed = NO_FAILURE;

   CRandom::CRNG *rand;
   ObstSensors obstsense;

   // Pick a direction to turn to mitigate osillation
   bool pickedDirection = false;
   // Which direction they chose to turn
   bool turningRight = false;

   /* indicates whether the beacon has been detetected by the lignt
      sensor*/
   bool beacon_detected;
   const int turn_threshold = 100; // TODO: define this as a time

   // Determine if the robot is performing flocking behavior
   bool flocking = false;
   int time_since_collision = 0;

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the e-puck proximity sensor */
   CCI_ProximitySensor* m_pcProximity;
   /* Pointer to the light sensor */
   CCI_LightSensor* m_pcLight;
   /* Pointer to the range and bearing sensor/actuator */
   CCI_RangeAndBearingSensor   *m_pcRABS;
   CCI_RangeAndBearingActuator *m_pcRABA;
   /* Pointer to the range-and-bearing sensor */
   CCI_PositioningSensor* m_pcPosSens;
   /**
    * Check the light sensor beacon to determine if the beacon is in
    * sight.
    */
   bool BeaconInSight();
   CRadians GetSwarmBearing();
   int TimeSinceLastAvoidanceCall();
   /**
    * Set up sensors to simulate sensor failure.
    */
   void SensorFailure();
   /**
    * Simulate a RABA Failure. This is used for the power failure
    * case, to signal to other robots that the failed robot should be
    * ignored.
    */
   void RABAFailure();
};
#endif
