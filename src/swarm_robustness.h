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
   /* indicates whether the beacon has been detetected by the lignt
      sensor*/
   bool beacon_detected;

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the e-puck proximity sensor */
   CCI_ProximitySensor* m_pcProximity;
   /* Pointer to the light sensor */
   CCI_LightSensor* m_pcLight;

   /**
    * Check light sensor readings to determine if the beacon is in
    * sight.
    */
   void BeaconInSight();
};
#endif