# include "obstacle.h"
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <math.h>

Obstacle::Obstacle()
{
    //constructor
}

void Obstacle::SetAvoidanceRadius(bool beacon_in_sight)
{
   if(beacon_in_sight)
   {
      avoidance_radius = BASE_RADIUS + AVOIDANCE_SHIFT;
   }
   else
   {
      avoidance_radius = BASE_RADIUS;
   }
}

bool Obstacle::ShouldAvoid(ObstSensors *sensdat)
{
    /* Do we have an obstacle in front? */
   if(abs(log(sensdat->distance)) <= avoidance_radius)
    {
      /* Yes, we do: avoid it */
      if(sensdat->sensorID == 0 || sensdat->sensorID == 1)
      {
        /* The obstacle is on the left, turn right */
        sensdat->turnRight = true;
      }
      else
      {
        /* The obstacle is on the left, turn right */
        sensdat->turnRight = false;
      }

      //returns yes see obstacle
      return true;

    }
    //returns no obstacle seen
    return false;
}
