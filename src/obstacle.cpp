# include "obstacle.h"

Obstacle::Obstacle()
{
    //constructor
}

bool Obstacle::ShouldAvoid(ObstSensors *sensdat)
{
    /* Do we have an obstacle in front? */
    if(sensdat->distance > 0.0f)
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
