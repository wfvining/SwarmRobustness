#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <argos3/core/utility/math/rng.h>

#include "data.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

class Obstacle
{

public:

  //Constructor
  Obstacle();

  bool ShouldAvoid(ObstSensors *data);
  void Avoid(DATA *info);
  void SetAvoidanceRadius(bool beacon_in_sight);

private:
  const Real BASE_RADIUS = 0.01f;
   const Real AVOIDANCE_SHIFT = 0.08f;
  Real avoidance_radius = BASE_RADIUS;
};

#endif
