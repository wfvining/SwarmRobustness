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

private:

};

#endif
