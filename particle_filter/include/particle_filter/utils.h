#pragma once

#include <cmath>  // For M_PI.

namespace particle_filter
{

/** Return the angle within ]-pi, pi].
 */
inline
double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2*M_PI;
  while (angle <= -M_PI) angle += 2*M_PI;
  return angle;
}

} /* namespace particle_filter */
