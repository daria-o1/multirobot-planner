/**
 * Particule filter localization - Motion Model
 *
 * Skeleton code for teaching BE3M33MKR/B3M33MKR
 * Czech Technical University
 * Faculty of Electrical Engineering
 * Intelligent and Mobile Robotics Group
 *
 * Authors:
 * - Gaël Écorchard <gael.ecorchard@cvut.cz>
 *
 * Licence: MIT (see LICENSE file)
 **/

#include <cstdlib>  // For EXIT_SUCCESS.
#include <random>
#include <cmath>  // For std::{atan2,sqrt}.

#include <particle_filter/gui_motion_model.h>
#include <particle_filter/typedefs.h>
#include <particle_filter/utils.h>

using particle_filter::RobotPosition;
using particle_filter::normalizeAngle;

std::random_device g_rd;
std::default_random_engine g_eng(g_rd());
std::uniform_real_distribution<double> g_uniform_distribution(0.0, 1.0);
std::normal_distribution<double> g_normal_distribution(0.0, 1.0);

/* Return one sample of a uniform distribution between min and max
 */
double rand(double min = 0.0, double max = 1.0)
{
  return min + g_uniform_distribution(g_eng) * (max - min);
}

/* Return one sample of a normal distribution with the given mean and standard deviation
 */
double nrand(double mean=0, double sigma=1)
{
  return mean + g_normal_distribution(g_eng) * sigma;
}

/** Return the Euclidean distance between two points
 */
double dist(const RobotPosition & from, const RobotPosition & to)
{
  const double dx{to.x - from.x};
  const double dy{to.y - from.y};
  return std::sqrt(dx*dx + dy*dy);
}

/** Return the angle of the vector defined by two points
 */
double angle(const RobotPosition & from, const RobotPosition & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

RobotPosition motionModel(RobotPosition robot_previous, RobotPosition robot_current, RobotPosition particle_position)
{
  constexpr double alpha1 = 0.005;
  constexpr double alpha2 = 0.005;

  /* Naïve implementation. */
  const double x_prime = particle_position.x + robot_current.x - robot_previous.x + nrand(0.0, alpha1);
  const double y_prime = particle_position.y + robot_current.y - robot_previous.y + nrand(0.0, alpha1);
  const double phi_prime = normalizeAngle(robot_current.phi - robot_previous.phi + particle_position.phi + nrand(0.0, alpha2));

  return RobotPosition{x_prime, y_prime, phi_prime};
}

int main(int /* argc */, char** /* argv */)
{
  particle_filter::GuiMotionModel gui;

  gui.displayMotionModel(motionModel);
  gui.startInteractor();

  gui.screenshot("/tmp/motion_model.png");

  return EXIT_SUCCESS;
}

