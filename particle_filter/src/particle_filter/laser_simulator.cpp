/*
 * Date:      2020-11-10
 * Author:    Miroslav Kulich, Gaël Écorchard
 */

#include <cmath>  // For M_PI,std::{abs,cos,sin,sqrt}.
#include <cstdint>  // For size_t.
#include <utility>  // For std::swap.

#include <tf2/LinearMath/Quaternion.h>  // For tf2::Quaternion.
#include <tf2/LinearMath/Matrix3x3.h>  // For tf2::Matrix3x3.
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // For tf2::fromMsg.

#include <particle_filter/typedefs.h>

#include <particle_filter/laser_simulator.h>

namespace particle_filter
{

const int g_default_ray_count{37};
const double g_default_ray_min_angle{-M_PI};
const double g_default_ray_max_angle{M_PI};
const double g_default_ray_max_range{5};

inline
double yawFromPose(const geometry_msgs::Pose & pose)
{
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3 t{q};
  double roll;
  double pitch;
  double yaw;
  t.getRPY(roll, pitch, yaw);
  return yaw;
}

LaserSimulator::LaserSimulator(const nav_msgs::OccupancyGrid & map)
{
  ros::NodeHandle private_nh{"~"};

  int ray_count = g_default_ray_count;
  if (!private_nh.getParam("ray_count", ray_count))
  {
    ROS_WARN_STREAM("Parameter " << private_nh.getNamespace() << "/ray_count not set, setting to " << g_default_ray_count);
  }

  if (ray_count < 2)
  {
    ROS_WARN_STREAM("Parameter " << private_nh.getNamespace() << "/ray_count must be > 2, forced to 2");
    ray_count = 2;
  }

  double ray_min_angle = g_default_ray_min_angle;
  if (!private_nh.getParam("ray_min_angle", ray_min_angle))
  {
    ROS_WARN_STREAM("Parameter " << private_nh.getNamespace() << "/ray_min_angle not set, setting to " << g_default_ray_min_angle);
    return;
  }

  double ray_max_angle = g_default_ray_max_angle;
  if (!private_nh.getParam("ray_max_angle", ray_max_angle))
  {
    ROS_WARN_STREAM("Parameter " << private_nh.getNamespace() << "/ray_max_angle not set, setting to " << g_default_ray_max_angle);
    return;
  }

  double ray_max_range = g_default_ray_max_range;
  if (!private_nh.getParam("ray_max_range", ray_max_range))
  {
    ROS_WARN_STREAM("Parameter " << private_nh.getNamespace() << "/ray_max_range not set, setting to " << g_default_ray_max_range);
    return;
  }

  laser_config_.count = ray_count;
  laser_config_.max_range = ray_max_range;
  laser_config_.min_angle = ray_min_angle;
  laser_config_.max_angle = ray_max_angle;
  laser_config_.resolution  = (ray_max_angle - ray_min_angle) / static_cast<double>(ray_count - 1);

  if (not setMap(map))
  {
    return;
  }

  initialized_ = true;
}

bool LaserSimulator::setMap(const nav_msgs::OccupancyGrid & map)
{
  initialized_ = false;
  if (std::abs(yawFromPose(map.info.origin)) > 1e-10)
  {
    ROS_ERROR("Map with non-zero orientation not supported yet");
    return false;
  }

  map_width_ = map.info.width;
  map_height_ = map.info.height;
  x0_ = map.info.origin.position.x;
  y0_ = map.info.origin.position.y;
  cell_size_ = map.info.resolution;

  grid_ptr_.reset(new ByteMatrix{static_cast<int>(map_height_), static_cast<int>(map_width_)});
  for (size_t r = 0; r < map_height_; r++)
  {
    for (size_t c = 0; c < map_width_; c++)
    {
      const uint8_t val = map.data[r * map_width_ + c];
      /* FREE=0, OCCUPIED=100, UNKNOWN=255. */
      (*grid_ptr_)(r, c) = (val == 0 ? FREESPACE_CELL : (val == 100 ? OCCUPIED_CELL : UNKNOWN_CELL));
    }
  }

  initialized_ = true;
  return true;
}

/** Return true if the pixel coordinates are on the map.
 */
bool LaserSimulator::onMap(int x, int y)
{
  if (not initialized_)
  {
    ROS_WARN("Cannot compute feasibility, no map received");
    return false;
  }

  return ((x >= 0) and (x < static_cast<int>(map_width_))
      and (y >= 0) and (y < static_cast<int>(map_height_)));
}


sensor_msgs::LaserScan LaserSimulator::getScan(const RobotPosition & pose)
{
  if (not initialized_)
  {
    ROS_WARN("Cannot compute scan, no map received");
    points_.clear();
    return {};
  }

  sensor_msgs::LaserScan scan;
  scan.header.stamp = ros::Time::now();
  scan.angle_min = laser_config_.min_angle;
  scan.angle_max = laser_config_.max_angle;
  scan.angle_increment = laser_config_.resolution;
  scan.range_max = laser_config_.max_range;

  const unsigned int n = laser_config_.count;

  points_.resize(n);
  scan.ranges.resize(n);

  const int rx = gridFromRealX(pose.x);
  const int ry = gridFromRealY(pose.y);
  double angle = pose.phi + laser_config_.min_angle;
  for (unsigned int i = 0; i < n; i++)
  {
    const double fx = pose.x + laser_config_.max_range * std::cos(angle);
    const double fy = pose.y + laser_config_.max_range * std::sin(angle);
    const PointInt pt = bresenham(rx, ry, gridFromRealX(fx), gridFromRealY(fy));
    const double dx = static_cast<double>(rx - pt.x);
    const double dy = static_cast<double>(ry - pt.y);
    scan.ranges[i] = cell_size_ * std::sqrt(dx * dx + dy * dy);
    points_[i].x = realFromGridX(pt.x);
    points_[i].y = realFromGridY(pt.y);

    angle += laser_config_.resolution;
  }

  return scan;
}

PointList LaserSimulator::getRawPoints()
{
  if (not initialized_)
  {
    ROS_WARN("Cannot compute raw points, no map received");
    points_.clear();
  }

  return points_;
}

bool LaserSimulator::isFeasible(const RobotPosition& pos)
{
  if (not initialized_)
  {
    ROS_WARN("Cannot compute feasibility, no map received");
    return false;
  }

  const int x = gridFromRealX(pos.x);
  const int y = gridFromRealY(pos.y);
  if (not onMap(x, y))
  {
    return false;
  }

  return (*grid_ptr_)(y, x) == FREESPACE_CELL;
}

PointInt LaserSimulator::bresenham(int x0, int y0, int x1, int y1)
{
  int dx = x1 - x0;
  int dy = y1 - y0;
  const int steep = (std::abs(dy) >= std::abs(dx));
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
    // Recompute Dx, Dy after swap.
    dx = x1 - x0;
    dy = y1 - y0;
  }
  int xstep = 1;
  if (dx < 0)
  {
    xstep = -1;
    dx = -dx;
  }
  int ystep = 1;
  if (dy < 0)
  {
    ystep = -1;
    dy = -dy;
  }
  const int two_dy = 2 * dy;
  const int two_dy_minus_two_dx = two_dy - 2 * dx;
  int e = two_dy - dx;
  int y = y0;
  int x_draw;
  int y_draw;
  for (int x = x0; x != x1; x += xstep)
  {
    if (steep)
    {
      x_draw = y;
      y_draw = x;
    }
    else
    {
      x_draw = x;
      y_draw = y;
    }
    if ((x_draw <= 0) or (x_draw >= (grid_ptr_->NCOLS - 1))
        or (y_draw <= 0) or (y_draw >= (grid_ptr_->NROWS - 1))
        or ((*grid_ptr_)(y_draw, x_draw) == OCCUPIED_CELL)
        or ((*grid_ptr_)(y_draw, x_draw) == UNKNOWN_CELL)
        )
    {
      return PointInt{x_draw, y_draw};
    }

    // next
    if (e > 0)
    {
      e += two_dy_minus_two_dx; //E += 2*Dy - 2*Dx;
      y = y + ystep;
    }
    else
    {
      e += two_dy; //E += 2*Dy;
    }
  }
  return PointInt{x1, y1};
}

} /* namespace particle_filter */
