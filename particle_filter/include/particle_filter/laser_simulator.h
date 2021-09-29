/*
 * Date:      2020-07-10
 * Author:    Miroslav Kulich, Gaël Écorchard
 */

#pragma once

#include <cmath>  //For std::round.
#include <cstdint>  // For uint8_t.
#include <memory>  // For std::unique_ptr.
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <particle_filter/matrix_utils.h>
#include <particle_filter/typedefs.h>

namespace particle_filter
{

struct LaserConfig
{
  unsigned int count;
  double max_range;
  double resolution;
  double min_angle;
  double max_angle;
};

struct PointInt
{
  int x;
  int y;
};

class LaserSimulator
{
  public:

    LaserSimulator(const nav_msgs::OccupancyGrid & map);

    bool setMap(const nav_msgs::OccupancyGrid & map);
    bool onMap(int x, int y);
    sensor_msgs::LaserScan getScan(const RobotPosition& pose);
    PointList getRawPoints();
    bool isFeasible(const RobotPosition & pos);

  private:

    enum CellState {FREESPACE_CELL, OCCUPIED_CELL, UNKNOWN_CELL};

    int gridFromRealX(double x) {return std::round((x - x0_) / cell_size_);};
    int gridFromRealY(double y) {return std::round((y - y0_) / cell_size_);};
    double realFromGridX(int x) {return x0_ + x * cell_size_;};
    double realFromGridY(int y) {return y0_ + y * cell_size_;};
    PointInt bresenham(int x0, int y0, int x1, int y1);

    bool initialized_{false};

    LaserConfig laser_config_;
    std::unique_ptr<ByteMatrix> grid_ptr_;
    size_t map_width_;
    size_t map_height_;
    double cell_size_;
    double x0_;  /* Map origin (lower-left corner). */
    double y0_;
    PointList points_;
};

} /* namespace particle_filter */
