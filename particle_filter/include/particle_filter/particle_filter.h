#pragma once

#include <memory>  // For std::unique_ptr.
#include <mutex>  // For std::mutex.

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/MarkerArray.h>

#include <particle_filter/laser_simulator.h>
#include <particle_filter/typedefs.h>

namespace particle_filter
{

class ParticleFilter
{
  public:

    ParticleFilter();

    ParticleVector generateRandomParticles(unsigned int count) const;

    /** Callback function where the motion model should be implemented. It is
     * called every time, when an update of the motion is received.
     *
     * @param[in] msg constant pointer to navigation message Odometry, where the
     *                motion of the robot is stored.
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg);
    
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg); //< the callback function where the sensor model should be implemented. It is called every time, the new laser sensor reading is received. @param[in] constant pointer to sensor message LaserScan, where the actual scan is stored.
    void setParticlePublisher( ros::Publisher* p) {particle_publisher_ = p;} //< setter for vizualization of particle in rviz
    void setLaserPublisher( ros::Publisher* s) {laser_publisher_ = s;} //< setter for vizualization of particle in rviz
    RobotPosition getPosition(); //< get the estimation of the robot position from the particle vector

  protected:

    bool getMap();
    ParticleVector resampleParticles(ParticleVector particles); //< generates vector of particles' indexes randomly selected according their weights;

    void publishParticles();   

    nav_msgs::OccupancyGrid map_;
    ros::Publisher* particle_publisher_;
    ros::Publisher* laser_publisher_;

  private:

    std::unique_ptr<LaserSimulator> laser_simulator_ptr_;
    ParticleVector particles_;
    RobotPosition latest_odometry_;
    std::mutex latest_odometry_mutex_;
    RobotPosition old_odometry_;  //< Odometry of the previous call to laserScanCallback.
    bool old_odometry_valid_;
};

} /* namespace particle_filter */
