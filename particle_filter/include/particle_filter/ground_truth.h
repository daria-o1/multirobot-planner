/** Provides the ground-truth pose obtained from tf.
 *
 * Call getPose() to get the latest pose or getPose(at) to get the pose at time at (as ros::Time).
 *
 * Parameter
 * - ~robot_frame: Robot frame providing the ground truth.
 *
 *  Optional parameter
 *  - ~world_frame: Parent of ~robot_frame. Defaults to "world".
 */
#pragma once

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

namespace particle_filter
{

class GroundTruth
{
  public:

    GroundTruth();

    bool isInitialized() const {return is_initialized_;}
    geometry_msgs::PoseStamped getPose(ros::Time at=ros::Time{0});

  private:

    bool is_initialized_{false};
    std::string world_frame_{"world"};
    std::string ground_truth_frame_;
    tf2_ros::TransformListener transform_listener_;
    tf2_ros::Buffer tf_buffer_;
};

} /* namespace particle_filter */
