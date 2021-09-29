#include <cmath>  // For NAN.

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>

#include <particle_filter/ground_truth.h>

namespace particle_filter
{

GroundTruth::GroundTruth() :
  transform_listener_{tf_buffer_}
{
  ros::NodeHandle private_nh("~");

  if (not private_nh.getParam("ground_truth_frame", ground_truth_frame_))
  {
    ROS_ERROR_STREAM("Parameter " << private_nh.getNamespace() << "/ground_truth_frame must be set, exiting");
    return;
  }

  if (not private_nh.getParam("world_frame", world_frame_))
  {
    ROS_WARN_STREAM("Parameter " << private_nh.getNamespace() << "/world_frame not be set, defaulting to \"" << world_frame_ << "\"");
  }

  /* Allow to fill tf_buffer. */
  ros::Rate{10}.sleep();

  is_initialized_ = true;
}


geometry_msgs::PoseStamped
GroundTruth::getPose(ros::Time at)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = NAN;
  pose.pose.position.y = NAN;
  pose.pose.position.z = NAN;
  pose.pose.orientation.w = NAN;
  pose.pose.orientation.x = NAN;
  pose.pose.orientation.y = NAN;
  pose.pose.orientation.z = NAN;

  if (not(isInitialized()))
  {
    ROS_WARN("Uninitialized");
    return pose;
  }

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(world_frame_, ground_truth_frame_, at);
  }
  catch (tf2::TransformException & e)
  {
    ROS_WARN("%s", e.what());
    return pose;
  }

  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;

  return pose;
}

} /* namespace particle_filter */
