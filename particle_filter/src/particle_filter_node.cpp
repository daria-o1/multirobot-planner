#include <cstdlib>  // For EXIT_FAILURE, EXIT_SUCCESS, std::exit.

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

#include <tf/tf.h>

#include <particle_filter/ground_truth.h>
#include <particle_filter/particle_filter.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "particle_filter");

  ros::NodeHandle private_nh("~");

  particle_filter::ParticleFilter filter;
  /* Get the map. */
  ros::spinOnce();

  ros::Subscriber odom_sub = private_nh.subscribe("odom", 1, &particle_filter::ParticleFilter::odometryCallback, &filter);
  ros::Subscriber laser_sub = private_nh.subscribe("scan", 1, &particle_filter::ParticleFilter::laserScanCallback, &filter);
  ros::Publisher particle_marker_array = private_nh.advertise<visualization_msgs::MarkerArray>("particles", 1, false);
  ros::Publisher laser_pub = private_nh.advertise<sensor_msgs::LaserScan>("simul", 1, false);
  filter.setParticlePublisher(&particle_marker_array);
  filter.setLaserPublisher(&laser_pub);

  /* Example of use of GroundTruth. Call getPose() to get the latest pose. */
  particle_filter::GroundTruth ground_truth;

  ros::Rate rate{1};
  while (ros::ok())
  {
    ros::spinOnce();

    geometry_msgs::PoseStamped gt{ground_truth.getPose()};
    ROS_INFO_STREAM("Ground truth (x, y, yaw): "
        << "(" << gt.pose.position.x
        << ", " << gt.pose.position.y
        << ", " << tf::getYaw(gt.pose.orientation)
        << ")");

    rate.sleep();
  }

  std::exit(EXIT_SUCCESS);
}
