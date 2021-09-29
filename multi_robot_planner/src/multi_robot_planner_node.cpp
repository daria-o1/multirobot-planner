#include <ros/ros.h>

#include <multi_robot_planner/multi_robot_planner.h>

int main(int argc, char * argv[])
{

  ros::init(argc, argv, "multi_robot_planner");

  multi_robot_planner::MultiRobotPlanner planner;

  ros::AsyncSpinner spinner{2};
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
