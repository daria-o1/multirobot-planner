#include <cmath>  // For NAN, std::isfinite.
#include <cstdint>  // For size_t.
#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <occupancy_grid_utils/coordinate_conversions.h>

#include <robot_coordination/AddPath.h>
#include <robot_coordination/StartMovement.h>
#include <robot_coordination/StopMovement.h>

#include <ros/ros.h>

#include <multi_robot_planner/multi_robot_planner.h>

namespace multi_robot_planner
{

/** Return true whether the point is in free space. */
inline
bool
isFeasible(const nav_msgs::OccupancyGrid & map, geometry_msgs::Point & point)
{
  const occupancy_grid_utils::Cell cell{occupancy_grid_utils::pointCell(map.info, point)};

  return occupancy_grid_utils::withinBounds(map.info, cell)
    and map.data.at(cellIndex(map.info, cell)) < occupancy_grid_utils::OCCUPIED;
}

MultiRobotPlanner::MultiRobotPlanner()
{
  ros::NodeHandle private_nh("~");

  if (not(private_nh.getParam("robot_count", robot_count_)))
  {
    ROS_WARN("Parameter %s/robot_count not set, settting to 1", private_nh.getNamespace().c_str());
  }

  if (robot_count_ < 0)
  {
    ROS_WARN("Parameter %s/robot_count must be strictly positive, settting to 1", private_nh.getNamespace().c_str());
    robot_count_ = 1;
  }

  last_odoms_.resize(static_cast<size_t>(robot_count_));
  odom_subs_.resize(static_cast<size_t>(robot_count_));
  add_path_clients_.resize(static_cast<size_t>(robot_count_));
  start_movement_clients_.resize(static_cast<size_t>(robot_count_));
  stop_movement_clients_.resize(static_cast<size_t>(robot_count_));

  for (unsigned int r = 0; r < static_cast<unsigned int>(robot_count_); ++r)
  {
    const std::string robot_name{std::string{"/turtle"} + std::to_string(r)};
    ros::Subscriber sub{node_handle_.subscribe<nav_msgs::Odometry>(
        robot_name + std::string{"/odom"},
        10,
        [this, r] (const nav_msgs::Odometry::ConstPtr & msg) {return odometryCallback(r, msg);}
        )};
    odom_subs_[r] = sub;
    last_odoms_[r].pose.pose.position.x = NAN;

    add_path_clients_[r] = node_handle_.serviceClient<robot_coordination::AddPath>(
        robot_name + std::string{"/add_path"});
    start_movement_clients_[r] = node_handle_.serviceClient<robot_coordination::StartMovement>(
        robot_name + std::string{"/start_movement"});
    stop_movement_clients_[r] = node_handle_.serviceClient<robot_coordination::StopMovement>(
        robot_name + std::string{"/stop_movement"});
  }

  map_client_ = node_handle_.serviceClient<nav_msgs::GetMap>(std::string{"/static_map"});

  goal_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>(
      std::string{"goal"},
      10,
      [this] (const geometry_msgs::PoseStamped::ConstPtr & goal) {return goalCallback(goal);});
}

bool
MultiRobotPlanner::odometryCallback(unsigned int index, const nav_msgs::Odometry::ConstPtr & msg)
{
  last_odoms_[index] = *msg;
  return true;
}

bool
MultiRobotPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal)
{
  ROS_WARN("goalCallback start");
  goals_.push_back(*goal);

  if (not(stop_robots()))
  {
    return false;
  }

  /* Check that we have as many goals as robots and that the odometry messages
   * were received. */
  if (not(haveAllGoalsAndOdom()))
  {
    return false;
  }

  /* Get the map. */
  if (not(get_map()))
  {
    return false;
  }

  /* Compute the trajectories for all robots. */
  const std::vector<WaypointList> trajectories{plan()};

  goals_.clear();

  if (trajectories.empty())
  {
    ROS_WARN("Could not compute a plan for all robots");
    return false;
  }

  if (trajectories.size() != static_cast<size_t>(robot_count_))
  {
    ROS_ERROR_STREAM("Plan is corrupted, has " << trajectories.size() << " trajectories, should have " << robot_count_);
    return false;
  }

  send_trajectories_to_robots(trajectories);

  ROS_WARN("goalCallback end");
  return true;
}

bool
MultiRobotPlanner::haveAllGoalsAndOdom()
{
  for (auto & odom : last_odoms_)
  {
    if (not(std::isfinite(odom.pose.pose.position.x)))
    {
      ROS_WARN("Received a goal but not all robots have odometry, ignoring goal");
      return false;
    }
  }

  /* Remove old goals. */
  for (auto it = goals_.begin(); it != goals_.end(); ++it)
  {
    if ((ros::Time::now() - it->header.stamp) > timeout_goals_)
    {
      /* Remove old goal. */
      ROS_WARN("Previous goal cleared because of timeout");
      goals_.erase(it);
      return false;
    }
  }

  if (goals_.size() != robot_count_)
  {
    return false;
  }

  return true;
}

bool
MultiRobotPlanner::get_map()
{
  nav_msgs::GetMap srv;
  if (not(map_client_.call(srv)))
  {
    ROS_ERROR_STREAM("Failed to call service " << map_client_.getService() << ", map not available");
    return false;
  }
  map_ = srv.response.map;
  return true;
}

std::vector<MultiRobotPlanner::WaypointList>
MultiRobotPlanner::plan()
{
  std::vector<WaypointList> trajectories;
  trajectories.reserve(static_cast<size_t>(robot_count_));

  /* Build a dummy plan with the goal as unique waypoint for each robot. */
  for (const auto & g : goals_)
  {
    WaypointList trajectory;
    robot_coordination::Waypoint wp;
    wp.pose = g.pose;
    wp.timepoint = ros::Duration{5.0};
    trajectory.push_back(wp);
    trajectories.push_back(trajectory);
  }

  return trajectories;
}

bool
MultiRobotPlanner::send_trajectories_to_robots(const std::vector<MultiRobotPlanner::WaypointList> & trajectories)
{
  /* Connect to the robot_coordination servers and send the plan for each robot. */
  for (size_t r = 0; r < trajectories.size(); ++r)
  {
    robot_coordination::AddPath srv;
    srv.request.waypoints = trajectories[r];
    if (add_path_clients_[r].call(srv))
    {
      ROS_DEBUG_STREAM("Sent planned trajectory to " << add_path_clients_[r].getService());
    }
    else
    {
      ROS_ERROR_STREAM("Failed to call service " << add_path_clients_[r].getService());
      return false;
    }
  }

  /* Start the robots. */
  if (not(start_robots()))
  {
    ROS_WARN("Cannot start all robots");
    return false;
  }

  return true;
}

bool
MultiRobotPlanner::start_robots()
{
  for (size_t r = 0; r < start_movement_clients_.size(); ++r)
  {
    robot_coordination::StartMovement srv;
    if (start_movement_clients_[r].call(srv))
    {
      ROS_DEBUG_STREAM("Started robot " << r << " through service " << start_movement_clients_[r].getService());
    }
    else
    {
      ROS_ERROR_STREAM("Failed to call service " << start_movement_clients_[r].getService());
      return false;
    }
  }
  return true;
}

bool
MultiRobotPlanner::stop_robots()
{
  for (size_t r = 0; r < stop_movement_clients_.size(); ++r)
  {
    robot_coordination::StopMovement srv;
    if (stop_movement_clients_[r].call(srv))
    {
      ROS_DEBUG_STREAM("Stopped robot " << r << " through service " << stop_movement_clients_[r].getService());
    }
    else
    {
      ROS_ERROR_STREAM("Failed to call service " << stop_movement_clients_[r].getService());
      return false;
    }
  }
  return true;
}

} /* namespace multi_robot_planner */
