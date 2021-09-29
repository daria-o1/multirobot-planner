#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <robot_coordination/Waypoint.h>

#include <ros/ros.h>

namespace multi_robot_planner
{

class MultiRobotPlanner
{
  public:

    MultiRobotPlanner();

  private:

    using WaypointList = std::vector<robot_coordination::Waypoint>;

    /** Odometry callback for robot with the given index. */
    bool odometryCallback(unsigned int index, const nav_msgs::Odometry::ConstPtr & msg);

    /** One callback for the goals for all robots.
     *
     * The first goal is assigned to the first robot, the second one to the second one, etc.
     * All goals are canceled if not all robots receive their goal within a given timeout. */
    bool goalCallback(const geometry_msgs::PoseStamped::ConstPtr & goal);

    /** Return true if an odometry message has been received from all robots
     * and if each robot has been assigned a goal. */
    bool haveAllGoalsAndOdom();

    /** Get the map through service call. */
    bool get_map();

    /** Plan a trajectory for each robot.
     *
     * The is the main function that you should implement as a student. */
    std::vector<WaypointList> plan();

    /** Send the trajectories to all robot and send visualization information. */
    bool send_trajectories_to_robots(const std::vector<WaypointList> & trajectories);


    bool start_robots();
    bool stop_robots();

    ros::NodeHandle node_handle_;

    int robot_count_{1};  // Linked to static parameter ~robot_count.
    std::vector<nav_msgs::Odometry> last_odoms_;  // Fixed size, one per robot (set to NAN to indicate the absence of message).
    std::vector<ros::Subscriber> odom_subs_;  // Fixed size, one per robot.
    std::vector<ros::ServiceClient> add_path_clients_;  // Fixed size, one per robot.
    std::vector<ros::ServiceClient> start_movement_clients_;  // Fixed size, one per robot.
    std::vector<ros::ServiceClient> stop_movement_clients_;  // Fixed size, one per robot.

    ros::Subscriber goal_sub_;  // Same topic for all robots.
    std::vector<geometry_msgs::PoseStamped> goals_;  // With changing size.
    ros::Duration timeout_goals_{30.0};  // Timeout to reset already clicked goals, in s.

    ros::ServiceClient map_client_;
    nav_msgs::OccupancyGrid map_;
};

} /* namespace multi_robot_planner */
