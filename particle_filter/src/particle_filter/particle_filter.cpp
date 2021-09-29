#include <cmath>  // For std::{atan2,cos,exp,sin,sqrt}.
#include <limits>  // For std::numeric_limits.
#include <mutex>  // For std::{lock_guard,mutex}.
#include <random>  // For std::{random_device,default_random_engine,uniform_real_distribution,normal_distribution}.

#include <nav_msgs/GetMap.h>

#include <opencv2/flann.hpp>
#include <opencv2/ml.hpp>

#include <tf/tf.h>

#include <particle_filter/dbscan.h>
#include <particle_filter/kdtree.h>
#include <particle_filter/particle_filter.h>
#include <particle_filter/utils.h>

namespace particle_filter
{

using particle_filter::normalizeAngle;

std::random_device random_device;
std::default_random_engine random_engine{random_device()};
std::uniform_real_distribution<double> uniform_distribution{0, 1};
std::normal_distribution<double> normal_distribution{0, 1};

/** Return one sample of a uniform distribution between min and max
 */
double randBetween(double min, double max)
{
  return min + uniform_distribution(random_engine) * (max - min);
}

/** Return one sample of a normal distribution with the given mean and standard deviation
 */
double nrand(double mean, double sigma)
{
  return mean + normal_distribution(random_engine) * sigma;
}

/** Return the position to which the transform (x0, y0, yaw0) is applied.
 */
inline
RobotPosition transformedPosition(double x, double y, double x0, double y0, double yaw0)
{
  RobotPosition pos;
  const double cos_yaw = std::cos(yaw0);
  const double sin_yaw = std::sin(yaw0);
  pos.x = x0 + x * cos_yaw - y * sin_yaw;
  pos.y = y0 + x * sin_yaw + y * cos_yaw;
  pos.phi = yaw0;
  return pos;
}

ParticleFilter::ParticleFilter()
{
  getMap();

  particles_ = generateRandomParticles(10);
  old_odometry_valid_ = false;
}

/** Publish the particle cloud as MarkerArray for RViz.
 */
void ParticleFilter::publishParticles()
{
  if (particle_publisher_ != nullptr)
  {
    int i = 0;
    visualization_msgs::MarkerArray marker_array;
    for (Particle p : particles_)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "particles";
      marker.id = i++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::MODIFY;
      marker.pose.position.x = p.pos.x;
      marker.pose.position.y = p.pos.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = std::sin(p.pos.phi / 2);
      marker.pose.orientation.w = std::cos(p.pos.phi / 2);
      marker.scale.x = 0.1;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker_array.markers.push_back(marker);
    }
    particle_publisher_->publish(marker_array);
  }
}

/** Get the map from ROS service /static_map
 */
bool ParticleFilter::getMap()
{
  ros::NodeHandle nh;

  ros::ServiceClient static_map_client = nh.serviceClient<nav_msgs::GetMap>("static_map");

  while (not static_map_client.waitForExistence(ros::Duration(5)))
  {
    ROS_WARN_STREAM("Waiting for service /static_map");
  }

  ROS_DEBUG("server found");

  nav_msgs::GetMap get_map_srv;
  if (not static_map_client.call(get_map_srv))
  {
    ROS_ERROR("Failed to call service /static_map");
    return false;
  }
  ROS_DEBUG("server called");

  map_ = get_map_srv.response.map;
  laser_simulator_ptr_.reset(new LaserSimulator{map_});

  ROS_DEBUG("Got a map");

  return true;
}

/** Generate a set of random of equally weighted particles
 *
 * The weight of each particle will be 1 / count.
 */
ParticleVector ParticleFilter::generateRandomParticles(unsigned int count) const
{
  ParticleVector particles;
  particles.reserve(count);
  const double min_x = map_.info.origin.position.x;
  const double max_x = min_x + map_.info.width * map_.info.resolution;
  const double min_y = map_.info.origin.position.y;
  const double max_y = min_y + map_.info.height * map_.info.resolution;
  for (size_t i = 0; i < count; )
  {
    const double x = randBetween(min_x, max_x);
    const double y = randBetween(min_y, max_y);
    const double phi = randBetween(-M_PI, M_PI);
    Particle p;
    p.pos = RobotPosition{x, y, phi}; //RobotPosition(transformedPosition(map_.info.origin.position.x, map_.info.origin.position.y, x, y, phi));
    if (laser_simulator_ptr_->isFeasible(p.pos))
    {
      p.weight = 1.0 / static_cast<double>(count);
      particles.push_back(p);
      i++;
    }
  }
  return particles;
}

ParticleVector ParticleFilter::resampleParticles(ParticleVector particles)
{
  ParticleVector resampled_particles;
  // Extract weights from the particles.
  std::vector<double> weights;
  for (Particle p : particles)
  {
    weights.push_back(p.weight);
  }
  // Initialize the discrete distribution.
  std::discrete_distribution<size_t> distribution(weights.begin(), weights.end());

  // Resample the particles.
  // It is possible to add some newly generated random particles here.
  // It is possible to use a different number of new particles than particles.size().
  for (int i = 0; i < particles.size(); i++)
  {
    const size_t index = distribution(random_engine);
    resampled_particles.push_back(particles[index]);
  }

  return resampled_particles;
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

void ParticleFilter::odometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
  std::lock_guard<std::mutex> guard{latest_odometry_mutex_};
  latest_odometry_.x = msg->pose.pose.position.x;
  latest_odometry_.y = msg->pose.pose.position.y;
  latest_odometry_.phi = tf::getYaw(msg->pose.pose.orientation);
}

double p_hit(double z, double z_star)
{
  const double sigma = 1.0;
  return std::exp((-(z-z_star)*(z-z_star)) / (2.0*sigma)) / (sigma*std::sqrt(2.0*M_PI));
}

double p_short(double z, double z_star)
{
  const double lamda = 1.0;

  if (z < z_star)
  {
    return lamda * std::exp(-z*lamda);
  }
  else
  {
    return 0.0;
  }
}

double p_max(double z, double z_max)
{
  const double dirac_width = 0.0001;

  if (((z_max - dirac_width) < z) and (z <= (z_max + 1.0 - dirac_width)))
  {
    return 1.0;
  }
  else
  {
    return 0.0;
  }
}

// Clustering and EM estimation of clusters' distribution parameters.
RobotPosition ParticleFilter::getPosition()
{
  // Cluster the data.
  std::vector<std::vector<double>> data;
  for (Particle p : particles_)
  {
    data.push_back({p.pos.x, p.pos.y});
  }
  auto dbscan = DBSCAN<std::vector<double>, double> ();
  dbscan.Run(&data, 2, 34.0, 2);
  const auto clusters = dbscan.Clusters;
  for (auto c : clusters)
  {
    std::cout << "Point in cluster: (" << particles_.at(c[0]).pos.x << "," << particles_.at(c[0]).pos.y << ")" << std::endl;
  }
  std::cout << "num of cluster " << static_cast<int>(clusters.size()) << std::endl;

  int nsamples = static_cast<int>(particles_.size());
  int nclusters = static_cast<int>(clusters.size());
  cv::Mat samples(nsamples, 2, CV_64F);
  for (int i = 0; i < nsamples; ++i)
  {
    samples.at<double>(i, 0) = particles_[i].pos.x;
    samples.at<double>(i, 1) = particles_[i].pos.y;
  }
  cv::Mat initial_means(nclusters, 2, CV_64F);
  for (size_t i = 0; i < nclusters; ++i)
  {
    /* The initial mean is the centroid of all points belonging to the current
     * cluster. */
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (auto index : clusters[i])
    {
      sum_x += particles_.at(index).pos.x;
      sum_y += particles_.at(index).pos.y;
    }
    initial_means.at<double>(i, 0) = sum_x / static_cast<double>(clusters[i].size());
    initial_means.at<double>(i, 1) = sum_y / static_cast<double>(clusters[i].size());
  }
  cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();
  em_model->setClustersNumber(nclusters);
  em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_SPHERICAL);
  em_model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 300, 0.1));
  em_model->trainE(samples, initial_means, cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray());
  cv::Mat weights = em_model->getWeights();
  /* I could not understand from the documentation whether classes are sorted.
   * Just take the max, to be on the safe side. */
  double max_weight = std::numeric_limits<double>::lowest();
  int max_weight_index;
  for (int j = 0; j < weights.cols; ++j)
  {
    if (weights.at<double>(0, j) > max_weight)
    {
      max_weight = weights.at<double>(0, j);
      max_weight_index = j;
    }
  }

  cv::Mat means = em_model->getMeans();

  return RobotPosition{means.at<double>(max_weight_index, 0), means.at<double>(max_weight_index, 1), 0.0}; // TODO: yaw
}

void ParticleFilter::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (not old_odometry_valid_)
  {
    std::lock_guard<std::mutex> guard{latest_odometry_mutex_};
    old_odometry_ = latest_odometry_;
    old_odometry_valid_ = true;
    return;
  }

  RobotPosition latest_odometry;
  {
    std::lock_guard<std::mutex> guard{latest_odometry_mutex_};
    latest_odometry = latest_odometry_;
  }

  std::cout << "scan" << std::endl;
  for (const auto & p : particles_)
  {
    sensor_msgs::LaserScan simulated_scan = laser_simulator_ptr_->getScan(p.pos);
    simulated_scan.header.frame_id = "odom";
    // put the sensor model here
    std::cout << "simul scan " << simulated_scan.ranges.size() << std::endl;
    std::cout << "ranges : ";
    for (auto r : simulated_scan.ranges)
    {
      std::cout << "\t" << r;
    }
    std::cout <<std::endl;
    laser_publisher_->publish(simulated_scan);  // DEBUGGING
  }

  // put the resampling here

  // Apply th motion model.
  /* Shown here with the old particle set but should be applied to the new particle set. */
  for (auto & p : particles_)
  {
    p.pos = motionModel(old_odometry_, latest_odometry, p.pos);
    std::cout <<"\t" << p.pos.x <<  " " << p.pos.y << std::endl;
  }

  publishParticles();

  old_odometry_ = latest_odometry;
  old_odometry_valid_ = true;
}

} /* namespace particle_filter */
