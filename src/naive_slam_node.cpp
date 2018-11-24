#include "mckinnc_ekf/common.hpp"
#include "mckinnc_ekf/naive_slam.hpp"

#include <chrono>
#include <optional>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

using namespace mckinnc_ekf;

typedef std::chrono::high_resolution_clock Clock;

// ### CONSTANTS ##############################################################

static constexpr double REFRESH_RATE_HZ = 30.0;

static constexpr double MODEL_COVARIANCE_SCALAR = 1e-6;
static constexpr double OBSERVATION_VARIANCE_SCALAR = 1e-4;
static constexpr double UNCERTAINTY_THRESHOLD = 1.0; // TODO: dial this in

static const Covariance MODEL_COVARIANCE
    = Covariance::Constant(MODEL_COVARIANCE_SCALAR);
static const Observation OBSERVATION_VARIANCE = {
      OBSERVATION_VARIANCE_SCALAR,
      OBSERVATION_VARIANCE_SCALAR,
      OBSERVATION_VARIANCE_SCALAR,
    };

// ### GLOBALS ################################################################

static std::optional<Control> control = {};
static std::chrono::time_point<Clock> lastCommandTimePoint;

static std::vector<Observation> observations;
static double likelihoodRadius;

// ### CALLBACKS ##############################################################

static void commandCallback(const geometry_msgs::TwistConstPtr& msg)
{
  // Compute time since the last callback.
  std::chrono::time_point<Clock> now = Clock::now();
  std::chrono::duration<double> duration = now - lastCommandTimePoint;
  double timeDelta = duration.count();
  lastCommandTimePoint = now;
  // If control is None, just pass through the command.
  if (!control) {
    Control c = {
      msg->linear.x,
      msg->angular.z,
      timeDelta,
    };
    control = c;
    return;
  }
  // Multiple commands since last run: compute average.
  double totalTime = control->timeDelta + timeDelta;
  double oldPortion = control->timeDelta / totalTime;
  double newPortion = timeDelta / totalTime;
  control->linear = (oldPortion * control->linear)
      + (newPortion * msg->linear.x);
  control->angular = (oldPortion * control->angular)
      + (newPortion * msg->angular.x);
  control->timeDelta = totalTime;
}

static void laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  // TODO: corner detection
}

// ### MAIN ###################################################################

int main(int argc, char** argv)
{
  ros::init(argc, argv, "naive_slam_node");
  ros::NodeHandle nh;
  NaiveSLAM slam(MODEL_COVARIANCE, OBSERVATION_VARIANCE);

  lastCommandTimePoint = Clock::now();

  nh.subscribe("cmd_vel", 1, commandCallback);
  nh.subscribe("scan", 1, laserCallback);

  ros::Rate rate(REFRESH_RATE_HZ);
  while (ros::ok()) {
    ros::spinOnce();
    if (control && !observations.empty()) {
      slam.run(*control, observations,
          UNCERTAINTY_THRESHOLD, likelihoodRadius);
      control = {};
      observations.clear();
    }
    rate.sleep();
  }
  return 0;
}
