#include "my_rb1_ros/Rotate.h"
#include "ros/init.h"
#include <boost/mpl/if.hpp>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ostream>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <tf/transform_datatypes.h>

class RotateService {

private:
  ros::NodeHandle nh_;
  ros::Publisher cmdPub_;
  ros::Subscriber odomSub_;
  ros::ServiceServer rotateService_;

  double angleStart_;
  double angleCurr_;
  bool angleSet_;

public:
  RotateService() : angleStart_(0), angleCurr_(0), angleSet_(false) {

    odomSub_ = nh_.subscribe("/odom", 1, &RotateService::odomCB, this);
    cmdPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    rotateService_ =
        nh_.advertiseService("rotate_robot", &RotateService::srvCB, this);
  }

  void odomCB(nav_msgs::Odometry odom) {
    angleCurr_ = tf::getYaw(odom.pose.pose.orientation) * (180.0 / M_PI);
    // std::cout << angleCurr_ << std::endl;
    if (!angleSet_) {
      angleStart_ = angleCurr_; // get starting angle
      angleSet_ = true;
    }
  }

  // for normalizing the angle due to wrapping around -179 to 180.
  double normalizeAngle(double angle) {
    while (angle > 180)
      angle -= 360;
    while (angle <= -180)
      angle += 360;
    return angle;
  }

  // for calculating angle difference
  double angleDifference(double a, double b) {
    double diff = normalizeAngle(a - b);
    return diff;
  }

  // @TODO does not currently handle angles greater than 179
  bool srvCB(my_rb1_ros::Rotate::Request &req,
             my_rb1_ros::Rotate::Response &res) {

    // check if entered angle is valid
    if (req.degrees < -179 || req.degrees > 179) {
      ROS_ERROR("Angle too large!");
      res.result = "Error: Angle too large";
      return false;
    }

    angleSet_ = false;

    double targetAngle = normalizeAngle(angleStart_ + req.degrees);
    double currDifference = angleDifference(angleCurr_, angleStart_);
    double targetDifference = angleDifference(targetAngle, angleStart_);

    geometry_msgs::Twist twist;

    // determine turn direction based on input degrees
    twist.angular.z = (req.degrees > 0) ? -0.05 : 0.05;

    while (true) {
      currDifference = angleDifference(angleCurr_, angleStart_);

      // Check if we have reached target, considering the direction
      if (req.degrees > 0 && currDifference >= targetDifference)
        break;
      if (req.degrees < 0 && currDifference <= targetDifference)
        break;

      cmdPub_.publish(twist);
      ros::spinOnce();
    }

    twist.angular.z = 0;
    cmdPub_.publish(twist);

    // reset the angle
    angleStart_ = angleCurr_;

    res.result = "rb1 rotation completed";
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service");
  RotateService rotateService;
  ros::spin();

  return 0;
}
