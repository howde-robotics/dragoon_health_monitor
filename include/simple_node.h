// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

// always add include guard for all .h files
#pragma once

// std libraries
#include <iostream>
#include <vector>
#include <cmath>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/tf.h>

// Include your main algorithm class
#include "myAlgo.h"

/**
 * @brief Explain briefly what this class does, e.g. it takes in this input, does this process and output this
 *
 * This part is now the detailed explanation of the class, you can write entire length essays (highly encouraged btw)
 * Explain what algo you use, any important details
 *
 * @see PythonNode
 * @return publish this topic of type
 */
class SimpleNode
{
public:
  SimpleNode();
  ~SimpleNode() { delete algo; }

private:
  howde::MyAlgo* algo;
  /////////////////////// State variables ///////////////////////

  /**
   * @brief enum of vehicle state for switching between teleop control
   */
  enum class MyVehState
  {
    kStateZero,  ///< zero does what
    kStateOne,   ///< one is this
    kStateTwo,   ///< two is hello
  };

  /// the state of the vehicle at any given moment
  MyVehState vehicleState_;
  /// current velocity of the vehicle in m/s
  double currVel_;
  /// current IMU data of the vehicle, contains only gyroscope (for example)
  sensor_msgs::Imu currImu_;

  /////////////////////// calculation variables ///////////////////////

  int myVar1_;     ///< explain explain, unit is bla
  int myVar3_;     ///< don't be lazy, unless it is super self-explanatory, unit is bla
  double myVar2_;  ///< it's just one line, pls say smth about the variable, unit is bla
  double myVar4_;  ///< if you look at source code, checkout the different way to comment, doxygen reads it differently
  geometry_msgs::TwistWithCovarianceStamped vehicleCmdMsg_;  ///< output of this node, which is twist command
  std_msgs::String vehicleStatsMsg_;  ///< output of this node, publish vehicl e status for telemetry

  // put your most important function and other helper functions here
  /**
   * @brief main loop over the ros timer that does this, process what, and does whaaat
   *
   * talk more here plsss
   * Seriously, if this is your most important function you better be writing essays here
   *
   * @param currVel_ current velocity data from odom callback, m/s
   * @param imu current imu data from sensor callback, radian and m/s^3
   * @param vehicleCmdMsg output of this node, twist command to vehicle
   * @param myVar1 bla bla, metre
   * @param myVar2 bla bla, secs^2
   *
   * @note It can be weird that you are putting function param here that is basically the same as the class member
   * variable, but this is good practice because the params of a function gives the reader an intuitive sense of what
   * the function does
   */
  void run(double currVel_, sensor_msgs::Imu imu, geometry_msgs::TwistWithCovarianceStamped& vehicleCmdMsg, int myVar1,
           double myVar4);

  // ROS and node related variables
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber odomSub_, imuSub_;
  ros::Publisher vehicleCmdPub_, vehicleStatsPub_;
  ros::Timer timer_;
  double timerFreq_;

  // Boiler plate ROS functions
  void initRos();
  void timerCallback(const ros::TimerEvent& e);
  void odomCallback(const nav_msgs::Odometry& odomMsg);

  /**
   * @brief typicall there's no need to comment on callbacks if they are just parsing in values
   *
   * But if you do complicated things in the callbacks, please comment
   * But But best practice is that if you need to run complicated things in callbacks
   * please add another function in the callbacks, e.g. creating a run() function
   *
   * @param imuMsg
   */
  void imuCallback(const sensor_msgs::Imu& imuMsg);
};
