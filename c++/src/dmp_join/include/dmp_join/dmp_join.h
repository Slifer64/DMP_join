/**
 * Copyright (C) 2017 as64_
 */

#ifndef DMP_JOIN_H
#define DMP_JOIN_H

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <memory>
#include <random>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <string>
#include <thread>
#include <csignal>
#include <mutex>

#include <armadillo>
#include <Eigen/Dense>

#include <dmp_join/utils.h>

#include <dmp_join/GUI/GUI.h>
//#include <dmp_join/LogData.h>
#include <dmp_join/Robot/Robot.h>
#include <dmp_join/Controller/Controller.h>

//using namespace as64_;

class DmpJoin
{
public:
  DmpJoin();
  ~DmpJoin();

  void execute();

private:
  ros::NodeHandle n;

  std::shared_ptr<GUI> gui;
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Controller> controller;

  void gotoStartPose();
};

#endif // DMP_JOIN_H
