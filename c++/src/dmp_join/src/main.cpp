/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <dmp_join/dmp_join.h>
#include <dmp_join/utils.h>

int main(int argc, char** argv)
{
  #ifdef CATCH_EXCEPTIONS
  try{
  #endif

  // Initialize the ROS node
  ros::init(argc, argv, "dmp_join");

  // =========  Create controller instance for the ur10 robot  =========
  std::unique_ptr<DmpJoin> controller(new DmpJoin());

  // =========  Main loop running the controller  =========
  controller->execute();

  std::cout << "[MAIN]: Exited loop...\n";

  ROS_INFO_STREAM("dmp_join node is going down.");

  // Shutdown ROS node
  ros::shutdown();

  #ifdef CATCH_EXCEPTIONS
  }
  catch(std::exception &e)
  {
    std::cerr << PRINT_ERROR_MSG(e.what());
  }
  #endif

  return 0;
}
