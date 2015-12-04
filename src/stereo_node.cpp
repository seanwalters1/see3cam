#include <ros/ros.h>
#include <nodelet/loader.h>

#include "see3cam/stereocamera.h"

int main (int argc, char **argv) {
  ros::init(argc, argv, "see3cam_stereo");

  see3cam::StereoCamera stereo(ros::NodeHandle(), ros::NodeHandle("~"));

  ros::spin();
  return 0;
}

