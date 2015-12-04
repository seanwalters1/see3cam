#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "see3cam/stereocamera.h"

namespace see3cam {

class StereoNodelet : public nodelet::Nodelet {
  public:
    StereoNodelet() {}

    void onInit() {
      ros::NodeHandle node = getNodeHandle();
      ros::NodeHandle pnode = getPrivateNodeHandle();

      stereo = new StereoCamera(node, pnode);
    }

    ~StereoNodelet() {
      if (stereo) delete stereo;
    }

  private:
    StereoCamera *stereo;
};

};

PLUGINLIB_DECLARE_CLASS(see3cam, StereoNodelet, see3cam::StereoNodelet, nodelet::Nodelet);

