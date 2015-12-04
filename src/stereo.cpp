#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

#include "see3cam/stereocamera.h"

#include <dynamic_reconfigure/server.h>
#include <see3cam/cameraParamsConfig.h>

using namespace sensor_msgs;

static inline void RotateRgb(unsigned char *dst_chr, unsigned char *src_chr, int pixels) {
    struct pixel_t {
      unsigned char r, g, b;
    };
    struct pixel_t *src = (pixel_t *) src_chr;
    struct pixel_t *dst = &(((pixel_t *) dst_chr)[pixels - 1]);

    for (int i = pixels; i != 0; --i) {
      *dst = *src;
      src++;
      dst--;
    }
}

static inline void RotateYuv(unsigned char *dst_chr, unsigned char *src_chr, int pixels) {
    struct pixel_t {
      unsigned char u, y, v, yy;
    };
    struct pixel_t *src = (pixel_t *) src_chr;
    struct pixel_t *dst = &(((pixel_t *) dst_chr)[(pixels / 2) - 1]);

    for (int i = pixels / 2; i != 0; --i) {
      *dst = *src;
      src++;
      dst--;
    }
}

static inline void RotateMono(unsigned char *dst_chr, unsigned char *src_chr, int pixels) {
    struct pixel_t {
      unsigned char g;
    };
    struct pixel_t *src = (pixel_t *) src_chr;
    struct pixel_t *dst = &(((pixel_t *) dst_chr)[pixels - 1]);

    for (int i = pixels; i != 0; --i) {
      *dst = *src;
      src++;
      dst--;
    }
}

namespace see3cam {

StereoCamera::StereoCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh) :
  node(comm_nh), pnode(param_nh), it(comm_nh),
  left_info_mgr(ros::NodeHandle(comm_nh, "left"), "left_camera"),
  right_info_mgr(ros::NodeHandle(comm_nh, "right"), "right_camera") {

  /* default config values */
  width = 1280;
  height = 960;
  fps = 20;
  left_device = "/dev/video0";
  right_device = "/dev/video1";
  frame = "camera";
  rotate_left = false;
  rotate_right = false;

  /* set up information managers */
  std::string left_url, right_url;

  pnode.getParam("left/camera_info_url", left_url);
  pnode.getParam("right/camera_info_url", right_url);

  left_info_mgr.loadCameraInfo(left_url);
  right_info_mgr.loadCameraInfo(right_url);

  /* pull other configuration */
  pnode.getParam("left/device", left_device);
  pnode.getParam("right/device", right_device);

  pnode.getParam("fps", fps);

  pnode.getParam("left/rotate", rotate_left);
  pnode.getParam("right/rotate", rotate_right);

  pnode.getParam("width", width);
  pnode.getParam("height", height);

  pnode.getParam("frame_id", frame);

  pnode.getParam("encoding", encoding);
  
  if (encoding == "rgb8")
  {
    mode = uvc_cam::Cam::MODE_RGB;
  }
  else if (encoding == "mono8")
  {
    mode = uvc_cam::Cam::MODE_GRAY;
  }
  else
  {
    encoding = "yuv422";
    mode = uvc_cam::Cam::MODE_YUYV;
  }


  /* advertise image streams and info streams */
  left_pub = it.advertise("left/image_raw", 1);
  right_pub = it.advertise("right/image_raw", 1);

  left_info_pub = node.advertise<CameraInfo>("left/camera_info", 1);
  right_info_pub = node.advertise<CameraInfo>("right/camera_info", 1);

  /* initialize the cameras */
  cam_left =
      new uvc_cam::Cam(left_device.c_str(), mode,
		       width, height, fps);
  cam_right =
      new uvc_cam::Cam(right_device.c_str(), mode,
		       width, height, fps);

  /* and turn on the streamer */
  ok = true;
  image_thread = boost::thread(boost::bind(&StereoCamera::feedImages, this));
}

void StereoCamera::sendInfo(ros::Time time) {
  CameraInfoPtr info_left(new CameraInfo(left_info_mgr.getCameraInfo()));
  CameraInfoPtr info_right(new CameraInfo(right_info_mgr.getCameraInfo()));

  info_left->header.stamp = time;
  info_right->header.stamp = time;
  info_left->header.frame_id = frame;
  info_right->header.frame_id = frame;

  left_info_pub.publish(info_left);
  right_info_pub.publish(info_right);
}

void StereoCamera::callback(see3cam::cameraParamsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d", config.exposureauto);
  try{cam_left->set_control(0x009a0901,config.exposureauto);}
  catch(int e){printf("callback failed\n");}
  try{cam_right->set_control(0x009a0901,config.exposureauto);}
  catch(int e){printf("callback failed\n");}
       
  ROS_INFO("Reconfigure Request: %d", config.exposure);
  try{cam_left->set_control(0x009a0902,config.exposure);} 
  catch(int e){printf("callback failed\n");}
  try{cam_right->set_control(0x009a0902,config.exposure);} 
  catch(int e){printf("callback failed\n");}

  ROS_INFO("Reconfigure Request: %d", config.brightness);
  try{cam_left->set_control(0x00980900,config.brightness);}
  catch(int e){printf("callback failed\n");}
  try{cam_right->set_control(0x00980900,config.brightness);}
  catch(int e){printf("callback failed\n");}
       
  ROS_INFO("Reconfigure Request: %d", config.contrast);
  try{cam_left->set_control(0x00980901,config.contrast);} 
  catch(int e){printf("callback failed\n");}
  try{cam_right->set_control(0x00980901,config.contrast);} 
  catch(int e){printf("callback failed\n");}

  ROS_INFO("Reconfigure Request: %d", config.saturation);
  try{cam_left->set_control(0x00980902,config.saturation);}
  catch(int e){printf("callback failed\n");}
  try{cam_right->set_control(0x00980902,config.saturation);}
  catch(int e){printf("callback failed\n");}
       
  ROS_INFO("Reconfigure Request: %d", config.Hue);
  try{cam_left->set_control(0x00980903,config.Hue);} 
  catch(int e){printf("callback failed\n");}
  try{cam_right->set_control(0x00980903,config.Hue);} 
  catch(int e){printf("callback failed\n");}

  ROS_INFO("Reconfigure Request: %d", config.Gamma);
  try{cam_left->set_control(0x00980910,config.Gamma);}
  catch(int e){printf("callback failed\n");}
  try{cam_right->set_control(0x00980910,config.Gamma);}
  catch(int e){printf("callback failed\n");}
       
  ROS_INFO("Reconfigure Request: %d", config.Gain);
  try{cam_left->set_control(0x00980913,config.Gain);} 
  catch(int e){printf("callback failed\n");}
  try{cam_right->set_control(0x00980913,config.Gain);} 
  catch(int e){printf("callback failed\n");}
}

void StereoCamera::feedImages() {
  unsigned int pair_id = 0;
  
  dynamic_reconfigure::Server<see3cam::cameraParamsConfig> server;
  dynamic_reconfigure::Server<see3cam::cameraParamsConfig>::CallbackType f;

  f = boost::bind(&see3cam::StereoCamera::callback, this, _1, _2);
  server.setCallback(f);

  while (ok) {
    unsigned char *frame_left = NULL, *frame_right = NULL;
    uint32_t bytes_used_left, bytes_used_right;

    int channels = 2;
    if (encoding == "rgb8") {
      channels = 3;
    }
    if (encoding == "mono8") {
      channels = 1;
    }

    int left_idx = cam_left->grab(&frame_left, bytes_used_left);
    int right_idx = cam_right->grab(&frame_right, bytes_used_right);

    ros::Time capture_time = ros::Time::now();

    if (frame_left && frame_right) {
	    ImagePtr image_left(new Image);
	    ImagePtr image_right(new Image);

	    image_left->height = height;
	    image_left->width = width;
	    image_left->step = channels * width;
	    image_left->encoding = encoding;
	    image_left->header.stamp = capture_time;
	    image_left->header.seq = pair_id;

	    image_right->height = height;
	    image_right->width = width;
	    image_right->step = channels * width;
	    image_right->encoding = encoding;
	    image_right->header.stamp = capture_time;
	    image_right->header.seq = pair_id;

	    image_left->header.frame_id = frame;
	    image_right->header.frame_id = frame;

	    image_left->data.resize(image_left->step * image_left->height);
	    image_right->data.resize(image_right->step * image_right->height);

	    if (rotate_left)
        switch (channels) {
        case 1:
	        RotateMono(&image_left->data[0], frame_left, width * height);
          break;
        case 3:
	        RotateRgb(&image_left->data[0], frame_left, width * height);
          break;
        default:
	        RotateYuv(&image_left->data[0], frame_left, width * height);
        }
	    else
	      memcpy(&image_left->data[0], frame_left, image_left->step * height);

	    if (rotate_right)
        switch (channels) {
        case 1:
	        RotateMono(&image_right->data[0], frame_right, width * height);
          break;
        case 3:
	        RotateRgb(&image_right->data[0], frame_right, width * height);
          break;
        default:
	        RotateYuv(&image_right->data[0], frame_right, width * height);
        }
	    else
	      memcpy(&image_right->data[0], frame_right, image_right->step * height);

	    left_pub.publish(image_left);
	    right_pub.publish(image_right);

	    sendInfo(capture_time);

	    ++pair_id;
    } else {
    	// Grab failure - need to restart camera driver.
		if (cam_left)
			delete cam_left;
		if (cam_right)
			delete cam_right;
		sleep (1);
		cam_left = new uvc_cam::Cam(left_device.c_str(), mode, width, height, fps);
		cam_right = new uvc_cam::Cam(right_device.c_str(), mode, width, height, fps);
		sleep (1);

		// Force dynamic reconfigure to update
		std::string cmd = "rosrun dynamic_reconfigure dynparam set " + ros::this_node::getName() + " exposureauto 1";
		printf((cmd + "\n").c_str());
		system(cmd.c_str());
    }
    if (frame_left)
      cam_left->release(left_idx);
    if (frame_right)
      cam_right->release(right_idx);
  }
}


StereoCamera::~StereoCamera() {
  ok = false;
  image_thread.join();
  if (cam_left)
    delete cam_left;
  if (cam_right)
    delete cam_right;
}

};
