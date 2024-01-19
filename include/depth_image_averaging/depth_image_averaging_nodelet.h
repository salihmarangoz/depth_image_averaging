#ifndef DEPTH_IMAGE_AVERAGING_NODELET_H
#define DEPTH_IMAGE_AVERAGING_NODELET_H

#include "nodelet/nodelet.h"
#include "ros/ros.h"

namespace depth_image_averaging
{

class DepthImageAveragingNodelet : public nodelet::Nodelet
{
public:


private:
  virtual void onInit();
  ros::NodeHandle nh_, private_nh_;
};

}  // namespace depth_image_averaging

#endif  // DEPTH_IMAGE_AVERAGING_NODELET_H