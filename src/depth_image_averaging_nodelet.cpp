#include "depth_image_averaging/depth_image_averaging_nodelet.h"
#include "pluginlib/class_list_macros.h"

namespace depth_image_averaging
{
  void DepthImageAveragingNodelet::onInit()
  {
    NODELET_INFO("it is working!");
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();
  };
}  // namespace depth_image_averaging

PLUGINLIB_EXPORT_CLASS(depth_image_averaging::DepthImageAveragingNodelet, nodelet::Nodelet);