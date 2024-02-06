#ifndef DEPTH_IMAGE_AVERAGING_NODELET_H
#define DEPTH_IMAGE_AVERAGING_NODELET_H

#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include <depth_image_averaging/depth_image_averager.h>

#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <deque>
#include <memory>
#include <vector>

namespace depth_image_averaging
{

class DepthImageAveragingNodelet : public nodelet::Nodelet
{
public:
  DepthImageAveragingNodelet();
private:
  virtual void onInit();
  void depthImageCallback(const sensor_msgs::ImageConstPtr &image);
  void publishAcc();
  bool checkMovement(const geometry_msgs::TransformStamped& transform_a, const geometry_msgs::TransformStamped& transform_b);

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber depth_image_sub_;
  ros::Publisher acc_pub_;
  std::shared_ptr<DepthImageAverager> depth_image_averager_;
  std::deque<sensor_msgs::ImageConstPtr> image_buffer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::TransformStamped last_stable_transform_;
  sensor_msgs::ImageConstPtr last_stable_image_;

  // Parameters
  std::string reference_frame_;
  double window_left_margin_;
  double window_right_margin_;
  int min_elements_;
  int max_elements_;
  double max_displacement_;
  double max_rotation_;
  int averaging_method_;
  bool true_median_;
  double mad_upper_limit_a_;
  double mad_upper_limit_b_;
  double mad_scale_;
};

}  // namespace depth_image_averaging

#endif  // DEPTH_IMAGE_AVERAGING_NODELET_H