#ifndef DEPTH_IMAGE_AVERAGING_NODELET_H
#define DEPTH_IMAGE_AVERAGING_NODELET_H

#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include <depth_image_averaging/depth_image_averager.h>

#include <image_transport/image_transport.h>
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
  void cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);
  void depthImageCallback(const sensor_msgs::ImageConstPtr &image);
  void publishAcc();
  bool checkMovement(const geometry_msgs::TransformStamped& transform_a, const geometry_msgs::TransformStamped& transform_b);

  ros::NodeHandle nh_, private_nh_;
  std::shared_ptr<DepthImageAverager> depth_image_averager_;
  std::deque<sensor_msgs::ImageConstPtr> image_buffer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::TransformStamped last_stable_transform_;

  ros::Subscriber depth_image_sub_;
  ros::Publisher depth_image_pub_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber camera_sub_;
  image_transport::CameraPublisher camera_pub_;
  sensor_msgs::CameraInfoConstPtr last_camera_info_;

  // Parameters
  std::string cl_kernel_path_;
  bool use_image_transport_;
  std::string reference_frame_;
  double window_left_margin_;
  double window_right_margin_;
  int min_elements_;
  int max_elements_;
  bool drop_last_;
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