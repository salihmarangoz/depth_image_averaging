#ifndef DEPTH_IMAGE_AVERAGING_NODELET_H
#define DEPTH_IMAGE_AVERAGING_NODELET_H

#include "nodelet/nodelet.h"
#include "ros/ros.h"

#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <mutex>
#include <thread>
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
  void resetAcc();
  template<typename T> void updateAcc(const sensor_msgs::ImageConstPtr& image);
  template<typename T> void publishAcc();
  bool checkMovement(const geometry_msgs::TransformStamped& transform_a, const geometry_msgs::TransformStamped& transform_b);

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber depth_image_sub_;
  ros::Publisher acc_pub_;
  std::deque<sensor_msgs::ImageConstPtr> image_buffer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::TransformStamped last_stable_transform_;
  sensor_msgs::ImageConstPtr last_stable_image_;
  int num_accumulated_images_;
  std::vector<float> acc_;
  std_msgs::Header acc_header_;

  // Parameters
  std::string reference_frame_;
  double window_left_margin_;
  double window_right_margin_;
  int min_window_size_;
  int max_window_size_;
  double max_displacement_;
  double max_rotation_;
};

}  // namespace depth_image_averaging

#endif  // DEPTH_IMAGE_AVERAGING_NODELET_H