#include "depth_image_averaging/depth_image_averaging_nodelet.h"
#include "pluginlib/class_list_macros.h"
#include <sensor_msgs/image_encodings.h>
#include <depth_image_proc/depth_traits.h>

namespace depth_image_averaging
{

DepthImageAveragingNodelet::DepthImageAveragingNodelet()
{
  last_stable_image_ = nullptr;
}

void DepthImageAveragingNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Read parameters
  private_nh_.param("reference_frame", reference_frame_, std::string("world"));
  private_nh_.param("window_left_margin_", window_left_margin_, 0.5);
  private_nh_.param("window_right_margin_", window_right_margin_, 0.5);
  private_nh_.param("min_window_size", min_window_size_, 15);
  private_nh_.param("max_window_size", max_window_size_, 30);
  private_nh_.param("max_displacement", max_displacement_, 0.01);
  private_nh_.param("max_rotation", max_rotation_, 0.01);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  acc_pub_ = nh_.advertise<sensor_msgs::Image>("depth_out", 20);
  depth_image_sub_ = nh_.subscribe("depth_in", 150, &DepthImageAveragingNodelet::depthImageCallback, this);
};

void DepthImageAveragingNodelet::depthImageCallback(const sensor_msgs::ImageConstPtr &image)
{
  NODELET_INFO_ONCE("DepthImageAveragingNodelet: First image received!");

  if (depth_image_averager_ == nullptr)
  {
    depth_image_averager_ = std::make_shared<DepthImageAverager>(image->width, image->height, min_window_size_, max_window_size_);
  }

  try
  {
    geometry_msgs::TransformStamped current_transform = tf_buffer_->lookupTransform(reference_frame_, image->header.frame_id, image->header.stamp, ros::Duration(10.0));
    bool is_moved = checkMovement(current_transform, last_stable_transform_);

    // If in front margin, drop the message and update transform if movement detected
    if ((current_transform.header.stamp - last_stable_transform_.header.stamp).toSec() < window_left_margin_)
    {
      if (is_moved)
      {
        // reset
        depth_image_averager_->reset();
        image_buffer_.clear();
        last_stable_transform_ = current_transform;
      }
      return;
    }

    // If movement detected, publish the accumulated data and reset
    if (is_moved)
    {
      if (depth_image_averager_->size() >= min_window_size_)
      {
        publishAcc();
      }

      // reset
      depth_image_averager_->reset();
      image_buffer_.clear();
      last_stable_transform_ = current_transform;
    }

    // Add the image to the buffer, we will decide to process or wait in the buffer later
    image_buffer_.push_back(image);

    // Process the buffer
    while (image_buffer_.size() > 0 && (image->header.stamp - image_buffer_.front()->header.stamp).toSec() > window_right_margin_)
    {
      depth_image_averager_->add(image_buffer_.front());
      image_buffer_.pop_front();
    }

    // Is the batch complete?
    if (depth_image_averager_->size() >= max_window_size_)
    {
      publishAcc();
      // soft-reset
      depth_image_averager_->reset();
    }

  }
  catch (tf2::TransformException &ex)
  {
    NODELET_WARN(ex.what());
  }
}

bool DepthImageAveragingNodelet::checkMovement(const geometry_msgs::TransformStamped& transform_a, const geometry_msgs::TransformStamped& transform_b)
{
  // Check displacement
  double distance_squared =   pow(transform_a.transform.translation.x - transform_b.transform.translation.x, 2) +
                              pow(transform_a.transform.translation.y - transform_b.transform.translation.y, 2) + 
                              pow(transform_a.transform.translation.z - transform_b.transform.translation.z, 2);
  if (distance_squared > pow(max_displacement_, 2))
  {
    return true;
  }

  // Check rotation
  double dot_prod = transform_a.transform.rotation.w * transform_b.transform.rotation.w +
                    transform_a.transform.rotation.x * transform_b.transform.rotation.x +
                    transform_a.transform.rotation.y * transform_b.transform.rotation.y +
                    transform_a.transform.rotation.z * transform_b.transform.rotation.z;
  double rotation_distance = std::acos(2*dot_prod*dot_prod-1);

  if (rotation_distance > max_rotation_)
  {
    return true;
  }

  return false;
}

void DepthImageAveragingNodelet::publishAcc()
{
  if (depth_image_averager_->size() <= 0) return;

  sensor_msgs::ImagePtr acc_image = boost::make_shared<sensor_msgs::Image>();
  //depth_image_averager_->computeMean(acc_image);
  //depth_image_averager_->computeMedian(acc_image);
  depth_image_averager_->computeMAD(acc_image); // todo: parameters
  acc_pub_.publish(acc_image);
}

}  // namespace depth_image_averaging

PLUGINLIB_EXPORT_CLASS(depth_image_averaging::DepthImageAveragingNodelet, nodelet::Nodelet);