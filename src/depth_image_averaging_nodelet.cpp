#include "depth_image_averaging/depth_image_averaging_nodelet.h"
#include "pluginlib/class_list_macros.h"
#include <sensor_msgs/image_encodings.h>
#include <depth_image_proc/depth_traits.h>

namespace depth_image_averaging
{

DepthImageAveragingNodelet::DepthImageAveragingNodelet()
{
  num_accumulated_images_ = 0;
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
  private_nh_.param("min_window_size", min_window_size_, 3);
  private_nh_.param("max_window_size", max_window_size_, 20);
  private_nh_.param("max_displacement", max_displacement_, 0.01);
  private_nh_.param("max_rotation", max_rotation_, 0.01);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  acc_pub_ = nh_.advertise<sensor_msgs::Image>("depth_out", 20);
  depth_image_sub_ = nh_.subscribe("depth_in", 150, &DepthImageAveragingNodelet::depthImageCallback, this);
};

void DepthImageAveragingNodelet::depthImageCallback(const sensor_msgs::ImageConstPtr &image)
{
  NODELET_INFO_ONCE("DepthImageAveragingNodelet: First image received!");

  try
  {
    geometry_msgs::TransformStamped current_transform = tf_buffer_->lookupTransform(reference_frame_, image->header.frame_id, image->header.stamp, ros::Duration(10.0));
    bool is_moved = checkMovement(current_transform, last_stable_transform_);
    NODELET_INFO("num_accumulated_images_: %d", num_accumulated_images_);

    // If in front margin, drop the message and update transform if movement detected
    if ((current_transform.header.stamp - last_stable_transform_.header.stamp).toSec() < window_left_margin_)
    {
      if (is_moved)
      {
        // reset
        resetAcc();
        image_buffer_.clear();
        last_stable_transform_ = current_transform;
      }
      return;
    }

    // If movement detected, publish the accumulated data and reset
    if (is_moved)
    {
      if (num_accumulated_images_ >= min_window_size_)
      {
        if (image_buffer_.front()->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        {
          publishAcc<float>();
        }
        else if (image_buffer_.front()->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
          publishAcc<uint16_t>();
        }
      }

      // reset
      resetAcc();
      image_buffer_.clear();
      last_stable_transform_ = current_transform;
    }

    // Add the image to the buffer, we will decide to process or wait in the buffer later
    image_buffer_.push_back(image);

    // Process the buffer
    while (image_buffer_.size() > 0 && (image->header.stamp - image_buffer_.front()->header.stamp).toSec() > window_right_margin_)
    {
      if (image_buffer_.front()->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        updateAcc<float>(image_buffer_.front());
      }
      else if (image_buffer_.front()->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        updateAcc<uint16_t>(image_buffer_.front());
      }
      image_buffer_.pop_front();
    }

    // Is the batch complete?
    if (num_accumulated_images_ >= max_window_size_)
    {
      if (image_buffer_.front()->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        publishAcc<float>();
      }
      else if (image_buffer_.front()->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        publishAcc<uint16_t>();
      }
      // soft-reset
      resetAcc();
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

void DepthImageAveragingNodelet::resetAcc()
{
  num_accumulated_images_ = 0;
}

template<typename T>
void DepthImageAveragingNodelet::updateAcc(const sensor_msgs::ImageConstPtr& image)
{
  if (acc_.size() <= 0) 
  {
    acc_.resize(image->height * image->step);
    return;
  }
  
  if (num_accumulated_images_ <= 0)
  {
    std::fill(acc_.begin(), acc_.end(), 0.0);
  }

  const T* depth_row = reinterpret_cast<const T*>(&image->data[0]);
  int row_step = image->step / sizeof(T);

  for (int v = 0; v < (int)image->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)image->width; ++u)
    {
      T depth = depth_row[u];
      if (depth_image_proc::DepthTraits<T>::valid(depth) && std::isfinite(acc_[v*image->width+u]))
      {
        acc_[v*image->width+u] = (acc_[v*image->width+u]*num_accumulated_images_ + depth_image_proc::DepthTraits<T>::toMeters(depth)) / (num_accumulated_images_+1);
      }
      else
      {
        acc_[v*image->width+u] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  acc_header_ = image->header;
  num_accumulated_images_++;
}

template<typename T>
void DepthImageAveragingNodelet::publishAcc()
{
  if (num_accumulated_images_ <= 0) return;

  // Generate an sensor_msgs::Image
  sensor_msgs::ImagePtr acc_image = boost::make_shared<sensor_msgs::Image>();
  const sensor_msgs::ImageConstPtr dummy_image = image_buffer_.front();
  acc_image->data.resize(dummy_image->height * dummy_image->step, 0);
  depth_image_proc::DepthTraits<T>::initializeBuffer(acc_image->data);
  acc_image->encoding = dummy_image->encoding;
  acc_image->header = acc_header_;
  acc_image->height = dummy_image->height;
  acc_image->is_bigendian = dummy_image->is_bigendian;
  acc_image->step = dummy_image->step;
  acc_image->width = dummy_image->width;

  T* depth_row = reinterpret_cast<T*>(&acc_image->data[0]);
  int row_step = acc_image->step / sizeof(T);
  for (int v = 0; v < (int)acc_image->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)acc_image->width; ++u)
    {
      auto acc_value = acc_[v*acc_image->width+u];
      if (std::isfinite(acc_value))
      {
        depth_row[u] = depth_image_proc::DepthTraits<T>::fromMeters(acc_value); // /num_accumulated_images_
      }
    }
  }
  acc_pub_.publish(acc_image);
}

}  // namespace depth_image_averaging

PLUGINLIB_EXPORT_CLASS(depth_image_averaging::DepthImageAveragingNodelet, nodelet::Nodelet);