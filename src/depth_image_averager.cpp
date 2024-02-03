#include "depth_image_averaging/depth_image_averager.h"
#include <sensor_msgs/image_encodings.h>
#include <depth_image_proc/depth_traits.h>
#include <boost/make_shared.hpp>

// TODO: count inf and nan separately

namespace depth_image_averaging
{

//////////////////////////////////////////////////////////////////////////

DepthImageAverager::DepthImageAverager(int width, int height, int min_elements, int max_elements)
{
  width_ = width;
  height_ = height;
  min_elements_ = min_elements;
  max_elements_ = max_elements;

  size_ = 0;
  arr_.resize(width*height*max_elements);
}

//////////////////////////////////////////////////////////////////////////

void DepthImageAverager::vector2DepthImage_(sensor_msgs::ImagePtr &image, const std::vector<float> arr)
{
  //image = boost::make_shared<sensor_msgs::Image>();
  image->encoding = last_image_->encoding;
  image->header = last_image_->header;
  image->height = last_image_->height;
  image->width = last_image_->width;
  image->is_bigendian = last_image_->is_bigendian;
  image->step = last_image_->step;

  if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    vector2DepthData<float>(image, arr);
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    vector2DepthData<uint16_t>(image, arr);
  }
  /*
  else
  {
    return nullptr;
  }

  return image;
  */
}

//////////////////////////////////////////////////////////////////////////

bool DepthImageAverager::add(const sensor_msgs::ImageConstPtr &image)
{
  if (size_ >= max_elements_)
  {
    return false;
  }

  if ( (image->width != width_) || (image->height != height_) )
  {
    return false;
  }

  if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    depthImageStridedCopy<float>(image, arr_, max_elements_, size_);
  }
  else if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    depthImageStridedCopy<uint16_t>(image, arr_, max_elements_, size_);
  }
  else
  {
    return false;
  }
  
  last_image_ = image;
  size_++;
  return true;
}

//////////////////////////////////////////////////////////////////////////

void DepthImageAverager::reset()
{
  size_ = 0;
}

//////////////////////////////////////////////////////////////////////////

int DepthImageAverager::size()
{
  return size_;
}

//////////////////////////////////////////////////////////////////////////

bool DepthImageAverager::computeMean(sensor_msgs::ImagePtr &averaged_image)
{
  if (size_ <= 0) return false;

  std::vector<float> averaged_arr;
  averaged_arr.resize(width_*height_);

  for (int i=0; i < width_*height_; i++)
  {
    // accumulate finite values
    int filtered_size = 0;
    float filtered_sum = 0.0;
    for (int j=0; j<size_; j++)
    {
      float val = arr_[i*max_elements_+j];
      if (std::isfinite(val))
      {
        filtered_sum += val;
        filtered_size++;
      }
    }

    if (filtered_size < min_elements_)
    {
      averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    averaged_arr[i] = filtered_sum / filtered_size;
  }

  vector2DepthImage_(averaged_image, averaged_arr);
  return true;
}

//////////////////////////////////////////////////////////////////////////

bool DepthImageAverager::computeMedian(sensor_msgs::ImagePtr &averaged_image)
{
  if (size_ <= 0) return false;

  std::vector<float> tmp_buffer;
  tmp_buffer.resize(size_);

  std::vector<float> averaged_arr;
  averaged_arr.resize(width_*height_);

  for (int i=0; i < width_*height_; i++)
  {
    // filter non-finite values
    int filtered_size = 0;
    for (int j=0; j<size_; j++)
    {
      float val = arr_[i*max_elements_+j];
      if (std::isfinite(val))
      {
        tmp_buffer[filtered_size] = val;
        filtered_size++;
      }
    }

    if (filtered_size < min_elements_)
    {
      averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    auto m = tmp_buffer.begin() + filtered_size / 2;
    std::nth_element(tmp_buffer.begin(), m, tmp_buffer.begin() + filtered_size);
    averaged_arr[i] = tmp_buffer[filtered_size / 2];
/*  // true median, but slower
    if (filtered_size % 2 == 1)
    {
        averaged_arr[i] = tmp_buffer[filtered_size / 2];
    }
    else
    {
        std::nth_element(tmp_buffer.begin(), m-1, tmp_buffer.begin() + filtered_size);
        float val_a = tmp_buffer[filtered_size / 2 - 1];
        float val_b = tmp_buffer[filtered_size / 2];
        averaged_arr[i] = (val_a + val_b) / 2.0;
    }
*/
  }

  vector2DepthImage_(averaged_image, averaged_arr);
  return true;
}

//////////////////////////////////////////////////////////////////////////

bool DepthImageAverager::computeMAD(sensor_msgs::ImagePtr &averaged_image)
{
  if (size_ <= 0) return false;

  std::vector<float> tmp_buffer;
  tmp_buffer.resize(size_);

  std::vector<float> averaged_arr;
  averaged_arr.resize(width_*height_);

  for (int i=0; i < width_*height_; i++)
  {
    // filter non-finite values
    int filtered_size = 0;
    for (int j=0; j<size_; j++)
    {
      float val = arr_[i*max_elements_+j];
      if (std::isfinite(val))
      {
        tmp_buffer[filtered_size] = val;
        filtered_size++;
      }
    }

    if (filtered_size < min_elements_)
    {
      averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    // find median
    auto m = tmp_buffer.begin() + filtered_size / 2;
    std::nth_element(tmp_buffer.begin(), m, tmp_buffer.begin() + filtered_size);
    float median_value = tmp_buffer[filtered_size / 2];

    if (!std::isfinite(median_value))
    {
      averaged_arr[i] = median_value;
      continue;
    }

    // compute mad
    float mad = 0;
    for (int j=0; j<size_; j++)
    {
      float val = arr_[i*max_elements_+j];
      if (std::isfinite(val))
      {
        mad += std::fabs(val-median_value);
      }
    }
    mad /= filtered_size;

    // TODO
    if (mad > 0.005)
    {
      averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    // filter outliers and compute mean
    float mad_sum = 0;
    int mad_size = 0;
    for (int j=0; j<size_; j++)
    {
      float val = arr_[i*max_elements_+j];
      if (std::isfinite(val) && std::fabs(val-median_value) < 1.5*mad ) // todo
      {
        mad_sum += val;
        mad_size++;
      }
    }
    mad_sum /= mad_size;

    if (mad_size < min_elements_)
    {
      averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    averaged_arr[i] = mad_sum;
  }

  vector2DepthImage_(averaged_image, averaged_arr);
  return true;
}

//////////////////////////////////////////////////////////////////////////

} // depth_image_averaging