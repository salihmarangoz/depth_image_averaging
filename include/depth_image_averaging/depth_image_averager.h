#ifndef DEPTH_IMAGE_AVERAGER_H
#define DEPTH_IMAGE_AVERAGER_H

#include <sensor_msgs/Image.h>
#include <vector>
#include <depth_image_proc/depth_traits.h>

namespace depth_image_averaging
{

//////////////////////////////////////////////////////////////////////////

class DepthImageAverager
{
public:
  DepthImageAverager(int width, int height, int min_elements, int max_elements);
  bool add(const sensor_msgs::ImageConstPtr &image);
  void reset();
  int size();
  bool computeMean(sensor_msgs::ImagePtr &averaged_image);
  bool computeMedian(sensor_msgs::ImagePtr &averaged_image, bool true_median);
  bool computeMAD(sensor_msgs::ImagePtr &averaged_image, float mad_upper_limit_a, float mad_upper_limit_b, float mad_scale, bool true_median);
private:
  void vector2DepthImage_(sensor_msgs::ImagePtr &image, const std::vector<float> arr);
  int width_;
  int height_;
  int min_elements_;
  int max_elements_;
  int size_;
  sensor_msgs::ImageConstPtr last_image_;
  std::vector<float> arr_;
};

//////////////////////////////////////////////////////////////////////////

template<typename T>
void vector2DepthData(sensor_msgs::ImagePtr &image, const std::vector<float> &arr)
{
  image->data.resize(image->step*image->height);
  T* depth_row = reinterpret_cast<T*>(&image->data[0]);
  int row_step = image->step / sizeof(T);

  int idx = 0;
  for (int v = 0; v < (int)image->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)image->width; ++u)
    {
      depth_row[u] = depth_image_proc::DepthTraits<T>::fromMeters(arr[idx]);
      idx++;
    }
  }
}

//////////////////////////////////////////////////////////////////////////

template<typename T>
void depthImageStridedCopy(const sensor_msgs::ImageConstPtr &image, std::vector<float> &arr, int stride, int shift)
{
  const T* depth_row = reinterpret_cast<const T*>(&image->data[0]);
  int row_step = image->step / sizeof(T);

  int idx = shift;
  for (int v = 0; v < (int)image->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)image->width; ++u)
    {
      const T depth = depth_row[u];
      if (depth_image_proc::DepthTraits<T>::valid(depth))
      {
        arr[idx] = depth_image_proc::DepthTraits<T>::toMeters(depth);
      }
      else
      {
        arr[idx] = std::numeric_limits<float>::quiet_NaN();
      }
      idx += stride;
    }
  }
}

//////////////////////////////////////////////////////////////////////////

}  // namespace depth_image_averaging

#endif  // DEPTH_IMAGE_AVERAGER_H