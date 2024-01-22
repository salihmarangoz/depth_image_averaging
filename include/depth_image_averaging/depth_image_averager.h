#ifndef DEPTH_IMAGE_AVERAGER_H
#define DEPTH_IMAGE_AVERAGER_H

#include <sensor_msgs/Image.h>

namespace depth_image_averaging
{

class IDepthImageAverager
{
public:
  virtual void add(const sensor_msgs::ImageConstPtr &image) = 0;
  virtual sensor_msgs::ImagePtr compute() = 0;
  virtual void reset() = 0;
};


}  // namespace depth_image_averaging

#endif  // DEPTH_IMAGE_AVERAGER_H