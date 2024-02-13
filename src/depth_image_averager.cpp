/* 
 * This file is part of the depth_image_averaging (https://github.com/salihmarangoz/depth_image_averaging).
 * Copyright (c) 2024 Salih Marangoz
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "depth_image_averaging/depth_image_averager.h"
#include <sensor_msgs/image_encodings.h>
#include <depth_image_proc/depth_traits.h>
#include <boost/make_shared.hpp>
#include <iostream>
#include <fstream>
#include <string>

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

#if USE_OPENCL
  is_opencl_initialized_ = false;
  opencl_kernel_ = NULL;
#endif
}

//////////////////////////////////////////////////////////////////////////

void DepthImageAverager::vector2DepthImage_(sensor_msgs::ImagePtr &image, const std::vector<float> arr)
{
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

int DepthImageAverager::computeMean(sensor_msgs::ImagePtr &averaged_image)
{
  if (size_ <= 0) return 1;

  std::vector<float> averaged_arr;
  averaged_arr.resize(width_*height_);

  for (int i=0; i < width_*height_; i++)
  {
    // accumulate finite values
    int filtered_size = 0;
    float filtered_sum = 0.0;
    int inf_count = 0;
    for (int j=0; j<size_; j++)
    {
      float val = arr_[i*max_elements_+j];
      if (std::isfinite(val))
      {
        filtered_sum += val;
        filtered_size++;
      }
      else if (std::isinf(val))
      {
        inf_count++;
      }
    }
    int nan_count = size_ - filtered_size - inf_count;

    // Min-element check
    if (filtered_size < min_elements_)
    {
      if (nan_count > inf_count)
      {
        averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      }
      else
      {
        averaged_arr[i] = std::numeric_limits<float>::infinity();
      }
      continue;
    }

    averaged_arr[i] = filtered_sum / filtered_size;
  }

  vector2DepthImage_(averaged_image, averaged_arr);
  return 0;
}

//////////////////////////////////////////////////////////////////////////

int DepthImageAverager::computeMedian(sensor_msgs::ImagePtr &averaged_image, bool true_median)
{
  if (size_ <= 0) return 1;

  std::vector<float> tmp_buffer;
  tmp_buffer.resize(size_);

  std::vector<float> averaged_arr;
  averaged_arr.resize(width_*height_);

  int inf_count = 0;
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
      else if (std::isinf(val))
      {
        inf_count++;
      }
    }
    int nan_count = size_ - filtered_size - inf_count;

    // Min-element check
    if (filtered_size < min_elements_)
    {
      if (nan_count > inf_count)
      {
        averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      }
      else
      {
        averaged_arr[i] = std::numeric_limits<float>::infinity();
      }
      continue;
    }

    auto m = tmp_buffer.begin() + filtered_size / 2;
    std::nth_element(tmp_buffer.begin(), m, tmp_buffer.begin() + filtered_size);

    if (true_median && filtered_size % 2 == 0)
    {
      // true median
      std::nth_element(tmp_buffer.begin(), m-1, tmp_buffer.begin() + filtered_size);
      float val_a = tmp_buffer[filtered_size / 2 - 1];
      float val_b = tmp_buffer[filtered_size / 2];
      averaged_arr[i] = (val_a + val_b) / 2.0;
    }
    else
    {
      // fast median, but biased if the array length is even
      averaged_arr[i] = tmp_buffer[filtered_size / 2];
    }
  }

  vector2DepthImage_(averaged_image, averaged_arr);
  return 0;
}

//////////////////////////////////////////////////////////////////////////

int DepthImageAverager::computeMAD(sensor_msgs::ImagePtr &averaged_image, float mad_upper_limit_a, float mad_upper_limit_b, float mad_scale, bool true_median)
{
  if (size_ <= 0) return 1;

  std::vector<float> tmp_buffer;
  tmp_buffer.resize(size_);

  std::vector<float> averaged_arr;
  averaged_arr.resize(width_*height_);

  int inf_count = 0;
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
      else if (std::isinf(val))
      {
        inf_count++;
      }
    }
    int nan_count = size_ - filtered_size - inf_count;

    // Min-element check
    if (filtered_size < min_elements_)
    {
      if (nan_count > inf_count)
      {
        averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      }
      else
      {
        averaged_arr[i] = std::numeric_limits<float>::infinity();
      }
      continue;
    }

    // find median
    auto m = tmp_buffer.begin() + filtered_size / 2;
    std::nth_element(tmp_buffer.begin(), m, tmp_buffer.begin() + filtered_size);
    float median_value;
    if (true_median && filtered_size % 2 == 0)
    {
      // true median
      std::nth_element(tmp_buffer.begin(), m-1, tmp_buffer.begin() + filtered_size);
      float val_a = tmp_buffer[filtered_size / 2 - 1];
      float val_b = tmp_buffer[filtered_size / 2];
      median_value = (val_a + val_b) / 2.0;
    }
    else
    {
      // fast median, but biased if the array length is even
      median_value = tmp_buffer[filtered_size / 2];
    }

    // compute mad
    float mad = 0;
    for (int j=0; j<filtered_size; j++)
    {
      mad += std::fabs(tmp_buffer[j]-median_value);
    }
    mad /= filtered_size;

    // MAD scale check
    if (mad > (mad_upper_limit_a*median_value + mad_upper_limit_b) )
    {
      averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    // filter outliers and compute mean
    float mad_mean = 0;
    int mad_size = 0;
    for (int j=0; j<filtered_size; j++)
    {
      float val = tmp_buffer[j];
      if (std::isfinite(val) && std::fabs(val-median_value) < mad_scale*mad )
      {
        mad_mean += val;
        mad_size++;
      }
    }
    mad_mean /= mad_size;

    // Min-element check
    if (mad_size < min_elements_)
    {
      averaged_arr[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    averaged_arr[i] = mad_mean; // TODO: maybe a weighted avg between the mad_mean and the median value? needs more investigation.
  }

  vector2DepthImage_(averaged_image, averaged_arr);
  return 0;
}

//////////////////////////////////////////////////////////////////////////
#if USE_OPENCL

cl_int DepthImageAverager::initOpenCL_()
{
  cl_platform_id platform_id = NULL;
  device_id_ = NULL;
  cl_uint num_devices, num_platforms;
  cl_int ret = clGetPlatformIDs(1, &platform_id, &num_platforms);
  if (ret != 0) return ret;
  ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_GPU, 1, &device_id_, &num_devices);
  if (ret != 0) return ret;

  context_ = clCreateContext(NULL, 1, &device_id_, NULL, NULL, &ret);
  if (ret != 0) return ret;
  command_queue_ = clCreateCommandQueue(context_, device_id_, 0, &ret);
  if (ret != 0) return ret;

  input_buffer_ = clCreateBuffer(context_, CL_MEM_READ_ONLY, sizeof(float) * width_*height_*max_elements_, NULL, &ret);
  if (ret != 0) return ret;
  output_buffer_ = clCreateBuffer(context_, CL_MEM_WRITE_ONLY, sizeof(float) * width_*height_, NULL, &ret);
  if (ret != 0) return ret;

  is_opencl_initialized_ = true;
  return ret;
}

//////////////////////////////////////////////////////////////////////////

cl_kernel DepthImageAverager::createKernelOpenCL_(const std::string &kernel_file, cl_int &ret)
{
  // read the source file
  std::ifstream stream(kernel_file.c_str());
  if (!stream.is_open()) {
    std::cout << "Cannot open file: " << kernel_file << std::endl;
    exit(1);
  }
  std::string kernel_source(std::istreambuf_iterator<char>(stream), (std::istreambuf_iterator<char>()));

  // Set constant values
  std::string token_max_elements("{max_elements}");
  auto pos_max_elements = kernel_source.find(token_max_elements);
  if (pos_max_elements != std::string::npos) kernel_source.replace(pos_max_elements, std::string(token_max_elements).size(), std::to_string(max_elements_));

  std::string token_min_elements("{min_elements}");
  auto pos_min_elements = kernel_source.find(token_min_elements);
  if (pos_min_elements != std::string::npos) kernel_source.replace(pos_min_elements, std::string(token_min_elements).size(), std::to_string(min_elements_));

  size_t kernel_source_size = kernel_source.length();
  char *tmp = kernel_source.data();
  cl_program program = clCreateProgramWithSource(context_, 1, (const char**)&tmp, (const size_t *)&kernel_source_size, &ret);
  if (ret != 0) return NULL;
  ret = clBuildProgram(program, 1, &device_id_, NULL, NULL, NULL);
  if (ret != 0) return NULL;
  cl_kernel kernel = clCreateKernel(program, "compute_average", &ret);
  if (ret != 0) return NULL;

  ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&input_buffer_);
  if (ret != 0) return NULL;
  ret = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void *)&output_buffer_);
  if (ret != 0) return NULL;

  return kernel;
}

//////////////////////////////////////////////////////////////////////////

int DepthImageAverager::computeOpenCL(sensor_msgs::ImagePtr &averaged_image, const std::string& kernel_file)
{
  cl_int ret = 0;

  if (size_ <= 0) return 1;

  if (!is_opencl_initialized_) ret = initOpenCL_();
  if (ret != 0) return ret;

  if (opencl_kernel_ == NULL) opencl_kernel_ = createKernelOpenCL_(kernel_file, ret);
  if (ret != 0) return ret;

  ret = clSetKernelArg(opencl_kernel_, 2, sizeof(cl_int), &size_);
  if (ret != 0) return NULL;

  // set the input
  ret = clEnqueueWriteBuffer(command_queue_, input_buffer_, CL_TRUE, 0, sizeof(float) * width_*height_*max_elements_, arr_.data(), 0, NULL, NULL);
  if (ret != 0) return ret;

  // run the kernel
  size_t global_item_size = width_*height_;
  ret = clEnqueueNDRangeKernel(command_queue_, opencl_kernel_, 1, NULL, &global_item_size, NULL, 0, NULL, NULL);
  if (ret != 0) return ret;

  // get the output
  std::vector<float> averaged_arr;
  averaged_arr.resize(width_*height_);
  ret = clEnqueueReadBuffer(command_queue_, output_buffer_, CL_TRUE, 0, sizeof(float) * width_*height_, averaged_arr.data(), 0, NULL, NULL);
  if (ret != 0) return ret;

  vector2DepthImage_(averaged_image, averaged_arr);
  return 0;
}

//////////////////////////////////////////////////////////////////////////
#endif // USE_OPENCL

} // depth_image_averaging