# depth_image_averaging

## Introduction

Depth image averaging can be regarded as the most simple way of combining depth images compared to more sophisticated methods such as TSDFs. Compared to these methods, this solution needs the sensor to stay still for a certain amount of time to generate accurate depth measurements of a static scene. If that is the case for your platform, it may be better to average first and then feed the averaged images to the mapping (not SLAM!) solution. In this sense, this package solves a few problems:

- Timing problems between the sensor (camera) and the moving platform that is affecting the quality of the map.
- Very-slow registration of mapping methods (e.g. TSDF) especially in very high resolutions. This becomes a more important problem on slower hardwares.

Contributions are welcome.

## Quick Start

Check the `launch/start.launch` file. You would need a RGBD/Depth sensor or a bag file including RGBD/Depth data since they are not provided with this package.

## How it works

### Batching and Dropping

The aim of this package is to average depth images while the sensor stays still. As seen below, the averager drops a part of accumulated images after the sensor stops and the sensor starts moving, and computes images in batches if enough data is accumulated. 

![how_it_works](assets/how_it_works.svg)

Figure: Red depth images are dropped. Blue depth images are valid for processing. Green depth images are the pixel-wise averaged images that are published.

## MAD Averager

Averaging of depth values can be done via computing mean or median. However there are certain problems. Mean is susceptible to outliers, and median is biased towards the side with the most outliers. Alternatively, outliers can be filtered out with the MAD (Mean absolute deviation) approach and the trimmed mean can be calculated as shown below.

![mad_filtering.drawio](assets/mad_filtering.drawio.svg)

Figure: In the image, red values are dropped, blue values are regarded as valid and the trimmed mean is computed using these values. Red line shows the trimmed mean value.

MAD filtering/averaging steps are as follows:

- If the number of finite values are below a certain threshold then return NaN or inf+ based on which is the most frequent.
- Find the median.
- Compute MAD of all values w.r.t. the median value.
- If the MAD value exceeds a certain threshold, return NaN.
- Filter values outside the scaled MAD region.
- If the number of remaining values are less than a certain threshold, return NaN.
- Compute the average of the remaining values (aka trimmed mean).

## Requirements

This package is tested on Ubuntu 20.04 and ROS Noetic. Other requirements are core packages of ROS which are listed in the `package.xml` file. 

## ROS

**Note:** If the latency and processing power is important, this package also provides a nodelet, allowing zero-copy transport.

### Subscribed Topics

`camera/depth/image_raw` ([sensor_msgs/Image](sensor_msgs/Image))
		Source depth image.

`camera/depth/camera_info` ([sensor_msgs/CameraInfo](sensor_msgs/CameraInfo))
		Source depth camera information. Not used internally, but required if `use_image_transport` is true. If the camera parameters are not fixed not only this topic but also this whole package is useless for your use case. 

### Published Topics

`camera/averaged_depth/image_raw` ([sensor_msgs/Image](sensor_msgs/Image))
		Averaged depth image.

`camera/averaged_depth/camera_info` ([sensor_msgs/CameraInfo](sensor_msgs/CameraInfo))
		Relay of the last camera information with the same header as the averaged depth image. Only published if `use_image_transport` is true. If the camera parameters are not fixed not only this topic but also this whole package is useless for your use case. 

### Parameters

`use_image_transport` (bool, default: true)
		If true, uses `image_transport::CameraPublisher` and `image::transport::CameraSubscriber` which synchronizes and publishes image and camera info. Otherwise, subscribes the image topic and publishes the image directly which may create some problems for visualization in RViz and other applications.

`reference_frame` (string, default: "world")
		Reference transformation frame for detecting sensor movements.

`window_left_margin` (double, default: 0.5)
		In seconds. Drops the images for the given time interval after the movement is stopped.

`window_right_margin` (double, default: 0.5)
		In seconds. Drops the images for the given time interval before the movement is started.

`min_elements` (int, default: 8)
		Minimum number of measurements required for valid averaging.

`max_elements` (int, default: 32)
		Maximum number of measurements. This value also specifies the maximum batch size and affects the memory use linearly.

`drop_last` (bool, default: true)
		If true, drops the whole batch if the batch is not complete after detecting sensor movement.

`max_displacement` (double, default: 0.01)
		In meters. Maximum displacement of the sensor w.r.t. reference frame. Resets the batch if the threshold is exceeded.

`max_rotation` (double, default: 0.01)
		In radians. Maximum rotation of the sensor w.r.t. reference frame. Uses the formula for geodesic distance between the two quaternions. Resets the batch if the threshold is exceeded.

`averaging_method` (int, default: 2)
		Selects the averaging method. `MEAN=0` computes the mean value, `MEDIAN=1` computes the median value, `MAD=2` filters outliers using MAD and computes the mean/median value using the remaining data.

`true_median` (bool, default: true)
		If true, uses the accurate median computation method for even number of elements which requires two iterations. Otherwise, computes median in a single iteration but the median may be biased towards the smallest value. ([Wikipedia](https://simple.wikipedia.org/wiki/Median))

`mad_upper_limit_a` (double, default: 0.008)
		Drops the averaged value for a pixel if the MAD value exceeds the given threshold. Threshold is computed via $a*median+b$. Used if the averaging method is `MAD`.

`mad_upper_limit_b` (double, default: 0.0015)
		Drops the averaged value for a pixel if the MAD value exceeds the given threshold. Threshold is computed via $a*median+b$. Used if the averaging method is `MAD`.

`mad_scale` (double, default: 1.5)
		Drops outliers having deviation higher than the scaled MAD. Used if the averaging method is `MAD`.

### Required tf Transforms

`reference_frame` <==> Sensor Frame

## Disclaimer

This package is developed as a part of created work as a Student Research Assistant at the University of Bonn Humanoid Robots Lab.