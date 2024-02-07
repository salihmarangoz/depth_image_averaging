# depth_image_averaging







## ROS

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
		In seconds. TODO

`window_right_margin` (double, default: 0.5)
		In seconds. TODO

`min_elements` (int, default: 8)
		Minimum number of measurements required for valid averaging.

`max_elements` (int, default: 32)
		Maximum number of measurements TODO

`drop_last` (bool, default: true)
		If true, drops the whole batch if the batch is not complete after detecting sensor movement.

`max_displacement` (double, default: 0.01)
		In meters. Maximum displacement of the sensor w.r.t. reference frame. Resets the batch if the threshold is exceeded.

`max_rotation` (double, default: 0.01)
		In radians. Maximum rotation of the sensor w.r.t. reference frame. Uses the formula for geodesic distance between the two quaternions. Resets the batch if the threshold is exceeded.

`averaging_method` (int, default: 2)
		Selects the averaging method. `MEAN=0` computes the mean value, `MEDIAN=1` computes the median value, `MAD=2` filters outliers using MAD and computes the mean/median value using the remaining data.

`true_median` (bool, default: true)
		If true, uses the accurate median computation method for even number of measurements which requires two quick select iterations. Otherwise, uses one quick select iterations but the median may be biased towards the zero. ([Wikipedia](https://simple.wikipedia.org/wiki/Median))

`mad_upper_limit_a` (double, default: 0.008)
		Used if the averaging method is `MAD`. TODO

`mad_upper_limit_b` (double, default: 0.0015)
		Used if the averaging method is `MAD`. TODO

`mad_scale` (double, default: 1.5)
		Used if the averaging method is `MAD`. TODO

### Required tf Transforms

`reference_frame` <==> Sensor Frame