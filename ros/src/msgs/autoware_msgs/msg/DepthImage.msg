# autoware_sensor_msgs/DepthImage
Header header

float32             min_depth     # min. distance from camera in meters
float32             max_depth     # max. distance from camera in meters

sensor_msgs/Image   depth_image   # Normalize (65535, 0) MONO16 encoding