# Information for Extracted Point Clouds from Stereo/Depth Cameras

If you are extracting a `sensor_msgs/PointCloud2` topic coming from a stereo camera you will probably need to apply a filter. The raw data will typically contain points that are extravagant distances away. We have seen point clouds from stereo cameras where points are almost 3km away from the sensor. When visualizing this unfiltered point cloud you can be forgiven for thinking the data is corrupted. 

You may be wondering "how can RVIZ visualize the data without issues, yet the unfiltered extracted data when visualized appears corrupted?". The reason is that RVIZ, due to the ROS ecosystem, knows the camera parameters (e.g., the focal length) and can utilise these to provide better visuals (i.e., you are seeing a filtered version anyway and not the raw data). To ensure the ROSData tools are simple but powerful, data association between topics was not desired and we didn't want to filter data. Hence, you will want to filter the extracted point clouds associated with depth images. 

If you are using Python Open3D, this [Open3D Utils Package](#)(link to come) has some great tools, including tools to do point cloud to camera projection.  