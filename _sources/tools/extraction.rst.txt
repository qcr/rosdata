Extraction Tool
=================

The Extraction Tool is the core of ROSData. It truly was the main reason why we developed the ROSData tools. To make the extraction of data from ROSBags as easy as possible and not require the creation of a script for every different dataset collected. 

The extraction tool requires you to write simple extraction config file. This config file is a YAML file defining the data you want extracted and to which folder you wanted it extracted. This means once you have created an extraction config file for a specific robot, platform, or ROS system, you can easily reuse that one config file and share it with your colleagues. We find this works really well with students and researchers who aren't familiar with ROS and need to get a project moving quickly. We can quickly teach them how to turn on the machine and how to record a ROSBag, and then simply provide them with the YAML file for extraction. 

Extraction Config File
-----------------------

The extraction config file is a YAML file. This file is used to specify what data should be extracted from a ROSBag and where it should be extracted. Currently only three types of high level keys are supported by the tool. These keys are:

    - :code:`transorm_<id>`; 
    - :code:`camera_info_<id>`; and
    - :code:`topic_<id>`.

The :code:`<id>` is simply a unique number defined by you. Each key is used to extract data in a specific format for a specific set of ROS message types. The message types associated with each key are:

+--------------------------+-------------------------------------------------------------------------+
| **Extraction Config Key**| **Associated ROS Message Types**                                        |
+==========================+=========================================================================+
| :code:`transform_<id>`   | tf2_msgs/TFMessage                                                      |
+--------------------------+-------------------------------------------------------------------------+
| :code:`camera_info_<id>` | sensor_msgs/CameraInfo                                                  |
+--------------------------+-------------------------------------------------------------------------+
| :code:`topic_<id>`       | - sensor_msgs/CompressedImage                                           |
|                          | - sensor_msgs/Image                                                     |
|                          | - sensor_msgs/PointCloud2                                               |
+--------------------------+-------------------------------------------------------------------------+

You can extract as many of each message type from your ROSBag by simply utilising unique IDs. For example, if you wish to extract a CSV file for the transform :code:`map` to :code:`odom`, as well as :code:`map` to :code:`base_link` you could simply have two transform keys (e.g., :code:`transform_0` and :code:`transform_1`), one for each. If your ROS system has a disconnected transform tree resulting in multiple tree roots, can specify the tree root in the config file using the key :code:`tree_root`.

Arguments
^^^^^^^^^^

Transforms
""""""""""""""""""""""""

.. code-block:: yaml

    transform_<id>:
        parent_frame: <parent_frame> # required - used to specify the parent/origin frame
        child_frame: <child_frame> # required - used to specify a child/destination frame
        filename: <filename> # optional - used to specify a filename, the file extension .csv will be appended. Defaults to transform_<id>.csv
        output_destination: <relative_destination> # optional - used to specify a directory relative to the root output directory to save the CSV file. Defaults to the root output directory.


Camera Info
""""""""""""""""""""""""""

.. code-block:: yaml

    camera_info_<id>:
        topic_name: <camera_info_topic> # required - used to specify the name ofthe camera info topic
        filename: <filename> # optional - used to specify a filename, the file extension .yaml will be appended. Defaults to camera_info_<id>.csv
        output_destination: <relative_destination> # optional - used to specify a directory relative to the root output directory to save the YAML file. Defaults to the root output directory.


Topics
""""""""""""""""""""""""

.. code-block:: yaml

    topic_<id>:
        topic_name: <ros_topic_name> # required - used to specify the name of the topic
        message_type: <ros_message_type> # required - determines the data extraction method
        output_destination: <relative_destination> # optional - used to specify a directory relative to the root output directory to save the topic data. Defaults to the root_output_directory/topic_<id>.
        filename_template: <filename_template> # optional - used as a filename template string (e.g. `image_%06d-<ros_timestamp>`), the appropriate file_extenstion will be automatically appended. Only a single topic index and ROS timestamp can be included in the template. Use the Python `%d` string formatter, or derivate of, to specify the topic index and use `<ros_timestamp>` to include the ROS topic timestamp as a string which will be in the format `<seconds>_<nanaseconds>`. Defaults to `frame_%06d`
        transforms: # optional - only required if wish to output a CSV(s) containing transform data associated with the topic. Can specify multiple transforms to generate multiple CSV files all with different parameters
            - parent_frame: <parent_frame_1> # required - used to specify the parent/origin frame
            child_frame: <child_frame> # optional - used to specify the child/destination frame. Defaults to the frame ID stored in the topic
            selection_method: <method_1> # optional - used to specify the method to determine the transform associated with each message within the topic. Options are exact, recent, nearest, and interpolate. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
            lookup_limit: <seconds_1> # optional - used to specify a lookup limit when determining the transform. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
            chain_limit: <seconds_1> # optional - used to specify a transform chain differential limit when determining the transform. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
            filename: <filename_1> # optional - used to specify a filename, the file extension .csv will be appended. Defaults to topic_<id>.csv
            output_destination: <relative_destination_1> # optional - used to specify a directory relative to the root output directory to save the CSV file. Defaults to the root output directory.
            - parent_frame: <parent_frame_2> # required - used to specify the parent/origin frame
            child_frame: <child_frame> # optional - used to specify the child/destination frame. Defaults to the frame ID stored in the topic
            selection_method: <method_2> # optional - used to specify the method to determine the transform associated with each message within the topic. Options are exact, recent, nearest, and interpolate. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
            lookup_limit: <seconds_2> # optional - used to specify a lookup limit when determining the transform. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
            chain_limit: <seconds_2> # optional - used to specify a transform chain differential limit when determining the transform. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
            filename: <filename_2> # optional - used to specify a filename, the file extension .csv will be appended. Defaults to topic_<id>.csv
            output_destination: <relative_destination_2> # optional - used to specify a directory relative to the root output directory to save the CSV file. Defaults to the root output directory.


A Complete Example
^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

    treet_root: map

    transform_0:
        parent_frame: map
        child_frame: base_link
        output_destination: data_files
        filename: poses

    camera_info_0:
        topic_name: /pylon_camera_node/camera_info
        filename: camera_info
        output_destination: data_files

    topic_0:
        topic_name: /os1_cloud_node/points
        message_type: sensor_msgs/PointCloud2
        filename_template: pcd_%06d-<ros_timestamp>
        output_destination: pointclouds
        transform:
            - parent_frame: map
            child_frame: ouster1/os1_lidar
            selection_method: interpolate
            lookup_limit: 0.5
            chain_limit: 1.0
            filename: pointclouds_map_poses
            output_destination: data_files
            - parent_frame: base_link
            child_frame: ouster1/os1_lidar
            selection_method: interpolate
            lookup_limit: None
            chain_limit: None
            filename: pointclouds_baselink_poses
            output_destination: data_files

    topic_1:
        topic_name: /pylon_camera_node/image_raw
        message_type: sensor_msgs/Image
        filename_template: image_%06d-<ros_timestamp>
        output_destination: images
