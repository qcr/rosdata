<!-- Created with QCR's code template tool: https://github.com/qcr/code_templates -->

# ROS Data: An Extraction Tool for ROSBags

[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
![Primary language](https://img.shields.io/github/languages/top/qcr/rosdata)
[![License](https://img.shields.io/github/license/qcr/rosdata)](./LICENSE.txt)


The ROS Data Python Package is a simple tool to extract, manipulate and visualise ROSBag Data. The manipulate tools coming soon. 

The tool can currently extract the following types of messages:

| **ROS Message Type** | **Ouput Format** |
|----------------------|------------------|
| tf2_msgs/TFMessage | CSV File |
| sensor_msgs/CompressedImage | Folder of Images, and if requested, a CSV with transforms |
| sensor_msgs/Image | Folder of Images, and if requested, a CSV with transforms |
| sensor_msgs/PointCloud2 | Folder of Point Clouds, and if requested, a CSV with transforms |
| sensor_msgs/CameraInfo | YAML File |

Details on how to extract this data can be found [below](#extraction-config-file)


## Installation

Conda is used to setup the appropriate environment/dependencies. We recommend the following procedure for the rosdata tool:

```
git clone https://github.com/qcr/rosdata.git
cd <rosdata_repo>
conda env create --file rosdata_env.yml
conda activate rosdata_env
pip install -e .
```

The rosdata tool can then be called from any directory using the command `rosdata <tool> <tool_arguments>`.

<!-- 
Currently, you will need to install dependencies manually. Main dependencies are numpy, treelib, rospy, rosbag, spatialmaths, open3d, and open3d_ros_helper.

```
conda env create --file envname.yml # Environment file not yet provided
conda activate rosdata_env # Not yet supported
pip install -e .
``` -->

## Usage

ROS Data currently has the following tools:

* **extract** - extracts data from a ROSBag given a user defined extraction config file.
* **visualise** - visualise poses contained within CSV file created via the extraction tool.

These tools can be called by running the following command, after installation:

```
rosdata <tool> <tool_arguments>
```

For example,

```
rosdata extract <extract_arguments>
```

Simply type in `-h` or `--help` to get a list of the available tools or the arguments for a specific tool. For example,

```
rosdata -h              # to show the list of available tools
rosdata <tool> --help   # to show the arguments for a specific tool
```

## Examples
A complete extraction example is provided. Run the following commands after installation to execute the example:

```
cd <rosdata_repo>/example
rosdata extract example.bag example_extraction_config.yaml data
```

After successful execution you will find a `<rosdata_repo>/example/data` folder containing the extracted items.

### Extraction Config File
The extraction config file is a YAML file. This file is used to specify what data should be extracted from a ROSBag and where it should be extracted. Currently only three types of high level keys are supported by the tool. These keys are `transorm_<id>, camera_info_<id>, and topic_<id>`, where `<id>` is simply a unique number defined by you. Each key is used to extract data in a specific format from a specific set of ROS message types. The set of message types associated with each key are:

| **Key** | **ROS Message Type** |
|---------|----------------------|
| `transform_<id>` | tf2_msgs/TFMessage | 
| `camera_info_<id>` || sensor_msgs/CameraInfo |
| `topic_<id>` | sensor_msgs/CompressedImage, sensor_msgs/Image, sensor_msgs/PointCloud2 | 

You can extract as many of each message type from your ROSBag by simply utilising unique IDs. For example, if you wish to extract a CSV file for the transform map to odom, as well as map to base_link you could simply have two transform keys (e.g., `transform_0` and `transform_1`), one for each.

The arguments for `transform_<id>` are:
```yaml
transform_<id>:
    parent_frame: <parent_frame> # required - used to specify the parent/origin frame
    child_frame: <child_frame> # required - used to specify a child/destination frame
    filename: <filename> # optional - used to specify a filename, the file extension .csv will be appended. Defaults to transform_<id>.csv
    output_destination: <relative_destination> # optional - used to specify a directory relative to the root output directory to save the CSV file. Defaults to the root output directory.
```

The arguments for `camera_info_<id>` are:
```yaml
camera_info_<id>:
    topic_name: <camera_info_topic> # required - used to specify the name ofthe camera info topic 
    filename: <filename> # optional - used to specify a filename, the file extension .yaml will be appended. Defaults to camera_info_<id>.csv
    output_destination: <relative_destination> # optional - used to specify a directory relative to the root output directory to save the YAML file. Defaults to the root output directory.
```

The arguments for `topic_<id>` are:
```yaml
topic_<id>:
    topic_name: <camera_info_topic> # required - used to specify the name of the camera info topic 
    message_type: <ros_message_type> # required - determines the data extraction method
    output_destination: <relative_destination> # optional - used to specify a directory relative to the root output directory to save the topic data. Defaults to the root_output_directory/topic_<id>.
    filename_template: <filename_template> # optional - used as a filename template string (e.g. `image_%06d-<ros_timestamp>`), the appropriate file_extenstion will be automatically appended. Only a single topic index and ROS timestamp can be included in the template. Use the Python `%d` string formatter, or derivate of, to specify the topic index and use `<ros_timestamp>` to include the ROS topic timestamp as a string which will be in the format `<seconds>_<nanaseconds>`. Defaults to `frame_%06d`
    transform: # optional - only required if wish to output a CSV containing transform data associated with the topic
        parent_frame: <parent_frame> # required - used to specify the parent/origin frame 
        child_frame: <child_frame>  # optional - used to specify the child/destination frame. Defaults to the frame ID stored in the topic
        selection_method: <method> # optional - used to specify the method to determine the transform associated with each message within the topic. Options are exact, recent, nearest, and interpolate. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
        lookup_limit: <seconds> # optional - used to specify a lookup limit when determining the transform. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
        chain_limit: <seconds> # optional - used to specify a transform chain differential limit when determining the transform. See lookup_transform in rosdata/rosbag_transforms.py for more details on methods.
        filename: <filename> # optional - used to specify a filename, the file extension .csv will be appended. Defaults to topic_<id>.csv
        output_destination: <relative_destination> # optional - used to specify a directory relative to the root output directory to save the CSV file. Defaults to the root output directory.
```

A complete example:

```yaml
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
    parent_frame: map
    child_frame: ouster1/os1_lidar
    selection_method: interpolate
    lookup_limit: 0.5
    chain_limit: 1.0
    filename: pointclouds
    output_destination: data_files

topic_1:
  topic_name: /pylon_camera_node/image_raw
  message_type: sensor_msgs/Image
  filename_template: image_%06d-<ros_timestamp>
  output_destination: images
```

## Acknowledgments

If you use the ROS Data tool in your work we would appreciate any form of acknowledgement.
