#!/usr/bin/env python3

### IMPORT MODULES ###
import sys
import csv
import yaml
import dill
# import pickle as dill
import pathlib
import numpy as np
# from enum import Enum
from tqdm import tqdm
# from typing import Union

# import treelib
import spatialmath as sm
import open3d as o3d
# from open3d_ros_helper import open3d_ros_helper as orh

import cv2
from cv_bridge import CvBridge, CvBridgeError

from .rosbag_transformer import ROSBagTransformer


from enum import IntEnum
class StateKeys(IntEnum):
    CleanState = 0
    TransformTreeBuilt = 1

### ROSBAG EXTRACTOR CLASS ###
class ROSBagExtractor:
    """
    A Class that extracts data from a ROS Bag file given an extraction config
    """
    def __init__(self, bag, extraction_config : dict, root_output_dir : str):
        """Constructs and pre-processes the data structure for further extraction.

        Args:
            bag (rosbag object): Opened ROSbag object containing data to be extracted
            extraction_config (dict): Dictionary of extraction config parameters 
            root_output_dir (str) : Root output directory to write data. 
        """

        # Setup variables
        self.bag = bag
        self.extraction_config = extraction_config
        self.root_output_dir = pathlib.Path(root_output_dir)
        self.bag_transformer = ROSBagTransformer(bag) # an object containing of all the transforms

        # Variables used for storing bag transformer object between executions
        self._bag_transformer_file = self.root_output_dir / "extraction_statefile.pickle"


    def extract_data(self, transform_topic : str="/tf", static_transform_topic : str="/tf_static", save_progress : bool = True):
        """Extracts the data from the ROSBag.

        Args:
            transform_topic (str, optional): used to specify the transform topic. Defaults to "/tf".
            static_transform_topic (str, optional): used to specify the transform topic. Defaults to "/tf_static".
            save_progress (bool, optional): used to specify if wish to save progress as a pickle file to the root output directory. Defaults to true.
        """
       
        # Pre-process bag transforms
        print("Processing the transforms")
        if not self._bag_transformer_file.exists():
            self.bag_transformer.build_transform_tree(transform_topic, static_transform_topic)
            self._save_bag_transformer()
        else:
            self._load_bag_transformer()

        # Extract dynamic transform data
        print("Extracting Transform Data")
        self._extract_dynamic_transform_data()

        # Extract camera info data
        print("Extracting Camera Info Data")
        self._extract_camera_info()

        # Extract sensor data
        print("Extracting Topic Data")
        self._extract_topic_data()

        print("\nROSBag Extraction Complete")
        

    def _save_bag_transformer(self):
        # Go pickle yourself
        with open(self._bag_transformer_file, 'wb') as f:
            dill.dump(self.bag_transformer, f)
            f.close()

    def _load_bag_transformer(self):
        # Only load if exists
        if not self._bag_transformer_file.exists():
            return

        # Open data
        with open(self._bag_transformer_file, 'rb') as f:
            self.bag_transformer = dill.load(f) 
            f.close()


    def _extract_dynamic_transform_data(self):
        """Extracts the dynamic transforms from the ROSBag. 
        """

        # Loop through all "transform_<number>" keys in the extraction config dictionary
        transform_keys = [x for x in self.extraction_config.keys() if "transform_" in x]

        for transform_key in transform_keys:

            # get data from extraction config
            parent_frame = self.extraction_config[transform_key]["parent_frame"]
            child_frame = self.extraction_config[transform_key]["child_frame"]

            print("\tExtracting %s to %s transform list"%(parent_frame, child_frame))

            # set output destination and filename
            filename = transform_key + ".csv" # default
            if "filename" in self.extraction_config[transform_key].keys():
                filename = self.extraction_config[transform_key]["filename"] + ".csv"

            output_dir = self.root_output_dir # default
            if "output_destination" in self.extraction_config[transform_key].keys():
                output_dir = self.root_output_dir / self.extraction_config[transform_key]["output_destination"]
                output_dir.mkdir(parents=True, exist_ok=True)
            
            # look up when transform updates
            transform_list, static_transform_chain = self.bag_transformer.lookup_transforms(parent_frame, child_frame)

            # open csv file
            with open(output_dir / filename, 'w', newline='') as csvfile:

                # create csvwriter object and write header
                csvwriter = csv.writer(csvfile, delimiter=',')
                csvwriter.writerow(["parent_frame", "child_frame", "timestamp", "pos_x", "pos_y", "pos_z", "quat_x", "quat_y", "quat_z", "quat_w"])

                # Write transform list
                for transform in transform_list:
                    position = transform[2].A[:3, -1]
                    quat = sm.base.r2q(transform[2].A[:3,:3])

                    csvwriter.writerow([parent_frame, child_frame, transform[0], 
                    position[0], position[1], position[2], 
                    quat[0], quat[1], quat[2], quat[3]])

    def _extract_camera_info(self):
        """Extracts the camera info data from the ROSBag.
        """

        # Loop through all "camera_info_<number>" keys in the extraction config dictionary
        camera_info_keys = [x for x in self.extraction_config.keys() if "camera_info_" in x]

        for camera_info_key in camera_info_keys:
            # get camera info topic
            camera_info_topic = self.extraction_config[camera_info_key]["topic_name"]

            print("\tExtracting camera info from %s"%(camera_info_topic))

            # set filename and output destination
            filename = camera_info_key + ".yaml" # default
            if "filename" in self.extraction_config[camera_info_key].keys():
                filename = self.extraction_config[camera_info_key]["filename"] + ".yaml"

            output_dir = self.root_output_dir # default
            if "output_destination" in self.extraction_config[camera_info_key].keys():
                output_dir = self.root_output_dir / self.extraction_config[camera_info_key]["output_destination"]
                output_dir.mkdir(parents=True, exist_ok=True)

            # Get and save camera info
            for _, msg, t in self.bag.read_messages(topics=[camera_info_topic]):

                # Save camera info 
                with open(output_dir / filename, 'w+') as output:                                        
                    y = yaml.load(str(msg), Loader=yaml.FullLoader)
                    yaml.dump(y, output)
                    pass
                break # camera info stays static


    def _extract_topic_data(self):
        """Extracts the topic data from the ROSBag.
        """

        # Loop through all "topic_<number>" keys in the extraction config dictionary
        topic_keys = [x for x in self.extraction_config.keys() if "topic_" in x]

        for topic_key in topic_keys:

            # set ouput directory and create
            output_dir = self.root_output_dir / topic_key # default
            if "output_destination" in self.extraction_config[topic_key].keys():
                output_dir = self.root_output_dir / self.extraction_config[topic_key]["output_destination"]
            output_dir.mkdir(parents=True, exist_ok=True)

            # extract topic data and return filelist
            print("\tExtracting topic %s data"%(self.extraction_config[topic_key]['topic_name']))
            if self.extraction_config[topic_key]["message_type"].lower() == "sensor_msgs/image":
                filelist, topic_frame_id = self._extract_image_topic(topic_key, output_dir)

            elif self.extraction_config[topic_key]["message_type"].lower() == "sensor_msgs/compressedimage":
                filelist, topic_frame_id = self._extract_image_compressed_topic(topic_key, output_dir)

            elif self.extraction_config[topic_key]["message_type"].lower() == "sensor_msgs/pointcloud2":
                filelist, topic_frame_id = self._extract_pointcloud_2_topic(topic_key, output_dir)

            else:
                print("Unknown message type %s. Ignoring topic data extraction"%(self.extraction_config[topic_key]['topic_name']))

            # output transform list if required
            if "transform" in self.extraction_config[topic_key].keys():
                print("\tWriting out transform file for topic %s"%(self.extraction_config[topic_key]['topic_name']))
                self._write_topic_transform_file(topic_key, topic_frame_id, filelist, output_dir)
                

    def _get_common_topic_extraction_data(self, topic_key : str):
        # Get topic name 
        topic_name = self.extraction_config[topic_key]["topic_name"]

        # Get filename
        filename = topic_name # default
        if "filename" in self.extraction_config[topic_key].keys():
            filename = self.extraction_config[topic_key]["filename"]

        # Get filename template
        filename_template = "frame_%06d" # default
        if "filename_template" in self.extraction_config[topic_key].keys():
            filename_template = self.extraction_config[topic_key]["filename_template"]

        return topic_name, filename, filename_template


    def _extract_image_topic(self, topic_key : str, output_dir : pathlib.Path) -> tuple:
        """Extracts an image topic (ROS message type sensor_msgs/Image) from the ROSBag.

        Args:
            topic_key (str): the topic key (e.g., topic_0) used to identify the portion
                of the extraction config data containing the extraction parameters.
            output_dir (pathlib.Path): the output directory to write this data to

        Returns:
            tuple: returns a tuple(list, str). The list contains the filenames 
                and timestamps [filaname (str), timestamp (float)]. The string
                contains the frame ID for the topic.
        """

        # Variables
        frame_list = []
        idx = 0
        cv_bridge = CvBridge()

        # Get topic name and filename template
        topic_name, _, filename_template = self._get_common_topic_extraction_data(topic_key)

        # Loop through bag
        current_time = 0
        total_time = int(self.bag.get_end_time() - self.bag.get_start_time())
        with tqdm(total=total_time) as pbar:
            for topic, msg, t in self.bag.read_messages(topics=[topic_name]):
                # Update the progress bar
                if current_time < int(t.to_sec() - self.bag.get_start_time()):
                    n = int(t.to_sec()- self.bag.get_start_time()) - current_time
                    current_time += n
                    pbar.update(n)
                else:
                    pbar.update(0)

                # Create image filename
                timestamp_str = str(t.to_sec()).replace(".", "_")
                image_filename = (filename_template%(idx)).replace("<ros_timestamp>", timestamp_str) + ".jpeg"

                # Save image, but first need to convert to OpenCV image
                try:
                    cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                    cv2.imwrite(str(output_dir / image_filename), cv_image)
                except CvBridgeError:
                    rospy.logwarn("Unable to convert image {image_count}")

                # Add image to image_list and increase idx
                frame_list.append([image_filename, t.to_sec()])
                idx += 1

        # get topic frame id and return list 
        topic_frame_id = msg.header.frame_id
        return frame_list, topic_frame_id

    
    def _extract_image_compressed_topic(self, topic_key, output_dir):
        """Extracts a compressed image topic (ROS message type sensor_msgs/CompressedImage) 
        from the ROSBag.

        Args:
            topic_key (str): the topic key (e.g., topic_0) used to identify the portion
                of the extraction config data containing the extraction parameters.
            output_dir (pathlib.Path): the output directory to write this data to

        Returns:
            tuple: returns a tuple(list, str). The list contains the filenames 
                and timestamps [filaname (str), timestamp (float)]. The string
                contains the frame ID for the topic.
        """

        # Variables
        frame_list = []
        idx = 0

        # Get topic name and filename template
        topic_name, _, filename_template = self._get_common_topic_extraction_data(topic_key)

        # Loop through bag
        current_time = 0
        total_time = int(self.bag.get_end_time() - self.bag.get_start_time())
        with tqdm(total=total_time) as pbar:
            for topic, msg, t in self.bag.read_messages(topics=[topic_name]):
                # Update the progress bar
                if current_time < int(t.to_sec() - self.bag.get_start_time()):
                    n = int(t.to_sec()- self.bag.get_start_time()) - current_time
                    current_time += n
                    pbar.update(n)
                else:
                    pbar.update(0)

                # Create image filename
                timestamp_str = str(t.to_sec()).replace(".", "_")
                image_filename = (filename_template%(idx)).replace("<ros_timestamp>", timestamp_str) + ".jpeg"

                # Save image directly
                image_file = open(output_dir / image_filename, "wb+")
                image_file.write(msg.data)
                image_file.close()

                # Add image to image_list and increase idx
                frame_list.append([image_filename, t.to_sec()])
                idx += 1

        # get topic frame id and return list 
        topic_frame_id = msg.header.frame_id
        return frame_list, topic_frame_id

    
    def _extract_pointcloud_2_topic(self, topic_key, output_dir):
        """Extracts a point cloud topic (ROS message type sensor_msgs/PointCloud2) 
        from the ROSBag.

        Args:
            topic_key (str): the topic key (e.g., topic_0) used to identify the portion
                of the extraction config data containing the extraction parameters.
            output_dir (pathlib.Path): the output directory to write this data to

        Returns:
            tuple: returns a tuple(list, str). The list contains the filenames 
                and timestamps [filaname (str), timestamp (float)]. The string
                contains the frame ID for the topic.
        """

        # Variables
        frame_list = []
        idx = 0

        # Get topic name and filename template
        topic_name, _, filename_template = self._get_common_topic_extraction_data(topic_key)

        # Loop through bag
        current_time = 0
        total_time = int(self.bag.get_end_time() - self.bag.get_start_time())
        with tqdm(total=total_time) as pbar:
            for _, msg, t in self.bag.read_messages(topics=[topic_name]):
                # Update the progress bar
                if current_time < int(t.to_sec() - self.bag.get_start_time()):
                    n = int(t.to_sec()- self.bag.get_start_time()) - current_time
                    current_time += n
                    pbar.update(n)
                else:
                    pbar.update(0) 

                # Create pointcloud filename
                timestamp_str = str(t.to_sec()).replace(".", "_")
                pcd_filename = (filename_template%(idx)).replace("<ros_timestamp>", timestamp_str) + ".ply"

                # Decode and save the point cloud
                # o3d_pcd = orh.rospc_to_o3dpc(msg) this throws an error message, not sure why haven't dug into it
                o3d_pcd = rospc_to_o3dpc(msg)
                o3d.io.write_point_cloud(str(output_dir / pcd_filename), o3d_pcd) 

                # Add image to image_list and increase idx
                frame_list.append([pcd_filename, t.to_sec()])
                idx += 1

        # get topic frame id and return list 
        topic_frame_id = msg.header.frame_id
        return frame_list, topic_frame_id

    
    def _write_topic_transform_file(self, topic_key : str, topic_frame_id : str, filelist : list, output_dir : pathlib.Path):
        """Gets and writes the transforms for a file list (e.g., returned from _extract_image_topic) to a CSV file. 

        Args:
            topic_key (str): the topic key (e.g., topic_0) used to identify the portion
                of the extraction config data containing the extraction parameters.
            topic_frame_id (str): the frame ID of the topic.
            filelist (list): the file list containing the filenames and timestamps.
            output_dir (pathlib.Path): the output directory to write this data to.
        """

        # get transform data from extraction config
        parent_frame = self.extraction_config[topic_key]["transform"]["parent_frame"]

        child_frame = topic_frame_id # default to the frame id in the message for the child frame
        if "child_frame" in self.extraction_config[topic_key]["transform"].keys():
            child_frame = self.extraction_config[topic_key]["transform"]["child_frame"]

        # get transform selection method type and limits
        method = "interpolate" # default
        if "selection_method" in self.extraction_config[topic_key]["transform"].keys():
            method = self.extraction_config[topic_key]["transform"]["selection_method"]

        lookup_limit = None # default
        if "lookup_limit" in self.extraction_config[topic_key]["transform"].keys():
            lookup_limit = self.extraction_config[topic_key]["transform"]["lookup_limit"]

        chain_limit = None # default
        if "chain_limit" in self.extraction_config[topic_key]["transform"].keys():
            chain_limit = self.extraction_config[topic_key]["transform"]["chain_limit"]

        # get transform for each
        filelist_with_pose = []
        for frame in tqdm(filelist):
            timestamp, chain_differential, transform, status = self.bag_transformer.lookup_transform(parent_frame, child_frame, frame[1],
                method=method, lookup_limit=lookup_limit, chain_limit=chain_limit)
            filelist_with_pose.append([frame[0], timestamp, transform, status])

        # set output destination and filename
        filename = topic_key + ".csv" # default
        if "filename" in self.extraction_config[topic_key]["transform"].keys():
            filename = self.extraction_config[topic_key]["transform"]["filename"] + ".csv"

        # default output directory is same as where the topic data is saved
        if "output_destination" in self.extraction_config[topic_key]["transform"].keys():
            output_dir = self.root_output_dir / self.extraction_config[topic_key]["transform"]["output_destination"]
            output_dir.mkdir(parents=True, exist_ok=True)

        # open csv file
        with open(output_dir / filename, 'w', newline='') as csvfile:

            # create csvwriter object and write header
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow(["filename", "parent_frame", "child_frame", "timestamp", "pos_x", "pos_y", "pos_z", "quat_x", "quat_y", "quat_z", "quat_w", "status"])

            # Write transform list
            for data in filelist_with_pose:
                if type(data[2]) == sm.pose3d.SE3: # make sure transform isn't none
                    print(data)
                    timestamp = data[1]
                    position = data[2].A[:3, -1]
                    quat = sm.base.r2q(data[2].A[:3,:3])
                else:
                    timestamp = "None"
                    position = ["None"]*3
                    quat = ["None"]*4

                csvwriter.writerow([data[0], parent_frame, child_frame, timestamp, 
                position[0], position[1], position[2], 
                quat[0], quat[1], quat[2], quat[3], data[3].name])
    
    

### HELPER FUNCTIONS ###
import ros_numpy

# the function rospc_to_o3dpc in the open3d_ros_helper 
# module throws an error for some reason. Here is a direct 
# copy of the function, and no error is thrown.

def rospc_to_o3dpc(rospc, remove_nans=False):
    """ covert ros point cloud to open3d point cloud
    Args: 
        rospc (sensor.msg.PointCloud2): ros point cloud message
        remove_nans (bool): if true, ignore the NaN points
    Returns: 
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
    """
    field_names = [field.name for field in rospc.fields]
    is_rgb = 'rgb' in field_names
    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(rospc).ravel()
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]
    if is_rgb:
        cloud_npy = np.zeros(cloud_array.shape + (4,), dtype=np.float)
    else: 
        cloud_npy = np.zeros(cloud_array.shape + (3,), dtype=np.float)
    
    cloud_npy[...,0] = cloud_array['x']
    cloud_npy[...,1] = cloud_array['y']
    cloud_npy[...,2] = cloud_array['z']
    o3dpc = o3d.geometry.PointCloud()

    if len(np.shape(cloud_npy)) == 3:
        cloud_npy = np.reshape(cloud_npy[:, :, :3], [-1, 3], 'F')
    o3dpc.points = o3d.utility.Vector3dVector(cloud_npy[:, :3])

    if is_rgb:
        rgb_npy = cloud_array['rgb']
        rgb_npy.dtype = np.uint32
        r = np.asarray((rgb_npy >> 16) & 255, dtype=np.uint8)
        g = np.asarray((rgb_npy >> 8) & 255, dtype=np.uint8)
        b = np.asarray(rgb_npy & 255, dtype=np.uint8)
        rgb_npy = np.asarray([r, g, b])
        rgb_npy = rgb_npy.astype(np.float)/255
        rgb_npy = np.swapaxes(rgb_npy, 0, 1)
        o3dpc.colors = o3d.utility.Vector3dVector(rgb_npy)
    return o3dpc
