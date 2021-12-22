#!/usr/bin/env python3

### IMPORT MODULES ###
# import sys
# import csv
# import yaml
# import pathlib
import numpy as np
from enum import Enum
from tqdm import tqdm
from typing import Union

import treelib
import spatialmath as sm

import rospy
from geometry_msgs.msg import Transform

### HELPER CLASSES ###
class Status(Enum):
    SUCCESS = 0
    UNKNOWN_METHOD = 1
    NO_MATCHING_TRANSFORM = 2
    LOOKUP_LIMIT_ERROR = 3
    CHAIN_LIMIT_ERROR = 4

class TransformData:

    def __init__(self, timestamp : rospy.rostime.Time, pose : Transform):
        
        self.timestamp = timestamp.to_sec()

        # set transform using spatial maths library
        self.transform = sm.SE3([pose.translation.x, pose.translation.y, pose.translation.z])
        self.transform.A[:3, :3] = sm.base.q2r([pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z])


class Transforms(object):

    def __init__(self, parent_frame : str, child_frame : str, static_transform : bool = False):

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.transforms = []
        self.static_transform = static_transform

    def add_transform(self, timestamp : rospy.rostime.Time, pose : Transform, static_transform : bool = None):

        if static_transform != None:
            # update static transform attribute if required
            self.static_transform = static_transform

        if self.static_transform:
            self.transforms = [TransformData(timestamp, pose)] # can only be one transform for static transform topics
        else:
            self.transforms.append(TransformData(timestamp, pose))

### ROSBAG TRANSFORMER CLASS ###
class ROSBagTransformer:
    """
    A Class that processes the transform data from a ROS Bag file
    """
    def __init__(self, bag):
        """Constructs and initialises a ROSBagTransform object.

        Args:
            bag (rosbag object): an opened ROSbag object containing the transform data to be processed.
        """

        # Class Variable Descriptions
        # self._bag - opened rosbag file
        # self._frames_dict - a dictionary where the key is a transfer frame and 
        #       the value is a list containing all child frames of that transfer frame. Is used
        #       to build the self._transforms_tree, as the Tree needs to be built from the root and
        #       the transforms messages will not necessarily be in correct order.
        # self._transforms_tree - a Tree object containing the transfer frame structure.
        #       The identifier for each node is the transfer frame. The transforms between 
        #       a parent and a child frame are stored within the child frame node's data attribute
        #       as a Transforms object. To access the data self._transforms_tree.get_node(<id>).data,
        #       to access a specific transform self._transforms_tree.get_node(<id>).data.transforms[<index>]
        # self._transforms - a dictionary where the key is a transfer frame name, same as the tree node IDs,
        #       the values are Transforms object. The value for each key is linked to the data attribute
        #       within each node within the self._transforms_tree Tree object, after the tree has been built.
        # self._all_transforms - is a list containing all transforms and has the following row setup 
        #       [[parent_frame, child_frame], timestamp, TransformData Object]. Order is based on timestamp

        # Setup variables
        self._bag = bag
        self._frames_dict = {}
        self._transforms_tree = treelib.Tree()
        self._transforms = {}
        self._all_transforms = []
        

    def build_transform_tree(self, transform_topic : str = "/tf", static_transform_topic : str ="/tf_static"):
        """Processes the data in the supplied ROSBag, passed during object construction, and builds the
        transform tree. This function must be called prior to any other class methods.

        Args:
            transform_topic (str, optional): used to specify the transform topic. Defaults to "/tf".
            static_transform_topic (str, optional): used to specify the transform topic. Defaults to "/tf_static".
        """

        # Build transform tree
        print("\nBuilding the transform tree")
        self._build_transform_tree(transform_topic, static_transform_topic)

        print("\nBuilt the following transform tree")
        self._transforms_tree.show()

    
    def _build_transform_tree(self, transform_topic : str, static_transform_topic : str):
        """A private function used to help process the transform data in the supplied ROSBag.
         This function must be called prior to any other class methods.

        Args:
            transform_topic (str, optional): used to specify the transform topic. Defaults to "/tf".
            static_transform_topic (str, optional): used to specify the transform topic. Defaults to "/tf_static".
        """

        # Loop through all tf and static tf messages and get all frames that have children,
        # store data as dictionary in self._frame_dict
        total_time = int(self._bag.get_end_time() - self._bag.get_start_time())
        current_time = 0
        with tqdm(total=total_time) as pbar:
            for topic, msg, t in self._bag.read_messages(topics=[transform_topic, static_transform_topic]):
                # Update the progress bar
                if current_time < int(t.to_sec() - self._bag.get_start_time()):
                    n = int(t.to_sec()- self._bag.get_start_time()) - current_time
                    current_time += n
                    pbar.update(n)
                else:
                    pbar.update(0)

                # get parent and child frame and if it's a static or dynamic transform topic
                msg = msg.transforms[0]
                parent_frame = msg.header.frame_id.strip("/")
                child_frame = msg.child_frame_id.strip("/")
                static_transform = topic == static_transform_topic

                # add parent frame as a key the dict if it does not exist
                if parent_frame not in self._frames_dict.keys():
                    self._frames_dict[parent_frame] = []

                # add child as a value to parent if it does not exist
                if child_frame not in self._frames_dict[parent_frame]:
                    self._frames_dict[parent_frame].append(child_frame)

                # add child as key to transform dictionary if it doesn't exist
                if child_frame not in self._transforms.keys():
                    self._transforms[child_frame] = Transforms(parent_frame, child_frame, static_transform)

                # add the transform to the transforms object associated with this child frame
                self._transforms[child_frame].add_transform(t, msg.transform)

                # add the transform to the list of all transforms
                self._all_transforms.append([[parent_frame, child_frame], t.to_sec(), self._transforms[child_frame].transforms[-1]])

        # sort all transforms into time order
        self._all_transforms = sorted(self._all_transforms, key=lambda x: x[1])


        # Determine the absolute root of the tree
        for tmp_root_frame in self._frames_dict.keys():
            found_key = False
            for searching_parent_frames in self._frames_dict.keys():
                if tmp_root_frame in self._frames_dict[searching_parent_frames]:
                    found_key = True
                    break
        
            if found_key == False:
                root = tmp_root_frame
                break
        
        # Establish root of the transform tree
        self._transforms_tree.create_node(tag=root, identifier=root)

        # Add child frames, starting with root transfer frame, done in recursive manner
        self._add_child_frames(root)

        # Change tag of all child frames to static if required



    def _add_child_frames(self, parent : str):
        """A private function to be used recursively with the self._frames_dict
            to add nodes to the transforms tree class variable.

        Args:
            parent (str): [description]
        """

        # get child frames of parent
        child_frames = self._frames_dict[parent]

        for child_frame in child_frames:
            # create node and add data as reference to the value within the transforms dictionary
            tag = child_frame
            if self._transforms[child_frame].static_transform:
                tag = "%s (static)"%(child_frame)
            self._transforms_tree.create_node(tag=tag, identifier=child_frame, parent=parent, data=self._transforms[child_frame])

            # do the child frames of this child, if it has any
            if child_frame in self._frames_dict.keys():
                self._add_child_frames(child_frame)
        
    
    def static_transform(self, parent : str, child : str) -> bool:
        """determines if parent to child transform chain is static.

        Args:
            parent (str): the name of the parent frame
            child (str): the name of the child frame (does not need to be directly connected to the parent)

        Returns:
            bool: returns true if the parent to child transform chain is static.
        """

        # check to see if each transform link in the chain is
        transform_chain = self.get_chain(parent, child)
        for transform_pair in transform_chain:
            if not self._transforms_tree.get_node(transform_pair[1]).data.static_transform:
                return False

        # return true
        return True


    def get_transform_tree_frames(self) -> dict:
        """returns the transform tree as a dictionary

        Returns:
            dict: the transform tree
        """

        return self._transforms_tree.to_dict()

    def get_chain(self, parent : str, child : str, return_type : type = tuple) -> Union[tuple, list]:
        """gets the transform chain from a parent to a child frame. The child frame does not need to
        be directly connected to the parent.

        Args:
            parent (str): the name of the parent frame
            child (str): the name of the child frame (does not need to be directly connected to the parent)
            return_type (type, optional): used to specify the return type. Defaults to tuple.

        Returns:
            Union[tuple, list]: returns either a tuple or list. If the return_type is set to tuple,
                the returned value will contain a tuple of pairs holding the names of directly 
                connected parent to child frames within the chain. If the return_type is set to 
                a list, the returned value will be an ordered list containing the names of the 
                frames in the transform chain.
        """

        retval_list = []
        retval_tuple = []

        if parent == child:
            return None
        
        # build up transform list
        current = child 
        while self._transforms_tree.parent(current) != None:
            parent = self._transforms_tree.parent(current).identifier
            retval_tuple.append([parent, current])
            retval_list.append(current)
            current = parent
        retval_list.append(parent)

        # return
        if return_type == list:
            return list(reversed(retval_list))
        elif return_type == tuple:
            return tuple(reversed(retval_tuple))

    def lookup_transforms(self, parent : str, child : str) -> tuple:
        """looks up and returns the transforms between a parent and child frame. 
            If the transform is a chain (i.e., the parent and child are not directly 
            connected) the transform will be updated whenever one of the transforms 
            in the chain is updated.

        Args:
            parent (str): the name of the parent frame
            child (str): the name of the child frame (does not need to be directly connected to the parent)

        Returns:
            tuple: returned tuple with types (list, bool). The list contains the 
                transforms between the parent and child. The  list will have the form 
                [timestamp (float), chain_differential (float), transform (SE3 object)]
                where timestamp is the most recent timestamp in the transform chain used
                to create the entire transform. The chain_differential is the difference
                between the most recent and oldest timestamp in the transform chain used
                to create the overall transform. The transform is an SE3 object containing
                the total transform between the parent and child frame. 
                The boolean represents if the entire chain is static.
        """

        transforms_list = []

        # get all frames within the chain
        transform_chain = self.get_chain(parent, child)

        # check to see if entire transform chain is static
        static_transform_chain = True
        for transform_pair in transform_chain:
            if not self.static_transform(transform_pair[0], transform_pair[1]):
                static_transform_chain = False
                break

        # if the frames are directly connected, get the transform from the child
        # node and return the transform list
        if self._transforms_tree.parent(child).identifier == parent:
            transforms = self._transforms_tree.get_node(child).data.transforms
            for transform in transforms:
                transforms_list.append([transform.timestamp, 0.0, transform.transform])

            return transforms_list, static_transform_chain

        # create transform list containing most recent transform for each
        # transform pair in the chain. Index corresponds to the pair
        # within the chain and the value holds a TransformData object
        most_recent_transforms = [None]*len(transform_chain)
        for transform in self._all_transforms:
            # Check to see if this transform is within the transform
            # chain. If it isn't continue to the next transform
            if not transform[0] in transform_chain:
                continue

            # Get the index in the transform chain associated with
            # this transform and set the appropriate index in
            # the most_recent_transforms to this TransformData object
            idx = transform_chain.index(transform[0])
            most_recent_transforms[idx] = transform[2]

            # only start creating the transform list, to be returned,
            # after the entire chain has been completed
            if not None in most_recent_transforms:
                # setup variables
                tot_transform = most_recent_transforms[0].transform
                # timestamps holds the timestamp and whether it is a static transform
                timestamps = np.empty((len(transform_chain), 2), dtype=np.float64) 
                timestamps[0,0] = most_recent_transforms[0].timestamp
                timestamps[0,1] = self.static_transform(transform_chain[0][0], transform_chain[0][1])
                
                # loop through remaining transforms in most recent transforms list
                for idy, x in enumerate(most_recent_transforms[1:], start=1):
                    tot_transform = x.transform * tot_transform # total transform of the chain 
                    timestamps[idy, 0] = x.timestamp
                    timestamps[idy,1] = self.static_transform(transform_chain[idy][0], transform_chain[idy][1])
                
                # determine chain time differential, need to consider static transforms
                if np.all(timestamps[:,1] == True):
                    # all are static transforms, take most recent and set different to 0
                    timestamp = np.max(timestamps[:,0])
                    timestamp_diff = 0.0
                else:
                    # only utilise dynamic transforms to for timestamp 
                    dynamic_timestamps = timestamps[timestamps[:,1] == False]
                    timestamp = np.max(dynamic_timestamps[:,0])
                    timestamp_diff = timestamp - np.min(dynamic_timestamps[:,0])

                # append to transform list [timestamp, largest_timestamp_differential_in_chain, total_transform]
                transforms_list.append([timestamp, timestamp_diff, tot_transform])

                # if entire chain is static will never change, so can return now
                if static_transform_chain:
                    break

        # return transforms, sorted based on timestamp (should already be sorted but, just in case)
        return sorted(transforms_list, key=lambda x : x[0]), static_transform_chain

    def lookup_transform(self, parent : str, child : str, time : float, method : str="nearest", lookup_limit : float=None, chain_limit : float=None) -> tuple:
        """Looks up and returns transform between a parent and child frame for a given time. If the entire transform chain
            is static then the method, and limit arguments are ignored.

        Args:
            parent (str): the name of the parent frame
            child (str): the name of the child frame (does not need to be directly connected to the parent)
            time (float): the time at which to look up the transform
            method (str, optional): the method to use when looking up the transform. 
                Options include "exact", "recent", "nearest", and "interpolate". Defaults to "nearest".
                exact - will attempt to get an exact transform based on the desired time.
                recent - will return a transform only if there is a past transform within the lookup_limit.
                nearest - will return a future or past transform if there is one within the lookup_limit.
                interpolate - will return an interpolated transform only if there is a transform either side
                    of the desired time within the lookup_limit.
            lookup_limit (float, optional): the limit to use when looking up a transform. If a transform 
            between the parent and child frame cannot be found within this limit a Status.LOOKUP_LIMIT_ERROR 
            will be returned. Defaults to None.
            chain_limit (float, optional): if the selected transform has a chain differential greater than 
            this value a Status.CHAIN_LIMIT_ERROR will be returned. Defaults to None.

        Returns:
            tuple: returns a tuple containing the [timestamp, chain_differential, transform, status].
                the timestamp will be that of the selected transform for the exact, recent and nearest methods,
                and be equal to the desired time for the interpolate method. The chain_differential will be
                that of the selected transform for the exact, recent and nearest methods, and be the maximum
                differential of the two transforms used in the interpolate method.
        """

        # get the transform list for the specified chain
        transform_list, static_transform_chain = self.lookup_transforms(parent, child)

        # if entire transform chain is static, transform never changes
        if static_transform_chain:
            # return desired time, chain differential of 0.0, transform and success
            time, 0.0, transform_list[0][2], Status.SUCCESS

        # convert the list to a numpy array
        transform_list = np.asarray(transform_list)

        # get time difference between the list and desired time
        time_diff = transform_list[:,0] - time
        
        # outcome depends on method
        if method.lower() == "exact":
            if np.all(time_diff != 0):
                # no transform with exact timestamp
                return None, None, None, Status.NO_MATCHING_TRANSFORM  
            
            # get the timestamp, chain differential and transform
            selected_idx = np.argmin(np.abs(time_diff))
            timestamp = transform_list[selected_idx][0]
            chain_dif = transform_list[selected_idx][1]
            transform = transform_list[selected_idx][2]   

        elif method.lower() == "recent":
            if np.all(time_diff > 0):
                # no transform in past
                return None, None, None, Status.NO_MATCHING_TRANSFORM 
            
            # get the timestamp, chain differential and transform
            selected_idx = np.argmax(time_diff[time_diff <= 0])
            timestamp = transform_list[selected_idx][0]
            chain_dif = transform_list[selected_idx][1]
            transform = transform_list[selected_idx][2] 

        elif method.lower() == "nearest":
            # get the timestamp, chain differential and transform
            selected_idx = np.argmin(np.abs(time_diff))
            timestamp = transform_list[selected_idx][0]
            chain_dif = transform_list[selected_idx][1]
            transform = transform_list[selected_idx][2]  

        elif method.lower() == "interpolate":
            if np.all(time_diff < 0) or np.all(time_diff > 0):
                # no transform on either side of desired time
                return None, None, None, Status.NO_MATCHING_TRANSFORM
            
            # get the index on each side of 0 time difference
            negative_idx = np.argmax(time_diff[time_diff <= 0])
            positive_idx = negative_idx + 1

            # make sure both values are within the lookup limit
            if lookup_limit != None and np.any(time_diff[negative_idx:positive_idx+1] > lookup_limit):
                return None, None, None, Status.LOOKUP_LIMIT_ERROR

            # get where the desired timestamp is between the two transform points in the range 0 to 1
            # use numpy interp function for this
            point = np.interp(time, [transform_list[negative_idx][0], transform_list[positive_idx][0]], [0.0, 1.0])
            
            # interpolate the position
            transform = transform_list[negative_idx][2].interp(transform_list[positive_idx][2], point)

            # set timestamp, to be the desired time
            timestamp = time

            # chain differential set to maximum of the two
            chain_dif = max(transform_list[negative_idx][1], transform_list[positive_idx][1])
        
        else:
            return None, None, None, Status.UNKNOWN_METHOD

        # check lookup and chain differential limits
        if lookup_limit != None and np.abs(timestamp - time) > lookup_limit:
            return None, None, None, Status.LOOKUP_LIMIT_ERROR
        elif chain_limit != None and chain_dif > chain_limit:
            return None, None, None, Status.CHAIN_LIMIT_ERROR

        # return
        return timestamp, chain_dif, transform, Status.SUCCESS