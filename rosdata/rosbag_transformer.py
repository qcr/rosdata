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

import dill
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
    SPATIALMATH_INTERP_ERROR = 5

class TransformData:

    def __init__(self, timestamp : rospy.rostime.Time, transform : Transform):
        
        self.timestamp = timestamp.to_sec()

        # set transform using spatial maths library
        self.transform = sm.SE3([transform.translation.x, transform.translation.y, transform.translation.z])
        self.transform.A[:3, :3] = sm.base.q2r([transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z])


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
    def __init__(self):
        """Constructs and initialises a ROSBagTransform object.
        """

        # Major Class Variable Descriptions
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
        self._frames_dict = {}
        self._transforms_tree = treelib.Tree()
        self._transforms = {}
        self._all_transforms = []

        # Following variables are used to store information from most recent self.lookup_transform call,
        # to increase following call
        self._previous_lookup_transforms_source = None
        self._previous_lookup_transforms_dest = None
        self._transforms_list = None
        self._static_transform_chain = None
        

        
    def _build_transform_tree(self, bag, transform_topic : str, static_transform_topic : str, tree_root : str = None):
        """A private function used to help process the transform data in the supplied ROSBag.
         This function must be called prior to any other class methods.

        Args:
            bag (rosbag object): an opened ROSbag object containing the transform data to be processed.
            transform_topic (str): used to specify the transform topic.
            static_transform_topic (str): used to specify the transform topic.
            tree_root (str, optional): used to specify the name of the frame that is the root of the tree. Defaults to none.

        Raises:
            RuntimeError: if there are multiple tree roots and a tree root was not specified.
            ValueError: if the specified tree root is not present in the transform tree.
        """


        # Loop through all tf and static tf messages and get all frames that have children,
        # store data as dictionary in self._frame_dict
        topic_count = bag.get_message_count(transform_topic) + bag.get_message_count(static_transform_topic)
        for topic, msg, t in tqdm(bag.read_messages(topics=[transform_topic, static_transform_topic]), total=topic_count):
            
            # check whether it is a static transform
            static_transform = topic == static_transform_topic

            # loop through all transforms in message
            for transform in msg.transforms:

                # get parent and child frame
                parent_frame = transform.header.frame_id.strip("/")
                child_frame = transform.child_frame_id.strip("/")

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
                self._transforms[child_frame].add_transform(t, transform.transform)

                # add the transform to the list of all transforms
                self._all_transforms.append([[parent_frame, child_frame], t.to_sec(), self._transforms[child_frame].transforms[-1]])

        # sort all transforms into time order
        self._all_transforms = sorted(self._all_transforms, key=lambda x: x[1])


        # Determine the absolute root of the tree
        if tree_root is None:
            roots = []
            for tmp_root_frame in self._frames_dict.keys():
                found_key = False
                for searching_parent_frames in self._frames_dict.keys():
                    if tmp_root_frame in self._frames_dict[searching_parent_frames]:
                        found_key = True
                        break
            
                if found_key == False:
                    roots.append(tmp_root_frame)

            root = roots[0]
            if len(roots) > 1:
                roots_str = ' ,'.join(roots).join(("[","]"))
                raise RuntimeError("The transform tree in the supplied rosbag has multiple tree roots. Automatic selection of the root cannot occur, please specify the root of the tree. Possible tree roots are: %s"%(roots_str))
        else:
            if tree_root not in self._frames_dict.keys():
                raise ValueError("The specified tree root \'%s\' does not exist in the transform tree."%(tree_root))
            root = tree_root
        
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
        

    def _check_frames_exist(self, frames : list):
        """checks to see if all frames in the passed list exist within the transform tree.

        Args:
            frames (list): a string list containing the frames to check.

        Raises:
            ValueError: if the frame does not exist within the tree.
        """
        for frame in frames:
            if not self.frame_exists(frame):
                raise ValueError('The transform frame \'%s\' does not exist in the transform tree.'%(frame))

    
    def save(self, filename):
        with open(filename, 'wb') as f:
            dill.dump(self.__dict__, f)
            f.close()

    
    def load(self, filename):
        with open(filename, 'rb') as f:
            tmp_dict = dill.load(f)
            f.close()
        
        self.__dict__.update(tmp_dict)


    def build_transform_tree(self, bag, transform_topic : str = "/tf", static_transform_topic : str ="/tf_static" , tree_root : str = None):
        """Processes the data in the supplied ROSBag, passed during object construction, and builds the
        transform tree. This function must be called prior to any other class methods. Depending on the 
        bag size, this can take a while.

        Args:
            bag (rosbag object): an opened ROSbag object containing the transform data to be processed.
            transform_topic (str, optional): used to specify the transform topic. Defaults to "/tf".
            static_transform_topic (str, optional): used to specify the transform topic. Defaults to "/tf_static".
            tree_root (str, optional): used to specify the name of the frame that is the root of the tree. Defaults to none, which will automatically try and determine the tree root.

        Raises:
            RuntimeError: if there are multiple tree roots and a tree root was not specified.
            ValueError: if the specified tree root is not present in the transform tree.
        """

        # Reset variables
        self._frames_dict = {}
        self._transforms_tree = treelib.Tree()
        self._transforms = {}
        self._all_transforms = []

        # Build transform tree
        self._build_transform_tree(bag, transform_topic, static_transform_topic, tree_root)


    def show_tree(self):
        self._transforms_tree.show()

    
    def frame_exists(self, frame : str) -> bool:
        """Checks to see if a frame exists within the transform tree.

        Args:
            frame (str): the name of the frame to check for

        Returns:
            bool: True if the frame exists
        """
        return self._transforms_tree.contains(frame)

    
    def get_common_frame(self, frame_1 : str, frame_2 : str) -> str:
        """gets the lowest common ancester frame between two frames.

        Args:
            frame_1 (str): the first transform frame
            frame_2 (str): the second transform frame

        Returns:
            str: the name of the lowest commom ancestor transform frame
        """
        
        # check to see which frame is deeper
        if self._transforms_tree.depth(frame_1) > self._transforms_tree.depth(frame_2):
            # frame 1 is further down tree
            lower_frame = frame_1
            higher_frame = frame_2
        else:
            # frame 2 is further down tree
            lower_frame = frame_2
            higher_frame = frame_1

        # go until lower frame has same depth as higher frame
        while self._transforms_tree.depth(lower_frame) != self._transforms_tree.depth(higher_frame):
            lower_frame = self._transforms_tree.parent(lower_frame).identifier

        # move both up one level until lower frame equals higher frame
        while lower_frame != higher_frame:
            lower_frame = self._transforms_tree.parent(lower_frame).identifier
            higher_frame = self._transforms_tree.parent(higher_frame).identifier

        # return
        return lower_frame

    
    def static_transform(self, source : str, dest : str) -> bool:
        """determines if the transform chain from a source to a destination frame is static.

        Args:
            source (str): the name of the source frame
            dest (str): the name of the destination frame (does not need to be directly connected to the source frame)

        Raises:
            ValueError: if the source or destination frame does not exist within the tree.

        Returns:
            bool: returns true if the transform chain is static.
        """

        # check frames exist in the tree
        self._check_frames_exist([source, dest])

        # check to see if each transform link in the chain is
        transform_chain = self.get_chain(source, dest)
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

    
    def get_chain(self, source : str, dest : str, return_type : type = tuple) -> Union[tuple, list]:
        """gets the transform chain from a source to a destination frame. The source frame does not need to
        be directly connected to the dest.

        Args:
            source (str): the name of the source (i.e., start) frame
            dest (str): the name of the dest (i.e., end) frame (does not need to be directly connected to the source)
            return_type (type, optional): used to specify the return type. Defaults to tuple.

        Raises:
            ValueError: if the source or destination frame does not exist within the tree.

        Returns:
            Union[tuple, list]: returns either a tuple or list. If the return_type is set to tuple,
                the returned value will contain a tuple of pairs holding the names of directly 
                connected source to destination frames within the chain. If the return_type is set to 
                a list, the returned value will be a list containing the names of the 
                frames in the transform chain.
        """

        # check frames exist in the tree
        self._check_frames_exist([source, dest])

        # quick check
        if source == dest:
            if return_type == list:
                return []
            elif return_type == tuple:
                return ()

        # Get lowest common ancester
        common_frame = self.get_common_frame(source, dest)

        # Create subtree with the common frame as the root, then use rsearch to get paths
        tmp_tree = self._transforms_tree.subtree(common_frame)
        source_to_common = list(tmp_tree.rsearch(source))
        common_to_dest = list(reversed(list(tmp_tree.rsearch(dest))))
        
        # Create return list
        retval_list = source_to_common + common_to_dest[1:]

        # return
        if return_type == list:
            return retval_list
        elif return_type == tuple:
            return tuple([retval_list[idx:idx+2] for idx in range(len(retval_list)-1)])

    
    def lookup_transforms(self, source : str, dest : str) -> tuple:
        """looks up and returns the transforms between a source and destination frame. 
            If the transform is a chain (i.e., the source and destination are not directly 
            connected) the transform will be updated whenever one of the transforms 
            in the chain is updated.

        Args:
            source (str): the name of the source frame
            dest (str): the name of the destination frame

        Raises:
            ValueError: if the source or destination frame does not exist within the tree.

        Returns:
            tuple: returned tuple with types (list, bool). The list contains the 
                transforms between the source and destination. The  list will have the form 
                [timestamp (float), chain_differential (float), transform (SE3 object)]
                where timestamp is the most recent timestamp in the transform chain used
                to create the entire transform. The chain_differential is the difference
                between the most recent and oldest timestamp in the transform chain used
                to create the overall transform. The transform is an SE3 object containing
                the total transform between the source and destination frame. 
                The boolean represents if the entire chain is static.
        """

        # check frames exist in the tree
        self._check_frames_exist([source, dest])

        # variables
        transforms_list = []

        # get all frames within the chain
        transform_chain = self.get_chain(source, dest)

        # check to see if entire transform chain is static
        static_transform_chain = True
        for transform_pair in transform_chain:
            if not self.static_transform(transform_pair[0], transform_pair[1]):
                static_transform_chain = False
                break

        # if the frames are directly connected, get the transform data
        # from the destination node and return the transform list
        if self._transforms_tree.parent(dest).identifier == source:
            transforms = self._transforms_tree.get_node(dest).data.transforms
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
                    tot_transform = tot_transform * x.transform # total transform of the chain 
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

    
    def lookup_transform(self, source : str, dest : str, time : float, method : str="nearest", lookup_limit : float=None, chain_limit : float=None) -> tuple:
        """Looks up and returns transform between a source and destination frame for a given time. If the entire transform chain
            is static then the method and limit arguments are ignored.

        Args:
            source (str): the name of the source frame
            dest (str): the name of the destination frame
            time (float): the time at which to look up the transform
            method (str, optional): the method to use when looking up the transform. 
                Options include "exact", "recent", "nearest", and "interpolate". Defaults to "nearest".
                exact - will attempt to get an exact transform based on the desired time.
                recent - will return a transform only if there is a past transform within the lookup_limit.
                nearest - will return a future or past transform if there is one within the lookup_limit.
                interpolate - will return an interpolated transform only if there is a transform either side
                    of the desired time within the lookup_limit.
            lookup_limit (float, optional): the limit to use when looking up a transform. If a transform 
            between the source and destination frame cannot be found within this limit a Status.LOOKUP_LIMIT_ERROR 
            will be returned. Defaults to None.
            chain_limit (float, optional): if the selected transform has a chain differential greater than 
            this value a Status.CHAIN_LIMIT_ERROR will be returned. Defaults to None.

        Raises:
            ValueError: if the source or destination frame does not exist within the tree.

        Returns:
            tuple: returns a tuple containing the [timestamp, chain_differential, transform, status].
                the timestamp will be that of the selected transform for the exact, recent and nearest methods,
                and be equal to the desired time for the interpolate method. The chain_differential will be
                that of the selected transform for the exact, recent and nearest methods, and be the maximum
                differential of the two transforms used in the interpolate method.
        """

        # check frames exist in the tree
        self._check_frames_exist([source, dest])

        # get the transform list for the specified chain
        if source != self._previous_lookup_transforms_source or dest != self._previous_lookup_transforms_dest:
            # convert the list to a numpy array
            self._transforms_list, self._static_transform_chain = self.lookup_transforms(source, dest)
            self._transforms_list = np.asarray(self._transforms_list, dtype=object)
        self._previous_lookup_transforms_source = source
        self._previous_lookup_transforms_dest = dest

        # if entire transform chain is static, transform never changes
        if self._static_transform_chain:
            # return desired time, chain differential of 0.0, transform and success
            return time, 0.0, self._transforms_list[0][2], Status.SUCCESS        

        # get time difference between the list and desired time
        time_diff = self._transforms_list[:,0] - time
        
        # outcome depends on method
        if method.lower() == "exact":
            if np.all(time_diff != 0):
                # no transform with exact timestamp
                return None, None, None, Status.NO_MATCHING_TRANSFORM  
            
            # get the timestamp, chain differential and transform
            selected_idx = np.argmin(np.abs(time_diff))
            timestamp = self._transforms_list[selected_idx][0]
            chain_dif = self._transforms_list[selected_idx][1]
            transform = self._transforms_list[selected_idx][2]   

        elif method.lower() == "recent":
            if np.all(time_diff > 0):
                # no transform in past
                return None, None, None, Status.NO_MATCHING_TRANSFORM 
            
            # get the timestamp, chain differential and transform
            selected_idx = np.argmax(time_diff[time_diff <= 0])
            timestamp = self._transforms_list[selected_idx][0]
            chain_dif = self._transforms_list[selected_idx][1]
            transform = self._transforms_list[selected_idx][2] 

        elif method.lower() == "nearest":
            # get the timestamp, chain differential and transform
            selected_idx = np.argmin(np.abs(time_diff))
            timestamp = self._transforms_list[selected_idx][0]
            chain_dif = self._transforms_list[selected_idx][1]
            transform = self._transforms_list[selected_idx][2]  

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
            point = np.interp(time, [self._transforms_list[negative_idx][0], self._transforms_list[positive_idx][0]], [0.0, 1.0])
            
            # interpolate the position
            transform = self._transforms_list[negative_idx][2].interp(self._transforms_list[positive_idx][2], point)
            if type(transform.A) != np.ndarray: # interpolation error
                return None, None, None, Status.SPATIALMATH_INTERP_ERROR

            # set timestamp, to be the desired time
            timestamp = time

            # chain differential set to maximum of the two
            chain_dif = max(self._transforms_list[negative_idx][1], self._transforms_list[positive_idx][1])
        
        else:
            return None, None, None, Status.UNKNOWN_METHOD

        # check lookup and chain differential limits
        if lookup_limit != None and np.abs(timestamp - time) > lookup_limit:
            return None, None, None, Status.LOOKUP_LIMIT_ERROR
        elif chain_limit != None and chain_dif > chain_limit:
            return None, None, None, Status.CHAIN_LIMIT_ERROR

        # return
        return timestamp, chain_dif, transform, Status.SUCCESS