#!/usr/bin/env python


###############
### MODULES ###
###############


import pathlib
from tqdm import tqdm
from tabulate import tabulate 

import rosbag

from rosdata.core import ROSBagTransformer


###################
### ENTRY POINT ###
###################


def info_main(args):
    """Shows information about the ROSBag.

    Args:
        args: the command line arguments
    """
    
    # Get arguments 
    bag_path = pathlib.Path(args.rosbag)
    show_transform_tree = args.transform_tree
    show_topic_info = args.topic_info
    tree_root = args.root

    # Show all if show transform tree and topic info both not set
    if show_transform_tree == False and show_topic_info == False:
        show_transform_tree= True
        show_topic_info = True

    # Open the ROS bag file
    print("Opening ROS Bag")
    bag = rosbag.Bag(bag_path, 'r')

    # Builds and shows the transform tree
    if show_transform_tree:
        print("Processing the Transforms... Building the transform tree")
        bag_transformer = ROSBagTransformer()
        bag_transformer.build_transform_tree(bag, tree_root=tree_root)
        print("\nBuilt the following transform tree")
        bag_transformer.show_tree()

    # Report information for topics
    if show_topic_info:
        print("\nExtracting Topic Information")

        # Get topic info, contains tuple so create data structure to hold info
        # which will be added to
        topic_data = bag.get_type_and_topic_info()[1]
        data = {x: {'type': topic_data[x][0], 'count': topic_data[x][1],
            'connections': topic_data[x][2], 'frequency': topic_data[x][3],
            'frames': []} for x in topic_data}
        topic_count = 0
        for topic in topic_data:
            topic_count += data[topic]['count']

        # Get frame if present for each topic
        for topic, msg, _ in tqdm(bag.read_messages(), total=topic_count):
            # add frame id to this topic if the message has a header and not already present in the list
            # for this topic
            if hasattr(msg, 'header') and msg.header.frame_id not in data[topic]['frames']:
                data[topic]['frames'].append(msg.header.frame_id)

            # could break once all topics have gone through once, however that assumes that
            # all topics are only associated with one transform frame. That assumption should hold
            # in a correct ROS system, but what about in a poorly coded ROS system and should we
            # try and catch that error?

        # Change data type for tabulate
        data_table = [['Topic', 'Message Type', 'Message Count', 'Connections', 'Frequency', 'Transform Frames']]
        for topic in data:
            row = [topic] + list(data[topic].values())
            data_table.append(row)
        
        print("\nTopic Information")
        print(tabulate(data_table, headers='firstrow'))#, tablefmt='grid'))
        print("")


#################
### FUNCTIONS ###
#################