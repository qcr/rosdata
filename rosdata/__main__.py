#!/usr/bin/env python3

### IMPORT MODULES ###
import sys
import pathlib
import argparse
from .rosdata import *


def main():

    p = argparse.ArgumentParser(
        prog="rosdata",
        description="A tool for extracting and manipulating data within ROSBags."
    )

    sp = p.add_subparsers(dest='mode')

    p_extract = sp.add_parser('extract', description='Extract data from a ROSBag.', help='Extract data from a ROSBag.')
    p_extract.add_argument('rosbag', type=str, help="the ROS Bag file to extract data from.")
    p_extract.add_argument('extraction_config', type=str, help="the path to extraction config file.")
    p_extract.add_argument('root_output_dir', type=str, help="the absolute path to the root output directory.")

    p_info = sp.add_parser('info', description='Shows information for a ROSBag.', help='Shows information for a ROSBag. Will automatically show all information. To only show a certain part, use the optional flags.')
    p_info.add_argument('rosbag', type=str, help="the ROS Bag file to show information for.")
    p_info.add_argument('-t', '--transform_tree', action='store_true', help='set to build and show the transform tree.')
    p_info.add_argument('-i', '--topic_info', action='store_true', help='set to show the topic info (number of message, message type and the transform frame(s) associated with the message, if present)')

    p_visualise = sp.add_parser('visualise', help='Visualise extracted data.')
    p_visualise.add_argument('csv_file', type=str, help="the path to the CSV file that wish to visualise.")
    p_visualise.add_argument('-t', '--type', type=str, help='specify the type of data you wish to visualise. Options are pose, chain_diff, or lookup_diff. Default is pose', default='pose')
    p_visualise.add_argument('-s', '--save', nargs='?', type=str, help="set to save the figure. A filename can be provided and if no file name is provided, it will default to the name of the CSV file with the extension \'.png\'.", default=None)
    p_visualise.add_argument('-d', '--dir', nargs='?', type=str, help="used to set the save directory. Defaults to the same directory as the CSV file.", default=None)


    args = p.parse_args()

    # print help if no args provided
    if len(sys.argv) == 1:
        p.print_help()
        return

    # run requested operation
    if args.mode.lower() == "extract":
        extract_rosbag_data(pathlib.Path(args.rosbag), pathlib.Path(args.extraction_config), pathlib.Path(args.root_output_dir))
    
    elif args.mode.lower() == "info":
        # get only the optional arguments for info mode as a dictionary
        info_optional_args = {x: args.__dict__[x] for x in args.__dict__ if x not in ['mode', 'rosbag']}
        show_info(pathlib.Path(args.rosbag), **info_optional_args)

    elif args.mode.lower() == "visualise":
        save_dir = pathlib.Path(args.csv_file).parent
        if args.dir is not None:
            save_dir = pathlib.Path(args.dir)
        save_filename = pathlib.Path(args.csv_file).stem
        if args.save is not None:
            save_filename = args.save

        if args.type.lower() == "pose":
            visualise_pose_data(args.csv_file, save_dir/save_filename)
        if args.type.lower() == "chain_diff":
            visualise_chain_differential_data(args.csv_file, save_dir/save_filename)
        if args.type.lower() == "lookup_diff":
            visualise_lookup_differential_data(args.csv_file, save_dir/save_filename)


if __name__ == '__main__':
    main()


