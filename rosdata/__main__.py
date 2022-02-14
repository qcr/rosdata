#!/usr/bin/env python3

### IMPORT MODULES ###
import sys
import pathlib
import argparse
from .rosdata import *


def main():

    # Base arg parser, tools will be add to the subparser object
    p = argparse.ArgumentParser(prog="rosdata", description="A tool for extracting and manipulating data within ROSBags.")
    sp = p.add_subparsers(dest='mode')

    # Information tool arguments
    p_info = sp.add_parser('info', description='Shows information for a ROSBag.', help='Shows information for a ROSBag. Will automatically show all information. To only show a certain part, use the optional flags.')
    p_info.add_argument('rosbag', type=str, help="the ROS Bag file to show information for.")
    p_info.add_argument('-t', '--transform_tree', action='store_true', help='set to build and show the transform tree.')
    p_info.add_argument('-i', '--topic_info', action='store_true', help='set to show the topic info (number of message, message type and the transform frame(s) associated with the message, if present)')
    p_info.add_argument('-r', '--root', type=str, help='used to specify the root of the transform tree. Defaults to none which will attempt to automatically identify the tree root.', default=None)

    # Extraction Tool arguments
    p_extract = sp.add_parser('extract', description='Extract data from a ROSBag.', help='Extract data from a ROSBag.')
    p_extract.add_argument('rosbag', type=str, help="the ROS Bag file to extract data from.")
    p_extract.add_argument('extraction_config', type=str, help="the path to extraction config file.")
    p_extract.add_argument('root_output_dir', type=str, help="the absolute path to the root output directory.")

    # Visualisation tool arguments
    p_vis = sp.add_parser('visualise', help='Visualise extracted data.')
    p_vis.add_argument('csv_file', type=str, help="the path to the CSV file that wish to visualise.")
    p_vis.add_argument('-t', '--type', type=str, help='specify the type of data you wish to visualise. Options are pose, chain_diff, or lookup_diff. Default is pose', default='pose')
    p_vis.add_argument('-s', '--save', nargs='?', type=str, help="set to save the figure. A filename can be provided and if no file name is provided, it will default to the name of the CSV file with the extension \'.png\'.", default=None, const=True)
    p_vis.add_argument('-d', '--dir', nargs='?', type=str, help="used to set the save directory. Defaults to the same directory as the CSV file.", default=None)

    # Manipulation tool arguments
    # Potentially many manipulation tools all with varying arguments, set it up so with its own argument subparser object
    # each manipulation action will then have its own set of arguments
    p_manip = sp.add_parser('manipulate', description='Manipulates extracted data.', help='Manipulates extracted data.')
    sp_manip = p_manip.add_subparsers(dest='action')

    p_manip_transform = sp_manip.add_parser('rot_images', help='Rotates a set of images by a specified amount. The newly rotated images will be stored in a folder with the suffix \'_rotated\' in the same location as the original folder. If a CSV file is provided, the new CSV file will have the rotated suffix too.')
    p_manip_transform.add_argument('data', type=str, help='the path to the folder containing the images. The folder must only contain images.')
    p_manip_transform.add_argument('rot', type=float, help='the amount to rotate the images, in degrees')
    p_manip_transform.add_argument('-f', '--csv_file', type=str, nargs=4, help='the path to an associated CSV file storing the current pose data for each image, as well as the rotation (r,p,y in degrees) to apply to each transform (e.g., images.csv 0, 180, 0). This transform will need to be relative to the current child frame for the data.', default=None)
    p_manip_transform.add_argument('-o', '--overwrite', action='store_true', help='set to overwrite the existing data (i.e., no folder or CSV file with the \'_rotated\' suffix will be created).')


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
        info_optional_args = {x: args.__dict__[x] for x in args.__dict__ if x not in ['mode', 'rosbag', 'root']}
        show_info(pathlib.Path(args.rosbag), args.root, **info_optional_args)

    elif args.mode.lower() == "manipulate":
        if args.action.lower() == "rot_images":
            if args.csv_file == None:
                rotate_images(pathlib.Path(args.data), args.rot, overwrite=args.overwrite)
            else:
                csv_file = pathlib.Path(args.csv_file[0])
                rot_trans = [float(x) for x in args.csv_file[1:]]
                rotate_images(pathlib.Path(args.data), args.rot, csv_file, rot_trans, overwrite=args.overwrite)

    elif args.mode.lower() == "visualise":

        savefile = None
        if args.save == True or isinstance(args.save, str):
            save_dir = pathlib.Path(args.csv_file).parent
            if args.dir is not None:
                save_dir = pathlib.Path(args.dir)
            save_dir.mkdir(parents=True, exist_ok=True)
            
            savefile = save_dir / pathlib.Path(args.csv_file).stem
            if isinstance(args.save, str):
                savefile = save_dir / args.save

        if args.type.lower() == "pose":
            visualise_pose_data(args.csv_file, savefile)
        if args.type.lower() == "chain_diff":
            visualise_chain_differential_data(args.csv_file, savefile)
        if args.type.lower() == "lookup_diff":
            visualise_lookup_differential_data(args.csv_file, savefile)





if __name__ == '__main__':
    main()


