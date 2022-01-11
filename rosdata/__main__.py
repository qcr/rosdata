#!/usr/bin/env python3

### IMPORT MODULES ###
import sys
import pathlib
import argparse
from .rosdata import extract_rosbag_data, visualise_data


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

    p_visualise = sp.add_parser('visualise', help='Visualise extracted data.')
    p_visualise.add_argument('csv_file', type=str, help="the path to the CSV file that wish to visualise.")


    args = p.parse_args()

    # print help if no args provided
    if len(sys.argv) == 1:
        p.print_help()
        return

    # run requested operation
    if args.mode.lower() == "extract":
        extract_rosbag_data(pathlib.Path(args.rosbag), pathlib.Path(args.extraction_config), pathlib.Path(args.root_output_dir))
    
    elif args.mode.lower() == "visualise":
        visualise_data(args.csv_file)


if __name__ == '__main__':
    main()


