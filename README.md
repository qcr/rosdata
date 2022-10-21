<!-- Created with QCR's code template tool: https://github.com/qcr/code_templates -->

# ROSData: A Tool for ROSBags

[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
![Primary language](https://img.shields.io/github/languages/top/qcr/rosdata)
[![License](https://img.shields.io/github/license/qcr/rosdata)](./LICENSE.txt)
[![Powered by the Spatial Math Toolbox](https://github.com/petercorke/spatialmath-python/raw/master/.github/svg/sm_powered.min.svg)](https://github.com/petercorke/spatialmath-python)

The ROSData Python Package and Tools provide easy ways to extract and interact with ROSBags. The tools and Python utilities require limited ROS knowledge and once you have extracted the data you are completely free of ROS (depending on your project requirements of course). This freedom from ROS allows you to focus on your work rather than getting caught up in nuances and headaches that can occur while working with ROS. It also makes it great for new roboticists and developers not familiar with ROS and wanting to get a project up and running quickly.

ROSData currently has the following tools:

- Extract - extracts data from a ROSBag given a user defined extraction config file.
- Info - provides info about a ROSBag, including data not shown by the standard rosbag info tool.
- Visualise - visualise poses contained within CSV file created via the extraction tool.

It also provides Python utilities to help you utilise the extracted data within your own Python projects. See the [ROSData Documentation](https://open.qcr.ai/rosdata/) for installation instructions, getting started information, and examples.


## Acknowledgments

If you use the ROS Data tool in your work we would appreciate acknowledgement through a link back to this repository.
