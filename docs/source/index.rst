.. ROSData documentation master file, created by
   sphinx-quickstart on Fri Oct  7 21:39:59 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to ROSData's Documentation!
===================================

The ROSData Python Package and Tools provide easy ways to extract and interact with ROSBags. The tools and Python utilities require limited ROS knowledge and once you have extracted the data you are completely free of ROS (depending on your project requirements of course). This freedom from ROS allows you to focus on your work rather than getting caught up in nuances and headaches that can occur while working with ROS. It also makes it great for new roboticists and developers not familiar with ROS and wanting to get a project up and running quickly.

ROSData currently has the following tools:

- **extract** - extracts data from a ROSBag given a user defined extraction config file.
- **info** - provides info about a ROSBag, including data not shown by the standard `rosbag info` tool.
- **visualise** - visualise poses contained within CSV file created via the extraction tool.

It also provides Python utilities to help you utilise the extracted data within your own Python projects.


.. toctree::
   :hidden:
   :caption: Getting Started:

   getting_started/installation
   getting_started/cl_tools
   getting_started/py_module

.. toctree::
   :hidden:
   :caption: Tools:

   tools/extraction
   tools/information
   tools/visualization
   tools/manipulation

.. toctree::
   :hidden:
   :caption: Python Module Reference:
   
   reference/index

.. .. toctree::
..    :hidden:
..    :caption: Navigation

..    genindex
