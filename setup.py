# Created with QCR's code template tool: https://github.com/qcr/code_templates

from setuptools import find_packages, setup

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(name='rosdata',
      version='0.1.1',
      author='James Mount',
      author_email='j.mount@qut.edu.au',
      url='https://github.com/qcr/rosdata',
      description='Provides tools to extract, manipulate and visualise ROSBag Data.',
      license_files=['LICENSE.txt'],
      long_description=long_description,
      long_description_content_type='text/markdown',
      packages=find_packages(),
      install_requires=['numpy'],#, 'yaml', 'cv2', 'tqdm', 'treelib', 'spatialmaths', 'open3d', 'open3d_ros_helper', 'cv_bridge', 'rospy'],
      entry_points={'console_scripts': ['rosdata=rosdata.__main__:main']},
      classifiers=(
          "Development Status :: 4 - Beta",
          "Programming Language :: Python :: 3",
          "License :: OSI Approved :: BSD License",
          "Operating System :: OS Independent",
      ))
