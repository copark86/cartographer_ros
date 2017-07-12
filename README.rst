.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

============================
Cartographer ROS Integration
============================

|build| |docs| |license|

Purpose
=======

`Cartographer`_ is a system that provides real-time simultaneous localization
and mapping (`SLAM`_) in 2D and 3D across multiple platforms and sensor
configurations. This project provides Cartographer's ROS integration.

.. _Cartographer: https://github.com/googlecartographer/cartographer
.. _SLAM: https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping

Getting started
===============

* Learn to use Cartographer with ROS at `our Read the Docs site`_.
* Please join the `mailing list`_ and ask questions.

.. _our Read the Docs site: https://google-cartographer-ros.readthedocs.io
.. _mailing list: https://groups.google.com/forum/#!forum/google-cartographer

Contributing
============

You can find information about contributing to Cartographer's ROS integration
at `our Contribution page`_.

.. _our Contribution page: https://github.com/googlecartographer/cartographer_ros/blob/master/CONTRIBUTING.md

.. |build| image:: https://travis-ci.org/googlecartographer/cartographer_ros.svg?branch=master
    :alt: Build Status
    :scale: 100%
    :target: https://travis-ci.org/googlecartographer/cartographer_ros
.. |docs| image:: https://readthedocs.org/projects/google-cartographer-ros/badge/?version=latest
    :alt: Documentation Status
    :scale: 100%
    :target: https://google-cartographer-ros.readthedocs.io/en/latest/?badge=latest
.. |license| image:: https://img.shields.io/badge/License-Apache%202.0-blue.svg
     :alt: Apache 2 license.
     :scale: 100%
     :target: https://github.com/googlecartographer/cartographer_ros/blob/master/LICENSE



Cartographer ROS Integration
Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations. This project provides Cartographer’s ROS integration.

System Requirements
See Cartographer’s system requirements.

The following ROS distributions are currently supported:

Indigo
Kinetic
Building & Installation
We recommend using wstool and rosdep. For faster builds, we also recommend using Ninja.

# Install wstool and rosdep.
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build

# Create a new workspace in 'catkin_ws'.
mkdir catkin_ws
cd catkin_ws
wstool init src

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
Running the demos
Now that Cartographer and Cartographer’s ROS integration are installed, download the example bags (e.g. 2D and 3D backpack collections of the Deutsches Museum) to a known location, in this case ~/Downloads, and use roslaunch to bring up the demo:

# Download the 2D backpack example bag.
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag

# Launch the 2D backpack demo.
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag

# Pure localization demo: We use 2 different 2D bags from the Deutsche
# Museum. The first one is used to generate the map, the second to run
# pure localization.
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-05-14-44-52.bag
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-27-12-31-41.bag
# Generate the map: Run the next command, wait until cartographer_offline_node finishes.
roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag
# Run pure localization:
roslaunch cartographer_ros demo_backpack_2d_localization.launch \
   bag_filename:=${HOME}/Downloads/b2-2016-04-27-12-31-41.bag \
   map_filename:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag.pbstream

# Download the 3D backpack example bag.
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-14-14-00.bag

# Launch the 3D backpack demo.
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag

# Download the Revo LDS example bag.
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/revo_lds/cartographer_paper_revo_lds.bag

# Launch the Revo LDS demo.
roslaunch cartographer_ros demo_revo_lds.launch bag_filename:=${HOME}/Downloads/cartographer_paper_revo_lds.bag

# Download the PR2 example bag.
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/pr2/2011-09-15-08-32-46.bag

# Launch the PR2 demo.
roslaunch cartographer_ros demo_pr2.launch bag_filename:=${HOME}/Downloads/2011-09-15-08-32-46.bag

# Download the Taurob Tracker example bag.
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/taurob_tracker/taurob_tracker_simulation.bag

# Launch the Taurob Tracker demo.
roslaunch cartographer_ros demo_taurob_tracker.launch bag_filename:=${HOME}/Downloads/taurob_tracker_simulation.bag
The launch files will bring up roscore and rviz automatically.


