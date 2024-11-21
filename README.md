## Hydra

<div align="center">
    <img src="doc/media/hydra.GIF">
</div>

This repository contains code to incrementally build 3D Scene Graphs in real-time and is based on the papers:
  - ["Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization"](http://www.roboticsproceedings.org/rss18/p050.pdf)
  - ["Foundations of Spatial Perception for Robotics: Hierarchical Representations and Real-time Systems"](https://journals.sagepub.com/doi/10.1177/02783649241229725)

> **Note**<br>
> ROS related code has been moved to a new repository located [here](https://github.com/MIT-SPARK/Hydra-ROS). This code still do rely on the ROS ecosystem.

## Installation and Running

### General Requirements

Hydra has been tested on Ubuntu 20.04 and ROS Noetic.

Then, make sure you have some general requirements:
```
sudo apt install python3-rosdep python3-catkin-tools python3-vcstool
```

Finally, if you haven't set up rosdep yet:
```
sudo rosdep init
rosdep update
```

### Building Hydra

To get started:

```
mkdir -p hydra_ws/src
cd hydra_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
cd src
git clone git@github.com:ArghyaChatterjee/Hydra.git hydra
vcs import . < hydra/install/hydra.rosinstall
rosdep install --from-paths . --ignore-src -r -y
cd ..
catkin build
```

> **Note**<br>
> Depending on the amount of RAM available on your machine and whether or not you are compiling Kimera-VIO as well, you may run out of memory when compiling with `catkin build` directly (which will result in a `GCC killed` error). If this occurs, you can either specify fewer threads for catkin via `catkin build -j NUM_THREADS` or compile certain larger packages directly first by building them specifically.

:warning: In the `vcs import` step, GitHub may block too many concurrent requests. If you receive `kex_exchange_identification: read: Connection reset by peer` errors, try running `vcs import . < hydra/install/hydra.rosinstall --workers 1`.

Please help us by creating new issues when you run into problems with these instructions!

### Quickstart

To test Hydra out, you can just download a single scene (the office scene without humans is recommended, and can be found [here](https://drive.google.com/uc?id=1CA_1Awu-bewJKpDrILzWok_H_6cOkGDb).
Make sure to decompress the rosbag (`rosbag decompress path/to/bagfile`) before running!

> **:warning: Warning**<br>
> Also make sure to source the workspace before starting.<br>
> This is typically `source path/to/catkin_ws/devel/setup.bash`, though if you use zsh you should use the correct setup file for that.

To start Hydra:
```
roslaunch hydra_ros uhumans2.launch
```

Then, start the rosbag in a separate terminal:
```
rosbag play path/to/rosbag --clock
```

### Running Hydra

See [here](https://github.com/MIT-SPARK/Hydra-ROS/blob/main/doc/quickstart.md) for detailed instructions discussing how to run Hydra using ROS.
These also detail how to use Hydra with [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO.git), including how to build Kimera-VIO alongside Hydra.

### Hydra Python Bindings

See [here](python/README.md) for information

### Hydra Evaluation

See [here](eval/README.md) for information

### Using a Semantic Segmentation Network

> **Note**<br>
> This package is not public (yet)

Add `semantic_recolor` to your workspace via:

```
roscd && cd ../src
vcs import . < hydra/install/semantic_overlay.rosinstall
```

Then, follow the instructions to install cuda and other dependencies for the `semantic_recolor` package (which can be found [here](https://github.mit.edu/SPARK/semantic_recolor_nodelet#semantic-recolor-utilities)).

Finally, build your workspace:

```
catkin build
```
