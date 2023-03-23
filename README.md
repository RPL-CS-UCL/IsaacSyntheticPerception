
# Isaac Synthetic Perception Data Generator
<img src="https://github.com/RPL-CS-UCL/IsaacSyntheticPerception/blob/main/docs/img/rpl_logo.png" width="250"> 

# Introduction
This data generator uses the photo-realistic NVIDIA Isaac Simulator to gather sensor data in procedurally created environments.

The system is built to use a SensorRig (that can be extended) that holds different types of sensors (RGB, RGB-D, LiDAR, IMU, US, and contact sensors). The Rig also captures semantic, object detection and semantic instances as ground truths with the data. This SensorRig can move through the environments through multiple methods (waypoints, velocity API). The Rig captures data based on the user's parameters. This allows the user to manually snapshot data, snapshot at points of interest, or at a set sample frequency.

The procedural generations use different layers of noise to create heightmaps, biomes, and specific object placement. These parameters can be customised to produce different and repeatable environments.

The major advantage of this system is that data can be generated for specific use cases saving space and ignoring obsolete data. Large datasets do not need to be downloaded from slow repositories. Data can be generated to have ideal class balances and can be optimised in a pipeline to generate data on the fly for poor-performing classes in ML models.

# Getting Started

## Installations

```
./python.sh -m pip install faiss-gpu, opencv-python, scikit-image, timm, fast_pytorch_keymeans, pytorch_metric_learning, kornia
```

### ToDO:

Figure out how to install `pydensecrf` in Isaac's python

## Isaac Extension

Open IsaacSim, and enable the FrankaCopycat extension.

# Using the Sensor Rig

## What is it

The SensorRig is a custom class that creates, manages, moves, and samples an array of sensors automatically and based on user parameters. For example; you can place cameras at certain positions on the sensor rig. You can then have it then follow user defined waypoints and sample at certain intervals (all within the UI).

## How to use it
There are a few key things to note...

## How to expand it.
...

## SensorRig base options

### Sensors
Custom array of sensors, their positions and orientations

### Movement system
Movement type (velocity, waypoint, human control)
Move speed
#### waypoint
load from file
initialise from scene (build when in the scene)
save waypoints
