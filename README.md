# CMU-VLA-Challenge

## Introduction
The CMU Vision-Language-Autonomy Challenge leverages computer vision and natural language understanding in navigation autonomy. The challenge aims at pushing the limit of embodied AI in real environments and on real robots - providing a robot platform and a working autonomy system to bring everybody's work a step closer to real-world deployment. The challenge provides a real-robot system equipped with a 3D lidar and a 360 camera. The system has base autonomy onboard that can estimate the sensor pose, analyze the terrain, avoid collisions, and navigate to waypoints. Teams will set up software on the robot's onboard computer to interface with the system and navigate the robot. For 2024, the challenge will be done in a custom simulation environment and move to the real-robot system the following year. 

To register for the challenge, please see our [Challenge Website](https://www.ai-meets-autonomy.com/cmu-vla-challenge).


## Objective 
Teams are expected to come up with a vision-language model that can take a natural language navigation query and navigate the vehicle system by generating a waypoint or path based on the query.


## Task Specification
In the challenge, teams are provided with a set of natural language questions/statements for a scene. The team is responsible for developing software that processes the questions together with onboard data of the scene provided by the system. The questions/statements all contain a spatial reasoning component that requires semantic spatial understanding of the objects in the scene. The environment is initially unknown and the scene data is gathered by navigating to appropriate viewpoints and exploring the scene by sending waypoints to the system. 5 questions/statements are provided for each of 15 Unity scenes and 3 scenes are held out for test evaluation.

The natural language questions are separated into three categories: numerical, object reference, and instruction following, which are further described below.

### Numerical
Numerical questions asks about the quantity of an object that fits certain attributes or spatial relations. The response is expected to be an integer number.

Examples:

    How many blue chairs are between the table and the wall?

    How many black trash cans are near the window? 

### Object Reference
Object reference statements asks the system to find a certain object located in the scene that is referred to by spatial relations and/or attributes. The response is expected to be a bounding box around the object and there exists only one correct answer in the scene.

Examples:

    Find the potted plant on the kitchen island that is closest to the fridge.

    Find the orange chair between the table and sink that is closest to the window.

### Instruction-Following
Instruction following statements ask the system to take a certain path, using objects to specify the trajectory of the path. The response is expected to be a sequence of waypoints.

Examples:

    Take the path near the window to the fridge.

    Avoid the path between the two tables and go near the blue trash can near the window.


## Setting Up

### Challenge Questions
A set of challenge questions for each Unity scene is provided in the pdf files [here](https://drive.google.com/drive/folders/1e7MO0GdosXGVRPV8xHcIs8vG5FEvjVkh?usp=sharing). Images of the correct answer in each scene are also provided for visualization purposes. 

### Challenge Scenes
A total of 18 Unity scenes are used for the challenge. 15 scenes are provided for model development while 3 are held out for testing. The majority of these scenes are single rooms while a few are multi-room buildings. For all the environment models, we provide the object and region information together with a point cloud of the scene (included in the Object-Referential Language Dataset). 

### System

Our system runs on Ubuntu 20.04 and uses ROS Noetic, in both simulation and onboard the real robot. Follow the instructions in the [docker/](docker/) folder to try the simulator by pulling the docker image provided and launching the system.

The system has two parts both in the home folder of the docker images:
- The base navigation system is in the [unity](system/unity/) folder. For the base navigation system, you may change the scene used by placing it in the [simulator mesh](system/unity/src/vehicle_simulator/mesh/unity/) directory. A set of 15 environment models can be downloaded from [here](https://drive.google.com/drive/folders/1bmxdT6Oxzt0_0tohye2br7gqTnkMaq20?usp=share_link).

- The vision-language model is in the `AI_module` folder. The model currently in the folder is a dummy model that produces random responses and teams are expected to come up with a model to replace this one. 

Launching the system startup script `start_cmu_vla_challenge.sh` in the home folder, the dummy model will output either a number to terminal, send bounding box visualization markers for object reference, or waypoints to guide vehicle navigation. The two types of messages are listed below. To integrate the a model with the system, please modify the system startup script.
- Visualization marker: ROS Marker message on topic name: `/selected_object_marker`, containing object label and bounding box of the selected object.
- Waypoint: ROS Pose2D message on topic `/way_point_with_heading` (neglect the heading for this year’s challenge).


The system provides onboard data to the team's software as follows.

    Image: ROS Image message from the 360 camera. The image is at 1920/640 resolution with 360 deg HFOV and 120VFOV.
                    Frequency: 10Hz, Frame: camera, ROS topic name: /camera/image

    Registered scan: ROS PointCloud2 message from the 3D lidar and registered by the state estimation module.
                    Frequency: 5Hz, Frame: map, ROS topic name: /registered_scan

    Sensor scan: ROS PointCloud2 message from the 3D lidar.
                    Frequency: 5Hz, Frame: sensor_at_scan, ROS topic name: /sensor_scan

    Local terrain map: ROS PointCloud2 message from the terrain analysis module around the vehicle.
                    Frequency: 5Hz, Frame: map, ROS topic name: /terrain_map (5m around the vehicle) /terrain_map_ext (20m around the vehicle)

    Sensor pose: ROS Odometry message from the state estimation module.
                    Frequency: 100-200Hz, Frame: from map to sensor, ROS topic name: /state_estimation.

    Traversable area: ROS PointCloud2 message containing the traversable area of the entire environment.
                    Frequency: 5Hz, Frame: map, ROS topic name: /traversable_area

    Ground truth semantics: ROS MarkerArray message containing object labels and bounding boxes within 2m around the vehicle.
                    Frequency: 5Hz, Frame: map, ROS topic name: /object_markers

The system takes waypoints from the team's software to navigate the robot. Waypoints located in the traversable area (listed above) are accepted directly, and waypoints out of the traversable area are adjusted and moved into the traversable area. The system also takes visualization markers from the team's software to highlight selected objects.

    Waypoint with heading: ROS Pose2D message with position and orientation.
    		ROS topic name: /way_point_with_heading

    Selected object marker: ROS Marker message containing object label and bounding box of the selected object.
    		ROS topic name: /selected_object_marker

The coordinate frames used by the system are shown below. The camera position (camera frame) with respect to the lidar (sensor frame) is measured based on a CAD model. The orientation is calibrated and the images are remapped to keep the camera frame and lidar frame aligned. 

## Simulation System

In addition to the simulation system described above based on Unity, a second simulator system is provided. The 2nd simulation setup is based on AI Habitat and uses Matterport3D environment models. Teams can launch the system, receive synthesized data, and send waypoints to navigate the robot in simulation. For a quick test, the command line below sends a waypoint to the system at 1m away from the start point. Note that the simulation systems provide a broader scope of data than the actual challenge. Teams can use the data to prepare the software in simulation. During the challenge, only data listed in [System](#system) is provided, matching the Real-Robot Dataset.

`rostopic pub -1 /way_point_with_heading geometry_msgs/Pose2D '{x: 1.0, y: 0.0, theta: 0.0}'`

## Object-Referential Language Dataset

To help with the subtask of referential object-grounding, a dataset containing 6.2K real-word scenes from 10K regions (together with the 15 Unity scenes for simulation) and 7.7M statements is provided. The dataset includes processed scene point clouds, object and region labels, a scene graph of semantic relations, and generated language statements. For access to the data and more details on the format, please see our [GitHub repository](https://github.com/HaochenZ11/VLA-3D).

## Real-Robot Challenge - Starting in 2025

A real-robot setup is used for the challenge. In the challenge, the system provides onboard data in the same way as the Real-Robot Dataset and takes waypoints in the same way as the Simulations. We only allow the team's software to send waypoints. Manually sending waypoints or teleoperation is not allowed. Each team will remotely login to the robot's onboard computer (16x i9 CPU cores, 32GB RAM, RTX 4090 GPU), and set up software in a Docker container that interfaces with the autonomy modules. The Docker container is used by each team alone and not shared with other teams. We will schedule multiple time slots for each team to set up the software and test the robot. The teams can also record data on the robot's onboard computer. We will make the data available to the teams afterward. 

## Real-Robot Dataset

Scene data collected from the real system is provided with some differences in the object layout. The dataset contains ROS messages provided by the system in the same format as during the challenge. An RVIZ configuration file is also provided for viewing the data. A ground truth map with object segmentation and IDs and an object list with bounding boxes and labels are also provided. The ground truth map and the object list are only available in the datasets but not at the challenge. The camera pose (camera frame) with respect to the lidar (sensor frame) is in the README.