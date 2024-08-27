# CMU-VLA-Challenge

## Table of Contents
[Introduction](#introduction)  
[Objective](#objective)  
[Task Specification](#task-specification)

[Setting Up](#setting-up)
- [Challenge Scenes](#challenge-scenes)
- [Challenge Questions](#challenge-questions)
- [System](#system)
- [Simulator](#simulator)
- [Object-Referential Dataset](#object-referential-dataset-vla-3d)

[Real-Robot Challenge](#real-robot-challenge-2025)
- [Real-Robot Data](#real-robot-data)

[Submission](#submission)

[Evaluation](#evaluation)

[Challenge FAQ](#challenge-faq)

## Introduction
The CMU Vision-Language-Autonomy Challenge leverages computer vision and natural language understanding in navigation autonomy. The challenge aims at pushing the limit of embodied AI in real environments and on real robots - providing a robot platform and a working autonomy system to bring everybody's work a step closer to real-world deployment. The challenge provides a real-robot system equipped with a 3D lidar and a 360 camera. The system has base autonomy onboard that can estimate the sensor pose, analyze the terrain, avoid collisions, and navigate to waypoints. Teams will set up software on the robot's onboard computer to interface with the system and navigate the robot. For 2024, the challenge will be done in a custom simulation environment and move to the real-robot system the following year. 

To register for the challenge, please see our [Challenge Website](https://www.ai-meets-autonomy.com/cmu-vla-challenge).


## Objective 
Teams are expected to come up with a vision-language model that can take a natural language navigation query and navigate the vehicle system by generating a waypoint or path based on the query.


## Task Specification
In the challenge, teams are provided with a set of natural language questions/statements for scenes from Unity [1]. The team is responsible for developing software that processes the questions together with onboard data of the scene provided by the system. The questions/statements all contain a spatial reasoning component that requires semantic spatial understanding of the objects in the scene. The environment is initially unknown and the scene data is gathered by navigating to appropriate viewpoints and exploring the scene by sending waypoints to the system. 5 questions/statements are provided for each of 15 Unity scenes and 3 scenes are held out for test evaluation.

The natural language questions are separated into three categories: numerical, object reference, and instruction following, which are further described below.

**Numerical**

Numerical questions asks about the quantity of an object that fits certain attributes or spatial relations. The response is expected to be an integer number.

Examples:

    How many blue chairs are between the table and the wall?

    How many black trash cans are near the window? 

**Object Reference**

Object reference statements asks the system to find a certain object located in the scene that is referred to by spatial relations and/or attributes. The response is expected to be a bounding box around the object and there exists only one correct answer in the scene (the referred object is unique). The center point of the bounding box marker will be used as a waypoint to navigate the robot system.

Examples:

    Find the potted plant on the kitchen island that is closest to the fridge.

    Find the orange chair between the table and sink that is closest to the window.

**Instruction-Following**

Instruction following statements ask the system to take a certain path, using objects to specify the trajectory of the path. The response is expected to be a sequence of waypoints.

Examples:

    Take the path near the window to the fridge.

    Avoid the path between the two tables and go near the blue trash can near the window.


## Setting Up
First, clone this repo and place it under your local `/home/$USER/` folder.

### Challenge Scenes
A total of 18 Unity scenes are used for the challenge. 15 scenes are provided for model development while 3 are held out for testing. The majority of these scenes are single rooms while a few are multi-room buildings.  A set of the training environment models can be downloaded from [here](https://drive.google.com/drive/folders/1bmxdT6Oxzt0_0tohye2br7gqTnkMaq20?usp=share_link). For all of the 15 training scenes, we also provide a processed point cloud of the scene, object and region information including color and size attributes, and referential language statements (please see [Object-Referential Dataset](#object-referential-dataset-vla-3d) for more details). 

![image](figures/scenes.png)

### Challenge Questions
A set of challenge questions for each Unity scene is provided in the pdf files for each of the 15 training scenes under the [questions](questions/) folder. Images of the correct answer in each scene are also provided for visualization purposes and a .ply file of the target trajectory is provided as well. All questions for all training scenes can also be found in JSON format under [questions/questions.json](questions/questions.json).

### System

Our system runs on Ubuntu 20.04 and uses ROS Noetic in both simulation and onboard the real robot. Follow the instructions in the [docker/](docker/) folder to try the simulator by pulling the docker image provided and launching the system.

The system uses Unity environments by default and has two parts:
- The base navigation system is in the [system/unity](system/unity/) folder. This sytem can be launched by itself without the AI module running. For the base navigation system, you may change the scene used by placing it in the [system/unity/src/vehicle_simulator/mesh/unity/](system/unity/src/vehicle_simulator/mesh/unity/) directory.
- The vision-language model should be in the [ai_module](ai_module/) folder. The model currently in the folder under [ai_module/src](ai_module/src) is a "dummy model" that produces arbitrary examples of the different types of output responses. **Teams are expected to come up with a model to replace this one.**

Launching the system startup script [launch.sh](launch.sh) will, be default, launch both the unity simulator and the dummy model. The dummy model will output either a number to terminal, send bounding box visualization markers for object reference, or waypoints to guide vehicle navigation. The two types of messages are listed below. To integrate the a model with the system, please modify the system startup script.
- Visualization marker: ROS Marker message on topic name: `/selected_object_marker`, containing object label and bounding box of the selected object.
- Waypoint: ROS Pose2D message on topic `/way_point_with_heading` (neglect the heading for this year’s challenge).

#### System Outputs
The system provides onboard data to the AI module as shown in the table below:

| Message | Description | Frequency | Frame | ROS Topic Name |
|-|-|-|-|-|
| Image | ROS Image message from the 360 camera. The image is at 1920/640 resolution with 360 deg HFOV and 120 VFOV. | 10Hz | camera | `/camera/image` |
| Registered Scan | ROS PointCloud2 message from the 3D lidar and registered by the state estimation module. | 5Hz | map | `/registered_scan` |
| Sensor Scan | ROS PointCloud2 message from the 3D lidar. | 5Hz | sensor_at_scan | `/sensor_scan` |
| Local Terrain Map | ROS PointCloud2 message from the terrain analysis module around the vehicle. | 5Hz | map | `/terrain_map` (5m around vehicle) <br> `/terrain_map_ext` (20m around vehicle) |
| Sensor Pose| ROS Odometry message from the state estimation module. | 100-200Hz | from map to sensor | `/state_estimation` |
| Traversable Area| ROS PointCloud2 message containing the traversable area of the entire environment. | 5Hz | map | `/traversable_area` |
| Ground-Truth Semantics| ROS MarkerArray message containing object labels and bounding boxes within 2m around the vehicle. | 5Hz | map | `/object_markers` |

**IMPORTANT NOTE**: While more topics may be available from the system, these are the only ones allowed to be used during test time. During training/development, you are free to use whatever information the system simulator provides.

#### System Inputs

The system takes waypoints output from the AI module to navigate the robot. Waypoints located in the traversable area (listed above) are accepted directly, and waypoints out of the traversable area are adjusted and moved into the traversable area. The system also takes visualization markers output by the module to highlight selected objects. The table below lists the ROS topics to use. The waypoints should be used for Numerical and Instruction-Following questions while the visualization marker should be the output for the Object Reference questions.

| Message | Description | ROS Topic Name |
|-|-|-|
| Waypoint with Heading | ROS Pose2D message with position and orientation. | `/way_point_with_heading` |
| Selected Object Marker | ROS Marker message containing object label and bounding box of the selected object. | `/selected_object_marker` |

The coordinate frames used by the physical system are shown below. The camera position (camera frame) with respect to the lidar (sensor frame) is measured based on a CAD model. The orientation is calibrated and the images are remapped to keep the camera frame and lidar frame aligned. 

<p align="center">
  <img src="figures/system.png" alt="system" width="30%"/>
</p>

### Simulator

In addition to the simulation system described above based on Unity, a second simulator is provided based on AI Habitat and uses Matterport3D environment models, which can be found under [matterport/](system/matterport/). Similarly, the system can be launched, synthesized data can be retrieved, and sending waypoints will navigate the robot in simulation. As a quick example, the command line below sends a waypoint to the system at 1m away from the start point. 

`rostopic pub -1 /way_point_with_heading geometry_msgs/Pose2D '{x: 1.0, y: 0.0, theta: 0.0}'`

Note that the simulation system provided a broader scope of data than the actual challenge but the data can be used to help prepare the AI module. During the challenge, only data listed above in [System](#system) is provided, matching the data onboard the real robot.

![image](figures/simulator.png)

### Object-Referential Dataset (VLA-3D)

To help with the subtask of referential object-grounding, the VLA-3D dataset containing 7.6K indoor 3D scenes with over 11K regions and 9M+ statements is provided. The dataset includes processed scene point clouds, object and region labels, a scene graph of semantic relations, and generated language statements for each 3D scene from a diverse set of data sources and includes the 15 training scenes in Unity. For access to the data and more details on the format, please see our [VLA-3D repository](https://github.com/HaochenZ11/VLA-3D).

## Real-Robot Challenge (2025)

Starting in 2025, the challenge evaluation will be done on the real-robot system instead of in simulation. In the challenge, the system provides onboard data as described below and takes waypoints in the same way as the simulator. The software developed in the AI module is only able to send waypoints to explore the scene. Manually sending waypoints or teleoperation is not allowed. Each team will remotely login to the robot's onboard computer (16x i9 CPU cores, 32GB RAM, RTX 4090 GPU), and set up software in a Docker container that interfaces with the autonomy modules. The Docker container is used by each team alone and not shared with other teams. We will schedule multiple time slots for each team to set up the software and test the robot. The teams can also record data on the robot's onboard computer and this data will be made available to participants afterwards.

### Real-Robot Data

Example scene data collected from the real system is provided [here](https://drive.google.com/drive/folders/1M0_UkY7aDIEpoVK6GdWzg45v-HX2UMSd?usp=drive_link) with some differences in the object layout. The dataset contains ROS messages provided by the system in the same format as during the challenge. An RVIZ configuration file is also provided for viewing the data. A ground truth map with object segmentation and IDs and an object list with bounding boxes and labels are also provided. The ground truth map and the object list are only available in the datasets but not at the challenge. The camera pose (camera frame) with respect to the lidar (sensor frame) can be found in the text file included.


## Submission
Submissions will be made as a github repository link to a public repository. The easiest way would be to fork this repository and make changes there, as the repository submitted will need to be structured in the same way. The only files/folders that should be changed are what's under [ai_module](ai_module/) and potentially the [launch.sh](launch.sh). If changes were made to the docker image to install packages, push the updated image to [Docker Hub](https://hub.docker.com/) and submit the link to the image as well.

Prior to submitting, please download the docker image and test it with the simulator as the submission will be evaluated in the same way. Please also make sure that waypoints and visualization markers sent match the types in the example dummy model and are on the same ROS topics so that the base navigation system can correctly receive them.

Please fill out the [Submission Form](https://forms.gle/KsjYNaTzSTvvPafC9) with a link to your Github repo.


## Evaluation
_\* More details on the evaluation procedure will be posted in the coming weeks \*_

The submitted code will be pulled and evaluated with 3 Unity environment models which have been held from the released data. Each scene will be unknown and the module will be given some time to explore the scene before being sent any language commands and the vehicles will be reset to some given starting position beforehand. The test scenes are of similar style to the provided training scenes. The system will be relaunched for each language command tested such that information about previous scenes are not retained. Note that the information onboard the system that is allowed to be used at test time is limited to what's listed in [System Outputs](#system-outputs).

For each scene, 5 questions similar to those provided will be tested and a score will be given to each response. The question types will be scored as follows:
- **Numerical** (/1): Exact number must be printed in the terminal. Score of 0 or 1.
- **Object Reference** (/1): ROS visualization marker must be sent that bounds the object with the center point of the marker within some X-Y radius of the ground-truth object's center point. Score of 0 or 1.
- **Instruction-Following** (/3): A series of waypoints sent that guides the vehicle. A score will be calculated as the total points gained from the trajectory minus a penalty, where the penalty is summated over *n* trajectory points. Points are gained from reaching the final destination and following path constraints specified in the command. Penalties result from deviation from reference trajectory and length of the trajectory taken. Score between 0 and 3, with possibility for partial points. 

The scores from all questions across the 3 test scenes will be totaled for each team's final score.


## Challenge FAQ
Any questions regarding the challenge can be emailed to haochen4@andrew.cmu.edu or any of the other challenge organizers. Frequently asked questions will be posted here.

1. Are multiple submissions allowed?

    There is no limit to the number of submissions allowed during the competition. The submission form is set up to allow multiple submissions and we will take your highest scoring one.

2. What are the time constraints for completing the task?

    There is a total time limit for the combined exploration and question-answering given one scene and one language statement. This will be 3 minutes for most scenes and longer for the scenes with multiple rooms, i.e. home_building_xx.

3. Any restrictions on the usage of LLMs/VLMs/APIs?

    There are no restrictions on using LLMs, VLMs, or online APIs. Any model can be used, however, keep in mind that we will need to be able to run your code and if it needs to query online APIs during runtime, you will have to provide your access token in your code.

4. What is the docker size limit?

    The size limitation depends on the machine we use to run evaluation. The specs for the machine can be found [HERE](https://simplynuc.com/product/nuc13rngi9-full/?gad_source=1&gclid=CjwKCAjwiaa2BhAiE[%E2%80%A6]g4P7AnhLOZQVIoVC9croO7-i74DfuezIOztALzi5RVJ3jv3bxoCxmEQAvD_BwE).

## References

[1] J. Haas. "A history of the unity game engine," in Diss. Worcester Polytechnic Institute, vol. 483, no. 2014, pp. 484, 2014.