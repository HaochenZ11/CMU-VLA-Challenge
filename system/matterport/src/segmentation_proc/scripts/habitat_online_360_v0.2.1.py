#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image as ROS_Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import argparse

import math
import multiprocessing
import os
import random
import time
from enum import Enum

import numpy as np
from PIL import Image
import cv2

from six_images_py_numba import SixPlanarNumba
from camera_models import ShapeStruct, Equirectangular

import habitat_sim
import habitat_sim.agent
from habitat_sim.utils.common import (
    d3_40_colors_rgb,
    quat_from_coeffs,
)

from scipy.spatial.transform import Rotation as R

default_sim_settings = {
    "frame_rate": 10, # image frame rate
    "width": 480, # horizontal resolution
    "height": 480, # vertical resolution
    "hfov": 90, # horizontal FOV
    "camera_offset_z": 0.5, # camera z-offset
    "class_semantic": False, # Use class id as semantic color (False: use instance id)
    "color_sensor": True,  # RGB sensor
    "depth_sensor": False,  # depth sensor
    "semantic_sensor": True,  # semantic sensor
    "scene": "../../vehicle_simulator/mesh/matterport/segmentations/matterport.glb",
}

image_width_360 = 1920
image_height_360 = 640
image_half_width_360 = int(image_width_360 / 2)
image_margin_360 = int((image_half_width_360 - image_height_360) / 2)

camera_model_out = Equirectangular(
    shape_struct= ShapeStruct(image_half_width_360, image_width_360),
)
sampler = SixPlanarNumba(camera_model_out)

parser = argparse.ArgumentParser()
parser.add_argument("--scene", type=str, default=default_sim_settings["scene"])
args = parser.parse_args()

def make_settings():
    settings = default_sim_settings.copy()
    settings["scene"] = args.scene

    return settings

settings = make_settings()

def make_cfg(settings):
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.frustum_culling = True
    sim_cfg.gpu_device_id = 0
    if not hasattr(sim_cfg, "scene_id"):
        raise RuntimeError(
            "Error: Please upgrade habitat-sim. SimulatorConfig API version mismatch"
        )
    sim_cfg.scene_id = settings["scene"]

    sensor_specs = []
    def create_camera_spec(**kw_args):
        camera_sensor_spec = habitat_sim.CameraSensorSpec()
        camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
        camera_sensor_spec.resolution = [settings["height"], settings["width"]]
        camera_sensor_spec.position = [0, 0, 0]
        for k in kw_args:
            setattr(camera_sensor_spec, k, kw_args[k])
        return camera_sensor_spec

    if settings["color_sensor"]:
    	for i in range(6):
            color_sensor_spec = create_camera_spec(
                uuid="color_sensor_"+str(i),
                hfov=settings["hfov"],
                sensor_type=habitat_sim.SensorType.COLOR,
                sensor_subtype=habitat_sim.SensorSubType.PINHOLE,
            )
            sensor_specs.append(color_sensor_spec)

    if settings["depth_sensor"]:
    	for i in range(6):
            depth_sensor_spec = create_camera_spec(
                uuid="depth_sensor_"+str(i),
                hfov=settings["hfov"],
                sensor_type=habitat_sim.SensorType.DEPTH,
                channels=1,
                sensor_subtype=habitat_sim.SensorSubType.PINHOLE,
            )
            sensor_specs.append(depth_sensor_spec)

    if settings["semantic_sensor"]:
        for i in range(6):
            semantic_sensor_spec = create_camera_spec(
                uuid="semantic_sensor_"+str(i),
                hfov=settings["hfov"],
                sensor_type=habitat_sim.SensorType.SEMANTIC,
                channels=1,
                sensor_subtype=habitat_sim.SensorSubType.PINHOLE,
            )
            sensor_specs.append(semantic_sensor_spec)

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])

class DemoRunnerType(Enum):
    BENCHMARK = 1
    EXAMPLE = 2
    AB_TEST = 3

class ABTestGroup(Enum):
    CONTROL = 1
    TEST = 2

class DemoRunner:
    def __init__(self, sim_settings, simulator_demo_type):
        if simulator_demo_type == DemoRunnerType.EXAMPLE:
            self.set_sim_settings(sim_settings)
        self._demo_type = simulator_demo_type

    def set_sim_settings(self, sim_settings):
        self._sim_settings = sim_settings.copy()

    def publish_color_observation(self, obs, t):
        color_img_front = obs["color_sensor_0"][:, :, :3]
        color_img_left = obs["color_sensor_1"][:, :, :3]
        color_img_back = obs["color_sensor_2"][:, :, :3]
        color_img_right = obs["color_sensor_3"][:, :, :3]
        color_img_top = obs["color_sensor_4"][:, :, :3]
        color_img_bottom = obs["color_sensor_5"][:, :, :3]
        image_dict_color = {}
        image_dict_color["front"] = color_img_front
        image_dict_color["back"] = color_img_back
        image_dict_color["left"] = color_img_left
        image_dict_color["right"] = color_img_right
        image_dict_color["top"] = color_img_top
        image_dict_color["bottom"] = color_img_bottom

        Fout_color, Fout_mask = sampler(image_dict_color, interpolation = 'linear', invalid_pixel_value = 127)
        image_360_color = Fout_color
        
        self.color_image.data = np.array(image_360_color[image_margin_360:-image_margin_360, :, :]).tobytes()
        self.color_image.header.stamp = rospy.Time.from_sec(t)
        self.color_image_pub.publish(self.color_image)

    def publish_semantic_observation(self, obs, t):
        semantic_img_front = obs["semantic_sensor_0"]
        semantic_img_left = obs["semantic_sensor_1"]
        semantic_img_back = obs["semantic_sensor_2"]
        semantic_img_right = obs["semantic_sensor_3"]
        semantic_img_top = obs["semantic_sensor_4"]
        semantic_img_bottom = obs["semantic_sensor_5"]
        image_dict_semantic = {}
        image_dict_semantic["front"] = semantic_img_front
        image_dict_semantic["back"] = semantic_img_back
        image_dict_semantic["left"] = semantic_img_left
        image_dict_semantic["right"] = semantic_img_right
        image_dict_semantic["top"] = semantic_img_top
        image_dict_semantic["bottom"] = semantic_img_bottom
        for key in image_dict_semantic.keys():
            if self._sim_settings["class_semantic"]:
                image_dict_semantic[key] = self.instance_id_to_label_id_func(image_dict_semantic[key])
            semantic_img_colored = Image.new("P", (image_dict_semantic[key].shape[1], image_dict_semantic[key].shape[0]))
            semantic_img_colored.putpalette(d3_40_colors_rgb.flatten())
            semantic_img_colored.putdata((image_dict_semantic[key].flatten() % 40).astype(np.uint8))
            semantic_img_colored = np.array(semantic_img_colored.convert("RGB"))
            image_dict_semantic[key] = semantic_img_colored

        Fout_semantic, Fout_mask = sampler(image_dict_semantic, interpolation = 'linear', invalid_pixel_value = 127)
        image_360_semantic = Fout_semantic[image_margin_360:-image_margin_360, :, :]

        self.semantic_image.data = np.array(image_360_semantic).tobytes()
        self.semantic_image.header.stamp = rospy.Time.from_sec(t)
        self.semantic_image_pub.publish(self.semantic_image)

    def publish_depth_observation(self, obs, t):
        depth_img_front = obs["depth_sensor_0"]
        depth_img_left = obs["depth_sensor_1"]
        depth_img_back = obs["depth_sensor_2"]
        depth_img_right = obs["depth_sensor_3"]
        depth_img_top = obs["depth_sensor_4"]
        depth_img_bottom = obs["depth_sensor_5"]
        image_dict_depth = {}
        image_dict_depth["front"] = depth_img_front
        image_dict_depth["back"] = depth_img_back
        image_dict_depth["left"] = depth_img_left
        image_dict_depth["right"] = depth_img_right
        image_dict_depth["top"] = depth_img_top
        image_dict_depth["bottom"] = depth_img_bottom

        Fout_depth, Fout_mask = sampler(image_dict_depth, interpolation = 'linear', invalid_pixel_value = -1)
        image_360_depth = Image.fromarray((Fout_depth[image_margin_360:-image_margin_360, :] / 10 * 255).astype(np.uint8), mode="L")

        self.depth_image.data = np.array(image_360_depth.convert("L")).tobytes()
        self.depth_image.header.stamp = rospy.Time.from_sec(t)
        self.depth_image_pub.publish(self.depth_image)

    def init_common(self):
        self._cfg = make_cfg(self._sim_settings)
        scene_file = self._sim_settings["scene"]

        self._sim = habitat_sim.Simulator(self._cfg)

        if self._sim_settings["class_semantic"]:
            self.instance_id_to_label_id = {int(obj.id.split("_")[-1]): obj.category.index() for obj in self._sim.semantic_scene.objects}
            self.instance_id_to_label_id_func=np.frompyfunc(lambda x:self.instance_id_to_label_id[x], 1, 1)

        if not self._sim.pathfinder.is_loaded:
            navmesh_settings = habitat_sim.NavMeshSettings()
            navmesh_settings.set_defaults()
            self._sim.recompute_navmesh(self._sim.pathfinder, navmesh_settings)

    def state_estimation_callback(self, msg):
        self.time = msg.header.stamp.to_sec()
        orientation = msg.pose.pose.orientation
        (self.camera_roll, self.camera_pitch, self.camera_yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.camera_x = msg.pose.pose.position.x
        self.camera_y = msg.pose.pose.position.y
        self.camera_z = msg.pose.pose.position.z

    def listener(self):
        start_state = self.init_common()

        rospy.init_node("habitat_online")
        
        rospy.Subscriber("/state_estimation", Odometry, self.state_estimation_callback)
        self.time = 0
        self.camera_roll = 0
        self.camera_pitch = 0
        self.camera_yaw = 0
        self.camera_x = 0
        self.camera_y = 0
        self.camera_z = 0.5

        if self._sim_settings["color_sensor"]:
            self.color_image_pub = rospy.Publisher("/camera/image", ROS_Image, queue_size=2)
            self.color_image = ROS_Image()
            self.color_image.header.frame_id = "camera"
            self.color_image.height = image_height_360
            self.color_image.width  = image_width_360
            self.color_image.encoding = "rgb8"
            self.color_image.step = 3 * image_width_360
            self.color_image.is_bigendian = False

        if self._sim_settings["depth_sensor"]:
            self.depth_image_pub = rospy.Publisher("/camera/depth_image", ROS_Image, queue_size=2)
            self.depth_image = ROS_Image()
            self.depth_image.header.frame_id = "camera"
            self.depth_image.height = image_height_360
            self.depth_image.width  = image_width_360
            self.depth_image.encoding = "mono8"
            self.depth_image.step = image_width_360
            self.depth_image.is_bigendian = False

        if self._sim_settings["semantic_sensor"]:
            self.semantic_image_pub = rospy.Publisher("/camera/semantic_image", ROS_Image, queue_size=2)
            self.semantic_image = ROS_Image()
            self.semantic_image.header.frame_id = "camera"
            self.semantic_image.height = image_height_360
            self.semantic_image.width  = image_width_360
            self.semantic_image.encoding = "rgb8"
            self.semantic_image.step = 3 * image_width_360
            self.semantic_image.is_bigendian = False

        r = rospy.Rate(default_sim_settings["frame_rate"])
        while not rospy.is_shutdown():
            roll = -self.camera_roll
            pitch = self.camera_pitch
            yaw = 1.5708 - self.camera_yaw
            rolls = [roll, roll, roll, roll, roll, roll]
            pitchs = [pitch, pitch, pitch, pitch, pitch-1.5708, pitch+1.5708]
            yaws = [yaw, yaw-1.5708, yaw-2*1.5708, yaw+1.5708, yaw, yaw]

            qxs = []
            qys = []
            qzs = []
            qws = []
            for i in range(len(rolls)):
                qxs.append(np.sin(rolls[i]/2) * np.cos(pitchs[i]/2) * np.cos(yaws[i]/2) - np.cos(rolls[i]/2) * np.sin(pitchs[i]/2) * np.sin(yaws[i]/2))
                qys.append(np.cos(rolls[i]/2) * np.sin(pitchs[i]/2) * np.cos(yaws[i]/2) + np.sin(rolls[i]/2) * np.cos(pitchs[i]/2) * np.sin(yaws[i]/2))
                qzs.append(np.cos(rolls[i]/2) * np.cos(pitchs[i]/2) * np.sin(yaws[i]/2) - np.sin(rolls[i]/2) * np.sin(pitchs[i]/2) * np.cos(yaws[i]/2))
                qws.append(np.cos(rolls[i]/2) * np.cos(pitchs[i]/2) * np.cos(yaws[i]/2) + np.sin(rolls[i]/2) * np.sin(pitchs[i]/2) * np.sin(yaws[i]/2))

            position = np.array([self.camera_x, self.camera_y, self.camera_z])
            position[1], position[2] = position[2], -position[1]
            
            agent_state = self._sim.get_agent(0).get_state()
            state_time = self.time

            i = 0
            j = 0
            k = 0
            for sensor in agent_state.sensor_states:
                agent_state.sensor_states[sensor].position = position + np.array([0, default_sim_settings["camera_offset_z"], 0])
                if sensor in ["color_sensor_0", "color_sensor_1", "color_sensor_2", "color_sensor_3", "color_sensor_4", "color_sensor_5"]:
                    agent_state.sensor_states[sensor].rotation = quat_from_coeffs(np.array([-qys[i], -qzs[i], qxs[i], qws[i]]))
                    i += 1
                elif sensor in ["depth_sensor_0", "depth_sensor_1", "depth_sensor_2", "depth_sensor_3", "depth_sensor_4", "depth_sensor_5"]:
                    agent_state.sensor_states[sensor].rotation = quat_from_coeffs(np.array([-qys[j], -qzs[j], qxs[j], qws[j]]))
                    j += 1
                else:
                    agent_state.sensor_states[sensor].rotation = quat_from_coeffs(np.array([-qys[k], -qzs[k], qxs[k], qws[k]]))
                    k += 1

            self._sim.get_agent(0).set_state(agent_state, infer_sensor_states = False)
            observations = self._sim.step("move_forward")

            if self._sim_settings["color_sensor"]:
                self.publish_color_observation(observations, state_time)
            if self._sim_settings["depth_sensor"]:
                self.publish_depth_observation(observations, state_time)
            if self._sim_settings["semantic_sensor"]:
                self.publish_semantic_observation(observations, state_time)

            state = self._sim.last_state()
            print("Publishing at time: " + str(state_time))
            r.sleep()

        self._sim.close()
        del self._sim

demo_runner = DemoRunner(settings, DemoRunnerType.EXAMPLE)
demo_runner.listener()
