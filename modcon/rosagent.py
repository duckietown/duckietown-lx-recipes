import copy
import logging
import os

import numpy as np
import yaml

import rospy
from duckietown_msgs.msg import EpisodeStart, WheelEncoderStamped, WheelsCmdStamped
from duckietown_msgs.msg import LEDPattern
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import ColorRGBA
from aido_schemas import RGB


class ROSAgent:
    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.getenv("VEHICLE_NAME")
        action_topic = f"/{self.vehicle}/wheels_driver_node/wheels_cmd"
        self.ik_action_sub = rospy.Subscriber(action_topic, WheelsCmdStamped, self._ik_action_cb)
        led_topic = f"/{self.vehicle}/led_emitter_node/led_pattern"
        self.led_sub = rospy.Subscriber(led_topic, LEDPattern, self._led_cb)

        # Place holder for the action, which will be read by the agent in solution.py
        self.action = np.array([0.0, 0.0])
        self.updated = True
        self.initialized = False

        white_led = RGB(1.0, 1.0, 1.0)
        self.leds = [white_led] * 5

        # Publishes onto the corrected image topic
        # since image out of simulator is currently rectified
        # topic = "/{}/image_topic".format(self.vehicle)
        topic = f"/{self.vehicle}/camera_node/image/compressed"
        self.cam_pub = rospy.Publisher(topic, CompressedImage, queue_size=10)

        # Publisher for camera info - needed for the ground_projection
        # topic = "/{}/camera_info_topic".format(self.vehicle)
        topic = f"/{self.vehicle}/camera_node/camera_info"
        self.cam_info_pub = rospy.Publisher(topic, CameraInfo, queue_size=1)

        episode_start_topic = f"/{self.vehicle}/episode_start"
        self.episode_start_pub = rospy.Publisher(episode_start_topic, EpisodeStart, queue_size=1, latch=True)

        # copied from camera driver:

        left_encoder_topic = "/{}/left_wheel_encoder_node/tick".format(self.vehicle)
        self.left_encoder_pub = rospy.Publisher(left_encoder_topic, WheelEncoderStamped, queue_size=1)
        right_encoder_topic = "/{}/right_wheel_encoder_node/tick".format(self.vehicle)
        self.right_encoder_pub = rospy.Publisher(right_encoder_topic, WheelEncoderStamped, queue_size=1)

        # For intrinsic calibration
        self.cali_file_folder = "/data/config/calibrations/camera_intrinsic/"
        self.frame_id = rospy.get_namespace().strip("/") + "/camera_optical_frame"
        self.cali_file = self.cali_file_folder + f"{self.vehicle}.yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            rospy.logwarn(f"Calibration not found: {self.cali_file}.\n Using default instead.")
            self.cali_file = self.cali_file_folder + "default.yaml"

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file. Aborting")

        # Load the calibration file
        self.original_camera_info = self.load_camera_info(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)
        rospy.loginfo(f"Using calibration file: {self.cali_file}")

        # Initializes the node
        rospy.init_node("ROSTemplate", log_level=rospy.DEBUG, disable_rosout=False)

        log_path = "/challenges/challenge-solution-output"
        if not os.path.exists(log_path):
            os.makedirs(log_path)

        fh = logging.FileHandler(f"{log_path}/rosagent-after-init_node.log")
        fh.setLevel(logging.DEBUG)
        # create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        # create formatter and add it to the handlers
        # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        # fh.setFormatter(formatter)
        # ch.setFormatter(formatter)
        # add the handlers to the logger
        root = logging.getLogger()
        root.addHandler(fh)
        root.addHandler(ch)
        #
        rospy.loginfo("Just after init_node.")

    def _ik_action_cb(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """

        self.initialized = True
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])
        self.updated = True

    def _led_cb(self, msg):
        self.leds_initialized = True
        for i in range(5):
            led = RGB(msg.rgb_vals[i].r, msg.rgb_vals[i].g, msg.rgb_vals[i].b)
            self.leds[i] = led
        self.updated = True

    def publish_info(self, timestamp: float):
        """
        Publishes a default CameraInfo - TODO: Fix after distortion applied in simulator
        """
        # Publish the CameraInfo message
        stamp = rospy.Time.from_seconds(timestamp)
        self.current_camera_info.header.stamp = stamp
        self.cam_info_pub.publish(self.current_camera_info)

    def publish_episode_start(self, episode_name: str, payload_yaml: str):
        episode_start_message = EpisodeStart()
        episode_start_message.episode_name = episode_name
        episode_start_message.other_payload_yaml = payload_yaml
        self.episode_start_pub.publish(episode_start_message)

    def publish_img(self, obs: bytes, timestamp: float):
        """
        Publishes the image to the compressed_image topic.
        """

        # XXX: make this into a function (there were a few of these conversions around...)
        img_msg = CompressedImage()

        img_msg.header.stamp = rospy.Time.from_sec(timestamp)

        img_msg.format = "jpeg"
        img_msg.data = obs

        self.cam_pub.publish(img_msg)

    def publish_odometry(self, resolution_rad: float, left_rad: float, right_rad: float, timestamp: float):
        """
        :param timestamp:
        :param resolution_rad:
        :param left_rad:
        :param right_rad:
        :return: none
        """
        if resolution_rad == 0:
            rospy.logerr("Can't interpret encoder data with resolution 0")
        stamp = rospy.Time.from_sec(timestamp)
        msg = WheelEncoderStamped(
            data=int(np.round(left_rad / resolution_rad)),
            resolution=int(np.round(np.pi * 2 / resolution_rad)),
            type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL,
        )
        msg.header.stamp = stamp

        self.left_encoder_pub.publish(msg)

        msg = WheelEncoderStamped(
            data=int(np.round(right_rad / resolution_rad)),
            resolution=int(np.round(np.pi * 2 / resolution_rad)),
            type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL,
        )
        msg.header.stamp = stamp
        self.right_encoder_pub.publish(msg)

        print('\n THIS IS THE TIMESTEP: {}\n'.format(str(timestamp)))

    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.

        Loads the intrinsic and extrinsic camera matrices.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraInfo`: a CameraInfo message object

        """
        with open(filename, "r") as stream:
            calib_data = yaml.load(stream, Loader=yaml.Loader)
        cam_info = CameraInfo()
        cam_info.width = calib_data["image_width"]
        cam_info.height = calib_data["image_height"]
        cam_info.K = calib_data["camera_matrix"]["data"]
        cam_info.D = calib_data["distortion_coefficients"]["data"]
        cam_info.R = calib_data["rectification_matrix"]["data"]
        cam_info.P = calib_data["projection_matrix"]["data"]
        cam_info.distortion_model = calib_data["distortion_model"]
        return cam_info
