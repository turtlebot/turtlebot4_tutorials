#!/usr/bin/env python3

# Copyright 2024 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Hilary Luo (hluo@clearpathrobotics.com)

from enum import Enum
from math import atan2, degrees

from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as string_msg
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

from irobot_create_msgs.msg import LightringLeds
from turtlebot4_vision_tutorials.MovenetDepthaiEdge import MovenetDepthai

POSE_DETECT_ENABLE = True

SEMAPHORE_FLAG = {
        (4, 7): 'E', (4, 6): 'F', (4, 5): 'G', (2, 3): 'H',
        (3, 4): 'A', (2, 4): 'B', (1, 4): 'C', (0, 4): 'D',
        (0, 3): 'I', (0, 6): 'J', (3, 0): 'K', (3, 7): 'L',
        (3, 6): 'M', (3, 5): 'N', (2, 1): 'O', (2, 0): 'P',
        (2, 7): 'Q', (2, 6): 'R', (2, 5): 'S', (1, 0): 'T',
        (1, 7): 'U', (0, 5): 'V', (7, 6): 'W', (7, 5): 'X',
        (1, 6): 'Y', (5, 6): 'Z',
}

# Semaphore letters that indicate to drive left
LEFT_LETTERS = [
    'B',
    'C',
    'H',
    'I',
    'O',
    'S',
]

# Semaphore letters that indicate to drive right
RIGHT_LETTERS = [
    'E',
    'F',
    'M',
    'W',
    'X',
    'Z',
]

# Semaphore letters that indicate to drive forward
FORWARD_LETTERS = [
    'T',
    'U',
    '#'
]

KEYPOINT_DICT = {
    'nose': 0,
    'left_eye': 1,
    'right_eye': 2,
    'left_ear': 3,
    'right_ear': 4,
    'left_shoulder': 5,
    'right_shoulder': 6,
    'left_elbow': 7,
    'right_elbow': 8,
    'left_wrist': 9,
    'right_wrist': 10,
    'left_hip': 11,
    'right_hip': 12,
    'left_knee': 13,
    'right_knee': 14,
    'left_ankle': 15,
    'right_ankle': 16
}


class Dir(Enum):
    STOP = 0
    LEFT = 1
    STRAIGHT = 2
    RIGHT = 3


class PoseDetection(Node):
    lights_blue_ = False

    def __init__(self):
        super().__init__('pose_detection')

        self.dir = Dir.STOP
        self.dir_confirm = 0

        # # Subscribe to the /interface_buttons topic
        # self.interface_buttons_subscriber = self.create_subscription(
        #     InterfaceButtons,
        #     '/interface_buttons',
        #     self.interface_buttons_callback,
        #     qos_profile_sensor_data)

        # Create a publisher for the /cmd_lightring topic
        self.lightring_publisher = self.create_publisher(
            LightringLeds,
            'cmd_lightring',
            10)

        self.semaphore_flag_publisher = self.create_publisher(
            string_msg,
            'semaphore_flag',
            10)

        self.body_pose_publisher = self.create_publisher(
            PoseArray,
            'body_pose',
            10)

        self.camera_publisher = self.create_publisher(
            Image,
            'oakd/rgb/preview/image_raw',
            10,
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10,
        )

        self._is_paused = False

        self.start_camera_srv = self.create_service(
            Trigger,
            'start_camera',
            self.handle_start_camera
        )

        self.stop_camera_srv = self.create_service(
            Trigger,
            'stop_camera',
            self.handle_stop_camera
        )

        timer_period = 0.0833  # seconds
        self.timer = self.create_timer(timer_period, self.pose_detect)

        self.pose = MovenetDepthai(input_src='rgb',
                                   model='thunder',
                                   score_thresh=0.3,
                                   crop=not 'store_true',
                                   smart_crop=not 'store_true',
                                   internal_frame_height=432)

        self.bridge = CvBridge()

    # # Interface buttons subscription callback
    # def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
    #     # Button 1 is pressed
    #     if create3_buttons_msg.button_1.is_pressed:
    #         self.get_logger().info('Button 1 Pressed!')
    #         self.button_1_function()

    def pose_detect(self):
        if self._is_paused:
            return

        if not (POSE_DETECT_ENABLE):
            return
        # Run movenet on next frame
        frame, body, size = self.pose.next_frame()
        if frame is None:
            return
        image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'oakd_rgb_camera_optical_frame'
        image_msg.width = size[0]
        image_msg.height = size[1]
        image_msg.encoding = 'bgr8'
        self.camera_publisher.publish(image_msg)

        pose_msg = PoseArray()
        pose_msg.header.stamp = image_msg.header.stamp
        for i, point in enumerate(body.keypoints):
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(body.scores[i])
            pose_msg.poses.append(pose)

        self.body_pose_publisher.publish(pose_msg)

        # Gesture recognition
        letter = self.recognize_gesture(body)
        if letter:
            # cv2.putText(frame, letter, (frame.shape[1] // 2, 100),
            # cv2.FONT_HERSHEY_PLAIN, 5, (0,190,255), 3)
            # Create a ROS2 message
            flag_msg = string_msg()
            # Stamp the message with the current time
            flag_msg.data = letter
            self.semaphore_flag_publisher.publish(flag_msg)
            self.get_logger().info(f'Letter detected is: {letter}')

        self.autonomous_lights()

    def recognize_gesture(self, b):
        # b: body

        def angle_with_y(v):
            # v: 2d vector (x,y)
            # Returns angle in degree of v with y-axis of image plane
            if v[1] == 0:
                return 90
            angle = atan2(v[0], v[1])
            return degrees(angle)

        # For the demo, we want to recognize the flag semaphore alphabet
        # For this task, we just need to measure the angles of both arms with vertical
        if b.scores[KEYPOINT_DICT['right_elbow']] < b.score_thresh or \
           b.scores[KEYPOINT_DICT['right_shoulder']] < b.score_thresh or \
           b.scores[KEYPOINT_DICT['left_elbow']] < b.score_thresh or \
           b.scores[KEYPOINT_DICT['left_shoulder']] < b.score_thresh:
            return None
        right_arm_angle = angle_with_y(b.keypoints[KEYPOINT_DICT['right_elbow']] -
                                       b.keypoints[KEYPOINT_DICT['right_shoulder']])
        left_arm_angle = angle_with_y(b.keypoints[KEYPOINT_DICT['left_elbow']] -
                                      b.keypoints[KEYPOINT_DICT['left_shoulder']])
        right_pose = int((right_arm_angle + 202.5) / 45) % 8
        left_pose = int((left_arm_angle + 202.5) / 45) % 8
        letter = SEMAPHORE_FLAG.get((right_pose, left_pose), None)

        dir_temp = Dir.STOP
        if letter in LEFT_LETTERS:
            dir_temp = Dir.LEFT
        elif letter in RIGHT_LETTERS:
            dir_temp = Dir.RIGHT
        elif letter in FORWARD_LETTERS:
            dir_temp = Dir.STRAIGHT

        if dir_temp == self.dir:
            self.dir_confirm += 1
        else:
            self.dir_confirm = 1
            self.dir = dir_temp

        self.lights_blue_ = self.dir != Dir.STOP

        if self.dir_confirm >= 3:
            cmd_vel_msg = Twist()
            if self.dir == Dir.LEFT:
                cmd_vel_msg.angular.z = 0.4
            elif self.dir == Dir.RIGHT:
                cmd_vel_msg.angular.z = -0.4
            elif self.dir == Dir.STRAIGHT:
                cmd_vel_msg.linear.x = 0.2
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.dir_confirm = 0
        return letter

    # Perform a function when Button 1 is pressed
    def autonomous_lights(self):
        # Create a ROS2 message
        lightring_msg = LightringLeds()
        # Stamp the message with the current time
        lightring_msg.header.stamp = self.get_clock().now().to_msg()

        # Lights are currently off
        if self.lights_blue_:
            # Override system lights
            lightring_msg.override_system = True

            # LED 0
            lightring_msg.leds[0].red = 0
            lightring_msg.leds[0].blue = 255
            lightring_msg.leds[0].green = 0

            # LED 1
            lightring_msg.leds[1].red = 0
            lightring_msg.leds[1].blue = 255
            lightring_msg.leds[1].green = 0

            # LED 2
            lightring_msg.leds[2].red = 0
            lightring_msg.leds[2].blue = 255
            lightring_msg.leds[2].green = 0

            # LED 3
            lightring_msg.leds[3].red = 0
            lightring_msg.leds[3].blue = 255
            lightring_msg.leds[3].green = 0

            # LED 4
            lightring_msg.leds[4].red = 0
            lightring_msg.leds[4].blue = 255
            lightring_msg.leds[4].green = 0

            # LED 5
            lightring_msg.leds[5].red = 0
            lightring_msg.leds[5].blue = 255
            lightring_msg.leds[5].green = 0

            # Toggle the lights on status
            self.lights_blue_ = not self.lights_blue_
            # self.get_logger().info('Lights set to blue')

        # Lights are currently on
        else:
            # Disable system override. The system will take back control of the lightring.
            lightring_msg.override_system = False

        # Publish the message
        self.lightring_publisher.publish(lightring_msg)

    def handle_start_camera(self, req, resp):
        if self._is_paused:
            self._is_paused = False
            self.pose = MovenetDepthai(input_src='rgb',
                                       model='thunder',
                                       score_thresh=0.3,
                                       crop=not 'store_true',
                                       smart_crop=not 'store_true',
                                       internal_frame_height=432)
            resp.success = True
        else:
            resp.message = 'Device already running'
        return resp

    def handle_stop_camera(self, req, resp):
        if not self._is_paused:
            self._is_paused = True
            self.pose.device.close()
            self.pose = None
            resp.success = True
        else:
            resp.message = 'Device already stopped'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = PoseDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
