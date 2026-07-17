#!/usr/bin/env python3
"""Adapts the Raspberry Pi ROS 1 protocol to the Unity GFS-X protocol.

The robot's unity_master_team1.py already consumes /cmd_vel, /cmd_gripper and
/cmd_camera_pan directly.  It publishes physical sensors in one Quaternion on
/sensor/data: x=ultrasonic metres, y=left IR, z=right IR, w=gripper IR.
"""

import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32, Int32


class GfsxUnityAdapter:
    def __init__(self):
        self._ultrasonic_scale = float(rospy.get_param("~ultrasonic_scale_to_meters", 1.0))
        self._ball_distance_scale = float(rospy.get_param("~ball_distance_scale_to_meters", 1.0))
        self._invert_left_ir = bool(rospy.get_param("~invert_left_ir", False))
        self._invert_right_ir = bool(rospy.get_param("~invert_right_ir", False))
        self._invert_gripper_ir = bool(rospy.get_param("~invert_gripper_ir", False))

        self._unity_ultrasonic = rospy.Publisher("/gfsx/ultrasonic", Float32, queue_size=1)
        self._unity_left_ir = rospy.Publisher("/gfsx/left_ir", Int32, queue_size=1)
        self._unity_right_ir = rospy.Publisher("/gfsx/right_ir", Int32, queue_size=1)
        self._unity_gripper_ir = rospy.Publisher("/gfsx/gripper_ir", Int32, queue_size=1)
        self._unity_ball_visible = rospy.Publisher("/gfsx/ball_visible", Int32, queue_size=1)
        self._unity_ball_horizontal = rospy.Publisher("/gfsx/ball_horizontal", Float32, queue_size=1)
        self._unity_ball_distance = rospy.Publisher("/gfsx/ball_distance", Float32, queue_size=1)

        self._ball_visible = 0
        self._ball_horizontal = 0.0
        self._ball_distance = float(rospy.get_param("~default_ball_distance_meters", 2.0))

        rospy.Subscriber(rospy.get_param("~real_sensor_packet_topic", "/sensor/data"), Quaternion,
                         self._receive_sensor_packet, queue_size=1)
        rospy.Subscriber(rospy.get_param("~source_ball_visible_topic", "/yolo/ball_visible"),
                         Int32, self._receive_ball_visible, queue_size=1)
        rospy.Subscriber(rospy.get_param("~source_ball_horizontal_topic", "/yolo/ball_horizontal"),
                         Float32, self._receive_ball_horizontal, queue_size=1)
        rospy.Subscriber(rospy.get_param("~source_ball_distance_topic", "/yolo/ball_distance"),
                         Float32, self._receive_ball_distance, queue_size=1)
        rospy.loginfo("GFS-X adapter started: /sensor/data -> /gfsx/*")

    def _receive_sensor_packet(self, message):
        self._unity_ultrasonic.publish(
            Float32(max(0.0, message.x * self._ultrasonic_scale)))
        self._relay_value(message.y, self._unity_left_ir, self._invert_left_ir)
        self._relay_value(message.z, self._unity_right_ir, self._invert_right_ir)
        self._relay_value(message.w, self._unity_gripper_ir, self._invert_gripper_ir)
        self._unity_ball_visible.publish(Int32(self._ball_visible))
        self._unity_ball_horizontal.publish(Float32(self._ball_horizontal))
        self._unity_ball_distance.publish(Float32(self._ball_distance))

    @staticmethod
    def _relay_value(value, publisher, invert):
        active = value != 0
        publisher.publish(Int32(1 if active != invert else 0))

    def _receive_ball_visible(self, message):
        self._ball_visible = 1 if message.data != 0 else 0

    def _receive_ball_horizontal(self, message):
        self._ball_horizontal = max(-1.0, min(1.0, message.data))

    def _receive_ball_distance(self, message):
        self._ball_distance = max(0.0, message.data * self._ball_distance_scale)


if __name__ == "__main__":
    rospy.init_node("gfsx_unity_adapter")
    GfsxUnityAdapter()
    rospy.spin()
