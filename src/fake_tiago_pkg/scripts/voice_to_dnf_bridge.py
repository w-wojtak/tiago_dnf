#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node that converts voice commands into DNF-compatible input vectors.
Publishes /voice_input_vector as Float32MultiArray.
Subscribes to /voice_message (std_msgs/String), which can be published manually for testing.
"""

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, String

class VoiceToDNFBridge:
    def __init__(self):
        rospy.init_node('voice_to_dnf_bridge', anonymous=True)

        # DNF spatial grid
        self.x_lim, self.dx = 80, 0.2
        self.x = np.arange(-self.x_lim, self.x_lim + self.dx, self.dx)
        self.vector_length = len(self.x)

        # Mapping voice commands → DNF spatial center
        self.command_positions = {
            "give_motor": 40,
            "give_load": -20,
            "give_bearing": 20,
            "give_base": -60
        }

        # Track commands that were already triggered (once per detection)
        self.triggered_commands = set()

        # Gaussian parameters
        self.amplitude = 5.0
        self.width = 2.0

        # Publisher for DNF-compatible vector
        self.pub = rospy.Publisher('/voice_input_vector', Float32MultiArray, queue_size=10)

        # Subscribe to incoming voice commands
        rospy.Subscriber('/voice_message', String, self.voice_callback)

        rospy.loginfo("Voice→DNF bridge initialized. Listening on /voice_message...")

    def gaussian(self, center, amplitude=None, width=None):
        """Return Gaussian profile over self.x"""
        a = amplitude if amplitude is not None else self.amplitude
        w = width if width is not None else self.width
        return a * np.exp(-((self.x - center) ** 2) / (2 * w ** 2))

    def voice_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd not in self.command_positions:
            rospy.logwarn(f"Unknown voice command '{cmd}' - ignoring")
            return

        # Check if already triggered
        if cmd in self.triggered_commands:
            rospy.loginfo(f"Command '{cmd}' already triggered - skipping")
            return

        # Mark as triggered
        self.triggered_commands.add(cmd)

        # Create Gaussian input
        center = self.command_positions[cmd]
        gaussian_vector = self.gaussian(center=center)

        # Publish
        out_msg = Float32MultiArray()
        out_msg.data = gaussian_vector.tolist()
        self.pub.publish(out_msg)

        rospy.loginfo(f"Published voice input for '{cmd}' at center={center}")


if __name__ == "__main__":
    try:
        node = VoiceToDNFBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
