#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node that simulates a human giving voice commands at scheduled times.
Publishes the name of the requested object as a std_msgs/String.
"""

import rospy
from std_msgs.msg import String

class FakeVoiceCommander:
    def __init__(self):
        rospy.init_node('fake_voice_commander', anonymous=True)

        # This schedule simulates WHEN the human asks for each object.
        # These are wall-clock times in seconds from the node's start.
        self.voice_command_schedule = [
            ('base', 17.0),      # Human asks for 'base' at t=17s
            ('load', 27.0),     # Human asks for 'load' 10s later
            ('bearing', 34.0),  # ...and so on
            ('motor', 41.0)
        ]

        # This topic will carry the voice command.
        self.pub = rospy.Publisher('/voice_command', String, queue_size=10)
        
        self.start_time = rospy.Time.now()
        rospy.loginfo("Fake Voice Commander started. Scheduling commands.")
        self.schedule_commands()

    def schedule_commands(self):
        """Creates one-shot timers to publish each command."""
        for object_name, command_time in self.voice_command_schedule:
            rospy.Timer(
                rospy.Duration(command_time),
                lambda event, o=object_name: self.publish_command(o),
                oneshot=True
            )
            rospy.loginfo(f"Scheduling voice command for '{object_name}' at t={command_time:.2f}s.")

    def publish_command(self, object_name):
        """Publishes a single voice command."""
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        rospy.loginfo(
            f"[Elapsed: {elapsed_time:.2f}s] HUMAN (VOICE SIM): Publishing command: '{object_name}'"
        )
        self.pub.publish(String(data=object_name))

if __name__ == '__main__':
    try:
        FakeVoiceCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass