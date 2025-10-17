#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class FakeHumanTaker:
    def __init__(self):
        rospy.init_node('fake_human_node', anonymous=True)
        rospy.Subscriber('/robot_state/handover_ready', String, self.robot_ready_callback)
        self.voice_pub = rospy.Publisher('/voice_command', String, queue_size=10)
        # We don't need the 'take' publisher if we simplify the logic
        rospy.loginfo("REACTIVE Fake Human Node started.")

    def robot_ready_callback(self, msg):
        object_name = msg.data
        rospy.loginfo(f"HUMAN (SIM): Sees robot is ready with '{object_name}'.")
        rospy.sleep(2.0) # Human reaction time
        rospy.loginfo(f"HUMAN (SIM): 'Give me the {object_name}'. Publishing voice command.")
        self.voice_pub.publish(String(data=object_name))

if __name__ == '__main__':
    try:
        FakeHumanTaker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass