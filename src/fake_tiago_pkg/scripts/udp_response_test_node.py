#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time

def main():
    rospy.init_node("udp_response_test_node")
    pub = rospy.Publisher("/response_command", String, queue_size=10)
    rospy.sleep(1.0)

    test_messages = [
        "Here is the motor.",
        "Here is the load.",
        "Here is the bearing.",
        "Here is the base.",
        "Error: wrong object detected. Please use the correct one.",
    ]

    rospy.loginfo("Starting UDP response test...")
    rate = rospy.Rate(0.5)  # one message every 2 seconds

    for msg_text in test_messages:
        msg = String(data=msg_text)
        pub.publish(msg)
        rospy.loginfo(f"Published test message: {msg_text}")
        rate.sleep()

    rospy.loginfo("All test messages sent. Exiting.")
    time.sleep(1.0)

if __name__ == "__main__":
    main()
