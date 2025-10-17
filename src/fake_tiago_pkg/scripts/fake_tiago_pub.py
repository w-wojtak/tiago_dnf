#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('fake_tiago_pub')
    pub = rospy.Publisher('/tiago/fake_cmd', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        msg = "hello_robot"
        rospy.loginfo(f"Publishing: {msg}")
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
