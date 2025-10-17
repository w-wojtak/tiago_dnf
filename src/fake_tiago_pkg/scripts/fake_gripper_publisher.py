#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
import random

if __name__ == "__main__":
    rospy.init_node("fake_gripper_publisher", anonymous=True)
    pub = rospy.Publisher("/dxl_input/gripper_right", PointStamped, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    rospy.loginfo("fake_gripper_publisher started -> publishing /dxl_input/gripper_right")

    gripper_states = [0.0, 0.02, 0.04, 0.06, 0.03, 0.01]  # example open/close values
    i = 0

    while not rospy.is_shutdown():
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        # Simulate opening and closing motion
        gripper_value = gripper_states[i % len(gripper_states)]
        msg.point.x = gripper_value
        msg.point.y = 0.0
        msg.point.z = 0.0

        pub.publish(msg)
        rospy.loginfo(f"[gripper #{i}] position={gripper_value:.3f} m")

        i += 1
        rate.sleep()

