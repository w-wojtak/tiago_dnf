#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import random
import time

def main():
    rospy.init_node('fake_feedback_publisher')
    pub = rospy.Publisher('/fake_tiago/feedback', String, queue_size=5)
    rate = rospy.Rate(1.0)  # 1 Hz feedback
    count = 0
    rospy.loginfo("fake_feedback_publisher started -> /fake_tiago/feedback")
    while not rospy.is_shutdown():
        # Build a small feedback dict; adapt as needed
        feedback = {
            'timestamp': rospy.Time.now().to_sec(),
            'seq': count,
            'status': random.choice(['ok', 'busy', 'error']),
            'ee_pose': {
                'x': round(random.uniform(-0.6, 0.6), 3),
                'y': round(random.uniform(-0.6, 0.6), 3),
                'z': round(random.uniform(0.0, 1.2), 3)
            }
        }
        msg = String()
        msg.data = json.dumps(feedback)
        pub.publish(msg)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
