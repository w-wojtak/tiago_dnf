#!/usr/bin/env python3
import rospy
import threading
import time
from std_msgs.msg import String, Float32MultiArray

class TaskExecutiveNode:
    def __init__(self):
        rospy.init_node('task_executive_node', anonymous=True)

        self.DNF_POS_TO_OBJECT = {-60.0: 'base', -20.0: 'load', 20.0: 'bearing', 40.0: 'motor'}
        
        # This publisher tells the vision system the object is gone
        self.pickup_announcement_pub = rospy.Publisher('/simulation/robot_pickup', String, queue_size=10)
        # This publisher provides the feedback for the DNF's u_f1 field
        self.robot_feedback_pub = rospy.Publisher('/simulation/robot_feedback', String, queue_size=10)
        
        rospy.Subscriber('threshold_crossings', Float32MultiArray, self.dnf_prediction_callback)
        
        rospy.loginfo("MINIMAL Task Executive started. Awaiting DNF predictions.")

        self.automatic_robot_feedback = True  # default ON
        # turn this off for sending info about robot waiting manually

    def dnf_prediction_callback(self, msg):
        """Called when the DNF predicts an action."""
        if not msg.data: return
        
        dnf_pos = msg.data[0]
        object_name = self.DNF_POS_TO_OBJECT.get(dnf_pos)
        
        if object_name:
            rospy.loginfo(f"Executive: Received DNF prediction for '{object_name}'.")
            
            if self.automatic_robot_feedback:
                # Use a thread to handle the delay and response
                t = threading.Thread(target=self.handle_robot_response, args=(object_name,))
                t.start()
            else:
                rospy.loginfo("Automatic robot feedback is OFF. Waiting for manual input...")

    def handle_robot_response(self, object_name):
        """Simulates the robot acting and then sends all necessary feedback."""
        try:
            # Simulate the time it takes for the robot to pick and move the object.
            robot_action_delay = 5.0 # seconds
            rospy.loginfo(f"Executive: Simulating {robot_action_delay}s robot action for '{object_name}'...")
            time.sleep(robot_action_delay)
            
            # --- After the delay, publish ALL feedback signals ---
            
            # 1. Announce the pickup to the vision system (for the 'vis' input)
            rospy.loginfo(f"Executive: Announcing pickup of '{object_name}' to vision simulator.")
            self.pickup_announcement_pub.publish(String(data=object_name))
            
            # 2. Announce the robot's arrival to the aggregator (for the 'rob' input / u_f1)
            rospy.loginfo(f"Executive: Sending robot feedback for '{object_name}'.")
            self.robot_feedback_pub.publish(String(data=object_name))
            
        except Exception as e:
            rospy.logerr(f"CRASH IN RESPONSE THREAD: {e}")

if __name__ == '__main__':
    try:
        TaskExecutiveNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass