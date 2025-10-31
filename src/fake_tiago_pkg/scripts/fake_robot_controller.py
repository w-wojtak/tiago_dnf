#!/usr/bin/env python3
import rospy
import threading
from geometry_msgs.msg import PoseStamped, PointStamped, Pose

class FakeRobotController:
    def __init__(self):
        rospy.init_node('fake_robot_controller')

        # --- State Variables ---
        self.current_pose = Pose()
        self.current_pose.position.z = 0.25 # Start in a home/retracted position
        self.current_pose.orientation.w = 1.0
        
        self.gripper_state = 1.0 # 1.0 = open, 0.0 = close
        self._lock = threading.Lock()

        # --- Publishers for State Feedback ---
        self.pose_feedback_pub = rospy.Publisher('/cartesian/right_arm/pose', PoseStamped, queue_size=10)
        
        # --- Subscribers for Commands ---
        rospy.Subscriber('/dxl_input/pos_right', PoseStamped, self.pose_command_callback)
        rospy.Subscriber('/dxl_input/gripper_right', PointStamped, self.gripper_command_callback)

        # Timer to continuously publish the robot's current state
        rospy.Timer(rospy.Duration(0.1), self.publish_state_feedback)

        rospy.loginfo("IMPROVED Fake Robot Controller started.")

    def publish_state_feedback(self, event):
        """Publishes the current state of the fake robot's arm."""
        with self._lock:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose = self.current_pose
            self.pose_feedback_pub.publish(pose_msg)

    def gripper_command_callback(self, msg):
        """Handles gripper commands."""
        rospy.loginfo("--- FAKE ROBOT: Received GRIPPER command ---")
        # Use a thread to avoid blocking the subscriber
        thread = threading.Thread(target=self.simulate_gripper_action, args=(msg,))
        thread.start()

    def simulate_gripper_action(self, msg):
        new_state = msg.point.x
        rospy.sleep(0.5) # Simulate action time
        with self._lock:
            self.gripper_state = new_state
        rospy.loginfo(f"Fake Gripper: Action complete. State is now {self.gripper_state}")

    def pose_command_callback(self, msg):
        """Handles arm movement commands."""
        rospy.loginfo(f"--- FAKE ROBOT: Received ARM command to move to z={msg.pose.position.z:.2f} ---")
        # Use a thread to simulate the long-running movement
        thread = threading.Thread(target=self.simulate_arm_movement, args=(msg.pose,))
        thread.start()

    def simulate_arm_movement(self, target_pose):
        """Simulates the time it takes for the arm to move."""
        rospy.loginfo("Fake Arm: Movement started...")
        rospy.sleep(1.0) # Simulate 1 seconds of movement time
        with self._lock:
            self.current_pose = target_pose
        rospy.loginfo("Fake Arm: Movement finished.")

def main():
    FakeRobotController()
    rospy.spin()

if __name__ == '__main__':
    main()