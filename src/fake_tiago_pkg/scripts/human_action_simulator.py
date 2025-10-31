#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class HumanActionSimulator:
    def __init__(self):
        rospy.init_node('human_action_simulator', anonymous=True)

        # This schedule simulates WHEN a human picks up each object from the table.
        # The timings are NOT uniform to be more realistic.
        self.pickup_schedule = [
            ('base', 16.0),      # First pickup at t=16s
            ('load', 26),     # Next pickup  
            ('bearing', 34.0),  # Next pickup  
            ('motor', 43.0)     # Final pickup  
        ]

        # --- ADD THIS ---
        # Store the start time of the node to calculate elapsed time.
        self.start_time = rospy.Time.now()
        # --- END OF ADDED BLOCK ---

        # This topic is used to tell the world simulator (fake_ar_publisher)
        # that an object has been removed from the table.
        self.pub = rospy.Publisher('/simulation/human_pickup', String, queue_size=10)
        rospy.loginfo("Human Action Simulator started. Scheduling pickup events.")
        self.schedule_pickups()

    def schedule_pickups(self):
        for object_name, pickup_time in self.pickup_schedule:
            rospy.Timer(
                rospy.Duration(pickup_time),
                lambda event, o=object_name: self.publish_pickup(o),
                oneshot=True
            )

    def publish_pickup(self, object_name):
        # --- MODIFIED THIS BLOCK ---
        # Calculate the elapsed time since the node started.
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        # Add the elapsed time to the log message for clarity.
        rospy.loginfo(
            f"[Elapsed: {elapsed_time:.2f}s] HUMAN (SIMULATED): Picking up '{object_name}' from the table."
        )
        # --- END OF MODIFICATION ---
        
        self.pub.publish(String(data=object_name))

if __name__ == '__main__':
    HumanActionSimulator()
    rospy.spin()