#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node that simulates a vision system by publishing fake object detections.
Publishes to /object_detections (std_msgs/String, JSON format).
Simulates picking up objects by changing their positions at scheduled times.
"""

import rospy
import json
from std_msgs.msg import String
from datetime import datetime

class FakeVisionPublisher:
    def __init__(self):
        rospy.init_node('fake_vision_publisher_node', anonymous=True)
        self.pub = rospy.Publisher('/object_detections', String, queue_size=10)
        
        # Initial positions of objects on the table (in meters)
        self.object_positions = {
            'base': {'x': -0.60, 'y': 0.25, 'z': 0.02},
            'load': {'x': -0.20, 'y': 0.30, 'z': 0.03},
            'bearing': {'x': 0.20, 'y': 0.28, 'z': 0.025},
            'motor': {'x': 0.40, 'y': 0.35, 'z': 0.05}
        }
        
        # Position after being "picked up" (moved away from table)
        self.picked_up_position = {'x': 1.5, 'y': 1.5, 'z': 0.5}  # Far away position
        
        # Pickup schedule: (object_name, pickup_time_seconds)
        # These are wall-clock times from node start
        self.pickup_schedule = [
            ('base', 5.0),      # Pick up 'base' at t=5s
            ('load', 10.0),     # Pick up 'load' at t=10s
            ('bearing', 15.0),  # Pick up 'bearing' at t=15s
            ('motor', 20.0),    # Pick up 'motor' at t=20s
        ]
        
        # Table detection boundaries (in meters)
        self.table_bounds = {
            'x_min': -0.82,
            'x_max': 0.71,
            'y_min': 0.06,
            'y_max': 0.52
        }
        
        # Publishing rate (Hz)
        self.publish_rate = rospy.get_param('~publish_rate', 2.0)  # Publish at 2 Hz by default
        
        rospy.loginfo("Fake Vision Publisher node started.")
        rospy.loginfo(f"Initial object positions: {self.object_positions}")
        rospy.loginfo(f"Pickup schedule: {self.pickup_schedule}")
        
        # Schedule pickup events
        self.schedule_pickups()
        
        # Start publishing detections at regular intervals
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_detections)
    
    def schedule_pickups(self):
        """Schedule object pickup events (position changes)"""
        for obj, pickup_time in self.pickup_schedule:
            rospy.loginfo(f"Scheduling pickup of '{obj}' at t={pickup_time:.1f}s")
            
            # Create timer for this pickup
            rospy.Timer(
                rospy.Duration(pickup_time),
                lambda event, o=obj: self.pickup_object(o),
                oneshot=True
            )
    
    def pickup_object(self, obj):
        """Simulate picking up an object by changing its position"""
        if obj in self.object_positions:
            old_pos = self.object_positions[obj].copy()
            # Move object to "picked up" position
            self.object_positions[obj] = self.picked_up_position.copy()
            
            rospy.loginfo(f"PICKED UP '{obj}': position changed from "
                         f"({old_pos['x']:.2f}, {old_pos['y']:.2f}, {old_pos['z']:.2f}) to "
                         f"({self.object_positions[obj]['x']:.2f}, {self.object_positions[obj]['y']:.2f}, {self.object_positions[obj]['z']:.2f})")
    
    def publish_detections(self, event):
        """Publish current object detections"""
        # Build detections list with current positions
        detections = []
        
        for obj_name, position in self.object_positions.items():
            detections.append({
                "object": obj_name,
                "position": position
            })
        
        # Create the detection message
        current_time = rospy.Time.now()
        detection_msg = {
            "timestamp": current_time.to_sec(),
            "frame_time": datetime.now().isoformat() + 'Z',
            "table_bounds": self.table_bounds,
            "detections": detections
        }
        
        # Publish the message
        msg = String()
        msg.data = json.dumps(detection_msg)
        self.pub.publish(msg)
        
        # Log only when objects are on the table (not picked up)
        on_table = [d for d in detections if d['position']['z'] < 0.1]
        if on_table:
            rospy.logdebug(f"Published {len(on_table)} objects on table, {len(detections)-len(on_table)} picked up")

def main():
    try:
        FakeVisionPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()