#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket


class UDPResponseSenderNode(object):
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('udp_response_sender_node', anonymous=True)

        # Get IP and port parameters (with defaults)
        self.udp_ip = rospy.get_param('~target_ip', '127.0.0.1')
        self.udp_port = rospy.get_param('~target_port', 5006)

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscribe to the response topic
        self.subscriber = rospy.Subscriber(
            'response_command',
            String,
            self.listener_callback,
            queue_size=10
        )

        rospy.loginfo("UDP Response Sender ready. Sending to %s:%d", self.udp_ip, self.udp_port)

    def listener_callback(self, msg):
        message = msg.data.strip()
        try:
            self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
            rospy.loginfo("Sent response: '%s'", message)
        except Exception as e:
            rospy.logerr("Failed to send UDP response: %s", str(e))


def main():
    node = UDPResponseSenderNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
