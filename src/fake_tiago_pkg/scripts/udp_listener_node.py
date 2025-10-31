#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket
import threading

UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 5005

UDP_SEND_IP = "10.205.240.222"
UDP_SEND_PORT = 5006

# Commands and responses
COMMANDS = ["give_motor", "give_load", "give_bearing", "give_base"]
RESPONSES = {
    "give_motor": "Here is the motor.",
    "give_load": "Here is the load.",
    "give_bearing": "Here is the bearing.",
    "give_base": "Here is the base."
}

# Set how many times each command can be published (1 or 2)
REPEAT_COUNT = 2  # change to 2 if you want each command allowed twice

class VoiceListener:
    def __init__(self):
        rospy.init_node('voice_listener', anonymous=True)

        # ROS publisher
        self.pub = rospy.Publisher('voice_message', String, queue_size=10)

        # Track how many times each command has been published
        self.sent_counts = {cmd: 0 for cmd in COMMANDS}

        # UDP socket for sending confirmations
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.loginfo(f"Listening for UDP messages on port {UDP_LISTEN_PORT}...")

        self.thread = threading.Thread(target=self.listen_loop)
        self.thread.daemon = True
        self.thread.start()

    def listen_loop(self):
        sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock_recv.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))

        while not rospy.is_shutdown():
            try:
                data, addr = sock_recv.recvfrom(1024)
                message = data.decode().strip()

                if message in COMMANDS:
                    if self.sent_counts[message] < REPEAT_COUNT:
                        rospy.loginfo(f"Received command: {message}")
                        self.pub.publish(String(data=message))
                        self.sent_counts[message] += 1

                        # Send back confirmation
                        response_text = RESPONSES.get(message, "")
                        if response_text:
                            self.sock_send.sendto(response_text.encode(), (UDP_SEND_IP, UDP_SEND_PORT))
                            rospy.loginfo(f"Sent confirmation: '{response_text}'")
                    else:
                        rospy.loginfo(f"Command '{message}' ignored (already published {REPEAT_COUNT} times).")
                else:
                    rospy.loginfo(f"Ignored unknown command: {message}")

            except Exception as e:
                rospy.logerr(f"Error in UDP listener: {e}")

def main():
    node = VoiceListener()
    rospy.spin()

if __name__ == '__main__':
    main()
