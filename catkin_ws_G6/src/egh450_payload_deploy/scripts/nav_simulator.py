#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
import sys
import tty
import termios


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def confirmation_callback(msg):
    if msg.data:
        print("\n[NAV SIM] Confirmation received - safe to resume flight.\n")
    else:
        print("\n[NAV SIM] Received confirmation message, but data was False.\n")


rospy.init_node('nav_simulator', anonymous=True)
pub = rospy.Publisher('/payload/nav_clearance', String, queue_size=10)
rospy.Subscriber('/payload/deploy_confirmation', Bool, confirmation_callback)
rospy.sleep(0.5)

print("Nav simulator ready (TEST TOOL ONLY - not the real nav code)")
print("Press T = Send clearance for Tracker")
print("Press E = Send clearance for EpiPen")
print("Press Q = Quit")

while not rospy.is_shutdown():
    key = get_key().upper()
    if key == 'T':
        print("Sending clearance: T")
        pub.publish('T')
    elif key == 'E':
        print("Sending clearance: E")
        pub.publish('E')
    elif key == 'Q':
        print("Quitting...")
        rospy.signal_shutdown("User quit")
        break