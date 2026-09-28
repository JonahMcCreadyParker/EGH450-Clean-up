#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys
import tty
import termios


from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()


rospy.init_node('manual_override', anonymous=True)
pub = rospy.Publisher('/motor_manual_override', String, queue_size=10)
rospy.sleep(0.5)  # let the publisher register before we start sending


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


print("Manual override ready (GROUND TEST ONLY)")
print("Press T = Deploy Tracker")
print("Press E = Deploy EpiPen")
print("Press N = Force neutral")
print("Press Q = Quit")

while not rospy.is_shutdown():
    key = get_key().upper()
    if key == 'T':
        print("Deploying Tracker...")
        pub.publish('T')
    elif key == 'E':
        print("Deploying EpiPen...")
        pub.publish('E')
    elif key == 'N':
        print("Forcing neutral...")
        pub.publish('N')
    elif key == 'Q':
        print("Quitting...")
        rospy.signal_shutdown("User quit")
        break