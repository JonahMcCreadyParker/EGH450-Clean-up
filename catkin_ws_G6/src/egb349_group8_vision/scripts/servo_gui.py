#!/usr/bin/env python3
import rospy # imports ros library
from std_msgs.msg import String # imports string message so we can publish commands to servo_command
import sys #Gives access to system level functions 
import tty # handle raw keyboard input on Linux.
import termios # handle raw keyboard input on Linux.

rospy.init_node('servo_gui', anonymous=True) #Registers this script with ROSCore as a node called servo_gui.
pub = rospy.Publisher('/servo_command', String, queue_size=10) #Creates a publisher that sends commands to the /servo_command topic that servo_controller.py is listening to
rospy.sleep(0.5)

def get_key(): #Gets the file descriptor of the keyboard input stream
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd) #Saves the current keyboard settings so they can be restored later
    try:
        tty.setraw(sys.stdin.fileno()) # keypresses are detected instantly without needing to press Enter
        key = sys.stdin.read(1) #Reads exactly 1 character from the keyboard
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings) #Restores the keyboard back to normal settings after reading the key
    return key

#displays what is happening
print("Servo Controller Ready")
print("Press 1 = Deploy M1")
print("Press 2 = Deploy M2")
print("Press n = Neutral")
print("Press q = Quit")

while not rospy.is_shutdown(): #loops forever until Ctrl + C
    key = get_key() # waits for a keypress and stores it in key
    if key == '1':
        print("Deploying Tracker...")
        pub.publish('1')
    elif key == '2':
        print("Deploying EpiPen...")
        pub.publish('2')
    elif key == 'n':
        print("Returning to neutral position")
        pub.publish('n')
    elif key == 'q':
        print("Quitting...")
        rospy.signal_shutdown("User quit") # allows for everything to quit, can also use ctrl + c 
        break
