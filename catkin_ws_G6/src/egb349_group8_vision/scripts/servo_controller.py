#!/usr/bin/env python3
import rospy #import ros library 
from std_msgs.msg import String 
from gpiozero import AngularServo # allows for communication between the GPIO pins and servo motors

# angles as measured on the 3D print 
negative_angle = 70
positive_angle = -40

# Sets servo 1 and 2 on respective pins with the pulse width as per the servo specifications 
M1 = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025)
M2 = AngularServo(13, min_pulse_width=0.0005, max_pulse_width=0.0025)

# Neutral on startup
M1.angle = negative_angle
M2.angle = negative_angle

def deploy_M1():
    rospy.loginfo("Deploying M1...") #ROS version of Print with time + date stamp 
    M1.angle = positive_angle
    rospy.sleep(1)

def deploy_M2():
    rospy.loginfo("Deploying M2...")
    M2.angle = positive_angle
    rospy.sleep(1)

def neutral(): # sets both motors to neutral (inactive) position
    rospy.loginfo("Returning to neutral...")
    M1.angle = negative_angle
    M2.angle = negative_angle
    rospy.sleep(1)

def command_callback(msg): #these two lines every time a message is received on the /servo_command 
    cmd = msg.data.strip().lower() # msg.data is the content, strip() removes spaces i.e. spaces before/after the input, and lower() makes it case in-sensitive  
    if cmd == '1':
        deploy_M1()
    elif cmd == '2':
        deploy_M2()
    elif cmd == 'n':
        neutral()
    else:
        rospy.logwarn(f"Unknown command: {cmd}") # prints unknown command in yellow instead of causing the drone to explode, f makes it a string which displays what exactly was the unknown command 

def shutdown_hook(): # shuts down with ctrl + c (built into rospy)
    rospy.loginfo("Stopping servo signals")
    M1.detach()
    M2.detach()

if __name__ == '__main__': # runs this file 
    rospy.init_node('servo_controller', anonymous=True) # registers this code with RosCore with name servo_controller (makes it a node)
    rospy.on_shutdown(shutdown_hook) # allows for the ctrl + c code to be active 

    rospy.Subscriber('/servo_command', String, command_callback) # allows for the messgaes to be displayed as per the command_callback loop
    rospy.loginfo("Servo controller node ready. Listening on /servo_command") # connects GUI with servo + prints that it is ready to receive commands

    rospy.spin() # allows for the code to continually run until the ctrl + c input

