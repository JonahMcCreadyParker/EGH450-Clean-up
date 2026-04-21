#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from gpiozero import AngularServo

servo = AngularServo(
    13,
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)

def callback(msg):
    if msg.data:
        rospy.loginfo("Servo to 90°")
        servo.angle = 90
    else:
        rospy.loginfo("Servo to -90°")
        servo.angle = -90

def shutdown():
    servo.detach()

if __name__ == '__main__':
    rospy.init_node('servo_controller')

    sub = rospy.Subscriber('/actuator_control/actuator_a', Bool, callback)

    rospy.on_shutdown(shutdown)

    rospy.spin()

