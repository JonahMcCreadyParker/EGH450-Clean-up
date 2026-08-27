#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

try:
    from mavros_msgs.msg import StatusText  # lets us push text to QGroundControl
    HAVE_MAVROS = True
except ImportError:
    # Means mavros_msgs isn't installed on this machine - the node still works, it just falls back to terminal-only logging 
    HAVE_MAVROS = False

# Topic names - edit these once nav confirms them, nothing else needs to change
NAV_CLEARANCE_TOPIC = '/payload/nav_clearance' # String: 'T' or 'E'  <-- CONFIRM WITH NAV
CONFIRM_TOPIC = '/payload/deploy_confirmation' # Bool: True = safe to resume flight

VALID_PAYLOADS = ('T', 'E')  # T = Tracker, E = EpiPen

deployed = {p: False for p in VALID_PAYLOADS} # starting state, it is saying that for both of the payloads, they have not been deployed, later on it will change to true and it will therefore only deploy once

servo_pub = None
confirm_pub = None
status_pub = None

#this should theoretically display all of the messages in different severity to QGroundControl 
def send_status(text, severity=6):
    """Log to the terminal AND push a line to QGroundControl's message
    panel (severity 6 = INFO, 4 = WARNING, 2 = ERROR - see MAV_SEVERITY)."""
    rospy.loginfo(text)
    if status_pub is not None:
        msg = StatusText()
        msg.severity = severity
        msg.text = text
        status_pub.publish(msg)


def nav_clearance_callback(msg):  # called automatically every time nav publishes
    payload = msg.data.strip().upper()  # strip spaces, ignore case, same as command_callback did

    if payload not in VALID_PAYLOADS: # makes sure that the inputs are T or E 
        rospy.logwarn(f"Unknown payload code from nav: '{payload}'")
        return

# Nav might re-send clearance (e.g. message repeated on the topic) - ignore it, don't fire the servo a second time
    if deployed[payload]:
        rospy.logdebug(f"{payload} already deployed, ignoring repeat clearance.")
        return

    deployed[payload] = True
    send_status(f"Nav clearance received - deploying {payload}")
    servo_pub.publish(payload)


def servo_confirmation_callback(msg):  # called automatically once servo_controller confirms
    payload = msg.data.strip().upper()
    send_status(f"{payload} deploy confirmed - resuming flight")
    confirm_pub.publish(Bool(True))


if __name__ == '__main__':
    rospy.init_node('payload_driver', anonymous=True)  # registers this file as a node

    servo_pub = rospy.Publisher('/motor_command', String, queue_size=10)
    confirm_pub = rospy.Publisher(CONFIRM_TOPIC, Bool, queue_size=10)
    if HAVE_MAVROS:
        status_pub = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=10)

    rospy.sleep(0.5)  # let the publishers register before anyone can talk to us

    rospy.Subscriber(NAV_CLEARANCE_TOPIC, String, nav_clearance_callback)
    rospy.Subscriber('/motor_confirmation', String, servo_confirmation_callback)

    send_status(f"Payload driver ready. Waiting on {NAV_CLEARANCE_TOPIC}")

    rospy.spin()  # keeps the node alive, listening, until ctrl+c