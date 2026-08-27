#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from gpiozero import AngularServo

try:
    from mavros_msgs.msg import StatusText
    HAVE_MAVROS = True
except ImportError:
    HAVE_MAVROS = False

NEUTRAL_ANGLE = 40      
DEPLOY_ANGLE = -40


RETURN_DELAY = 2.0  # seconds before auto-returning to neutral after a deploy

# M1 = Tracker ('T') on GPIO12, M2 = EpiPen ('E') on GPIO13
MOTORS = {
    'T': AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025),
    'E': AngularServo(13, min_pulse_width=0.0005, max_pulse_width=0.0025),
}

#nothing has been published yet 
confirm_pub = None
status_pub = None

#sends messages to the QGround Control - so we can see what is happening
def send_status(text, severity=6):
    rospy.loginfo(text)
    if status_pub is not None:
        msg = StatusText()
        msg.severity = severity
        msg.text = text
        status_pub.publish(msg)

#code that deploys payloads
def deploy(payload, confirm): # from below, if it is T or E it will run 
    rospy.loginfo(f"Deploying {payload}...") # text displayed
    MOTORS[payload].angle = DEPLOY_ANGLE # move arm to deployed angle
    send_status(f"PDD: {payload} deployed") # tells us that it is deployed

    if confirm:
        confirm_pub.publish(payload)


    rospy.Timer(rospy.Duration(RETURN_DELAY), lambda event: neutral(payload), oneshot=True) # return arm to netural after 2 seconds


def neutral(payload):
    rospy.loginfo(f"{payload} arm returning to neutral.")
    MOTORS[payload].angle = NEUTRAL_ANGLE

# tells "main" that it is an automated message
def command_callback(msg):
    """Automated path - from payload_driver.py."""
    _handle_command(msg.data, source='automated', confirm=True)

# tells "main" that it is a manual message
def manual_override_callback(msg):
    """Ground-test path - from manual_override.py. Never confirms back."""
    _handle_command(msg.data, source='MANUAL OVERRIDE', confirm=False)


def _handle_command(data, source, confirm): #data = reads T, E or N. Source is whether it is automated or manual override. Confirm - tells payload_drive once done (if true)
    cmd = data.strip().upper() #cleans up the input
    if cmd in MOTORS: #making sure it is T or E 
        rospy.loginfo(f"[{source}] {cmd} command received.") #if it is a valid payload command, then deploy the payload 
        deploy(cmd, confirm=confirm)
    elif cmd == 'N': #if it is N return to neutral 
        for payload in MOTORS:
            neutral(payload)
    else:
        rospy.logwarn(f"[{source}] Unknown command: {cmd}") 


def shutdown_hook():
    rospy.loginfo("Stopping motor signals")
    for motor in MOTORS.values():
        motor.detach()


if __name__ == '__main__':
    rospy.init_node('motor_controller', anonymous=True)
    rospy.on_shutdown(shutdown_hook)     # tells ROS: "when this node is shut down (Ctrl+C), run shutdown_hook first"

    confirm_pub = rospy.Publisher('/motor_confirmation', String, queue_size=10)     # creates a publisher - gives this program the ABILITY to send messages

    if HAVE_MAVROS:
        status_pub = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=10)
    rospy.sleep(0.5) # gives ROS time to register the publishers above with the rest of the system,

    rospy.Subscriber('/motor_command', String, command_callback) # starts listening on '/motor_command',  every time a message arrives here, ROS automatically calls command_callback
    rospy.Subscriber('/motor_manual_override', String, manual_override_callback) # starts listening on '/motor_manual_override', every time a message arrives here, ROS automatically calls manual_override_callback

 # just prints a message to the terminal, confirming setup finished and it's ready to go
    rospy.loginfo(
        "motor controller ready. /motor_command (automated), "
        "/motor_manual_override (ground test)."
    )
    rospy.spin()