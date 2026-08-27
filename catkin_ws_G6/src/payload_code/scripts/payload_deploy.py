#automatedpayloaddeploy
import rospy
from std_msgs.msg import Bool, String

#depend on image subsystem
TRIGGER_TOPIC = '/payload_deploy_trigger'   # topic the image subsystem publishes to
TRIGGER_TYPE  = Bool                        # message type it publishes True = "at 10cm, go"
DEPLOY_CMD    = '1'                         # '1' = Tracker, '2' = EpiPen
RETURN_TO_NEUTRAL_AFTER = 3.0               # seconds to wait before returning to neutral, or None to skip

class PayloadAutoDeploy:
    def __init__(self):
        self.deployed = False  # one-shot latch so we don't re-fire on repeated True messages

        self.pub = rospy.Publisher('/servo_command', String, queue_size=10)
        rospy.sleep(0.5)  # give the publisher time to register before anyone might publish

        rospy.Subscriber(TRIGGER_TOPIC, TRIGGER_TYPE, self.trigger_callback)

        rospy.loginfo(
            f"Payload auto-deploy node ready. Waiting for trigger on {TRIGGER_TOPIC}"
        )

    def trigger_callback(self, msg):
        # Adjust this line if TRIGGER_TYPE isn't Bool 
        is_go = bool(msg.data)

        if is_go and not self.deployed:
            rospy.loginfo("Image subsystem confirms correct altitude. Deploying payload now.")
            self.deployed = True
            self.pub.publish(DEPLOY_CMD)

            if RETURN_TO_NEUTRAL_AFTER is not None:
                rospy.Timer(
                    rospy.Duration(RETURN_TO_NEUTRAL_AFTER),
                    self.return_to_neutral,
                    oneshot=True
                )
        elif is_go and self.deployed:
            # Already deployed once ignore repeated triggers
            rospy.logdebug("Trigger received again, but payload already deployed. Ignoring.")

    def return_to_neutral(self, event):
        rospy.loginfo("Returning servo to neutral position after deployment.")
        self.pub.publish('n')


if __name__ == '__main__':
    rospy.init_node('payload_auto_deploy', anonymous=True)
    PayloadAutoDeploy()
    rospy.spin()