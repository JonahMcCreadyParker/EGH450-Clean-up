#!/usr/bin/env python3

import rospy
from egb349_group8_vision.aruco_detector import ArucoDetector


def main():
    rospy.init_node("egb349_aruco_hud_detector", anonymous=True)
    ArucoDetector()
    #rospy.on_shutdown(detector.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()