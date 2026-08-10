#!/usr/bin/env python3

import json
import time
from pathlib import Path
from typing import Dict, List
import cv2
import depthai as dai
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# Still gotta make this
#class DaiYoloPublisher:
    


def main() -> None:
    rospy.init_node("dai_yolo_publisher")

    try:
        publisher = DaiYoloPublisher()
        publisher.run()
    except Exception as error:
        rospy.logfatal("DaiYoloPublisher failed: %s", error)
        raise


if __name__ == "__main__":
    main()
