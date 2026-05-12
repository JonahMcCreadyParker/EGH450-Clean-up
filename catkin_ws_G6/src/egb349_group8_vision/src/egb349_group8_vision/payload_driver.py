#!/usr/bin/env python3

import cv2
import rospy
import numpy as np

from gpiozero import AngularServo
from time import sleep

# Very similar to the Aruco_subscriber.py file, but this has pose estimation and a different HUD,
# as well as integrated camera intrinsics.
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError


class ArucoDetector():
    # These are top level constants that are easy to adjust

    # ROS Topics
    FRAME_SUB_TOPIC = "/depthai_node/image/compressed"
    CAMERA_INFO_TOPIC = "/depthai_node/camera/camera_info"
    OUTPUT_TOPIC = "/processed_aruco/image/compressed"
    POSE_LOG_TOPIC = "/aruco/pose_log"
    RVIZ_LOG_TOPIC = "/aruco/detection_log_text"

    # ArUco setup
    ARUCO_DICT = cv2.aruco.DICT_5X5_100
    MARKER_LENGTH = 0.200  # metres

    # Payload setup
    PAYLOAD_TRIGGER_ID = 32

    SERVO_M1_PIN = 12
    SERVO_M2_PIN = 13

    NEGATIVE_ANGLE = 70
    POSITIVE_ANGLE = -40

    # Payload trigger safety checks
    PAYLOAD_REQUIRED_COUNT = 5
    PAYLOAD_MAX_DISTANCE = 4.0

    # HUD setup
    SHOW_HUD = False
    SHOW_CROSSHAIR = True
    SHOW_DISTANCE_TEXT = True
    CROSSHAIR_SIZE = 18
    CROSSHAIR_GAP = 6
    HUD_BOX_WIDTH = 135
    HUD_BOX_HEIGHT = 34

    def __init__(self):
        # Camera calibration values are filled from the /camera_info topic
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detected_marker_log = {}

        self.br = CvBridge()

        # Payload servo setup
        self.M1 = AngularServo(
            self.SERVO_M1_PIN,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )

        self.M2 = AngularServo(
            self.SERVO_M2_PIN,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )

        # Start payload servos in neutral position
        self.M1.angle = self.NEGATIVE_ANGLE
        self.M2.angle = self.NEGATIVE_ANGLE
        sleep(1)

        # Payload state
        self.payload_deployed = False
        self.payload_seen_count = 0

        # Compatible ArUco dictionary selection
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(self.ARUCO_DICT)

        # Compatible detector parameter creation
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # Processed image output
        self.aruco_pub = rospy.Publisher(
            self.OUTPUT_TOPIC,
            CompressedImage,
            queue_size=10
        )

        # Camera calibration input for solvePnP
        self.camera_info_sub = rospy.Subscriber(
            self.CAMERA_INFO_TOPIC,
            CameraInfo,
            self.camera_info_callback
        )

        # Main camera image input
        self.frame_sub = rospy.Subscriber(
            self.FRAME_SUB_TOPIC,
            CompressedImage,
            self.img_callback
        )

        # Text log output for detected marker poses
        self.pose_log_pub = rospy.Publisher(
            self.POSE_LOG_TOPIC,
            String,
            queue_size=10
        )

        # RViz text marker showing all markers seen so far
        self.rviz_log_pub = rospy.Publisher(
            self.RVIZ_LOG_TOPIC,
            Marker,
            queue_size=10
        )

        # Log info
        rospy.loginfo("ArUco detector started")
        rospy.loginfo("Input image: {}".format(self.FRAME_SUB_TOPIC))
        rospy.loginfo("Output image: {}".format(self.OUTPUT_TOPIC))
        rospy.loginfo("Pose log: {}".format(self.POSE_LOG_TOPIC))
        rospy.loginfo("RViz detection log: {}".format(self.RVIZ_LOG_TOPIC))
        rospy.loginfo("Payload trigger marker ID: {}".format(self.PAYLOAD_TRIGGER_ID))

    def deploy_M1(self):
        rospy.loginfo("Deploying M1 payload...")
        self.M1.angle = self.POSITIVE_ANGLE
        sleep(1)

    def deploy_M2(self):
        rospy.loginfo("Deploying M2 payload...")
        self.M2.angle = self.POSITIVE_ANGLE
        sleep(1)

    def neutral_payload(self):
        rospy.loginfo("Returning payload servos to neutral...")
        self.M1.angle = self.NEGATIVE_ANGLE
        self.M2.angle = self.NEGATIVE_ANGLE
        sleep(1)

    def trigger_payload(self, marker_id, distance):
        # Do nothing if payload has already deployed
        if self.payload_deployed:
            return

        # Reset confirmation count if this is not the correct marker
        if marker_id != self.PAYLOAD_TRIGGER_ID:
            self.payload_seen_count = 0
            return

        # Ignore detections that look too far away to be reliable
        if distance > self.PAYLOAD_MAX_DISTANCE:
            rospy.logwarn(
                "Ignoring payload marker ID {} because distance looks too large: {:.2f} m".format(
                    marker_id,
                    distance
                )
            )
            self.payload_seen_count = 0
            return

        # Confirm the marker over multiple frames
        self.payload_seen_count += 1

        rospy.loginfo(
            "Payload marker ID {} confirmed frame {}/{}".format(
                marker_id,
                self.payload_seen_count,
                self.PAYLOAD_REQUIRED_COUNT
            )
        )

        if self.payload_seen_count >= self.PAYLOAD_REQUIRED_COUNT:
            rospy.loginfo("Payload trigger marker confirmed: ID {}".format(marker_id))

            # Deploy payload once
            self.deploy_M1()

            self.payload_deployed = True

    def camera_info_callback(self, msg):
        # Fills the camera calibration values using the camera intrinsics for pose estimation
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D, dtype=np.float64)
        rospy.loginfo_once("Camera info received")

    def img_callback(self, msg_in):
        # Main image processing callback
        # Convert ROS Compressed image to OpenCV format
        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg_in)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Run marker detection and pose estimation
        aruco = self.find_aruco(frame)

        # Publish processed image for RViz / rqt_image_view / GCS display
        self.publish_to_ros(aruco)

    def draw_crosshair(self, frame):
        # Draw a basic centre crosshair on the screen
        height, width = frame.shape[:2]
        centre_x = width // 2
        centre_y = height // 2

        cv2.line(
            frame,
            (centre_x - self.CROSSHAIR_SIZE, centre_y),
            (centre_x - self.CROSSHAIR_GAP, centre_y),
            (0, 255, 255),
            1
        )

        cv2.line(
            frame,
            (centre_x + self.CROSSHAIR_GAP, centre_y),
            (centre_x + self.CROSSHAIR_SIZE, centre_y),
            (0, 255, 255),
            1
        )

        cv2.line(
            frame,
            (centre_x, centre_y - self.CROSSHAIR_SIZE),
            (centre_x, centre_y - self.CROSSHAIR_GAP),
            (0, 255, 255),
            1
        )

        cv2.line(
            frame,
            (centre_x, centre_y + self.CROSSHAIR_GAP),
            (centre_x, centre_y + self.CROSSHAIR_SIZE),
            (0, 255, 255),
            1
        )

    def draw_marker_hud(self, frame, cX, cY, marker_id, distance, x, y, z):
        # Draw a small info box beside the marker
        box_x = cX + 10
        box_y = cY - 28

        frame_h, frame_w = frame.shape[:2]

        # Keep box on screen
        if box_x + self.HUD_BOX_WIDTH > frame_w:
            box_x = cX - self.HUD_BOX_WIDTH - 10

        if box_y < 5:
            box_y = cY + 10

        if box_y + self.HUD_BOX_HEIGHT > frame_h:
            box_y = frame_h - self.HUD_BOX_HEIGHT - 5

        # Background box
        cv2.rectangle(
            frame,
            (box_x, box_y),
            (box_x + self.HUD_BOX_WIDTH, box_y + self.HUD_BOX_HEIGHT),
            (0, 0, 0),
            -1
        )

        # Border
        cv2.rectangle(
            frame,
            (box_x, box_y),
            (box_x + self.HUD_BOX_WIDTH, box_y + self.HUD_BOX_HEIGHT),
            (0, 255, 0),
            1
        )

        # Small line from marker to box
        cv2.line(
            frame,
            (cX, cY),
            (box_x, box_y + 8),
            (0, 255, 0),
            1
        )

        # Text
        cv2.putText(
            frame,
            "ID: {}".format(marker_id),
            (box_x + 5, box_y + 11),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.33,
            (0, 255, 0),
            1
        )

        cv2.putText(
            frame,
            "D:{:.2f}m".format(distance),
            (box_x + 5, box_y + 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (0, 255, 255),
            1
        )

        cv2.putText(
            frame,
            "x:{:.2f} y:{:.2f} z:{:.2f}".format(x, y, z),
            (box_x + 5, box_y + 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.28,
            (0, 255, 255),
            1
        )

    def update_detection_log(self, marker_id, cX, cY, x, y, z, distance):
        # Save latest pose for each marker ID seen during this run
        self.detected_marker_log[int(marker_id)] = {
            "px": cX,
            "py": cY,
            "x": x,
            "y": y,
            "z": z,
            "distance": distance,
            "time": rospy.Time.now().to_sec()
        }

        log_lines = ["Seen ArUco markers:"]

        for saved_id in sorted(self.detected_marker_log.keys()):
            marker = self.detected_marker_log[saved_id]

            log_lines.append(
                "ID {}: pixel=({}, {}), x={:.2f}, y={:.2f}, z={:.2f}, dist={:.2f}m".format(
                    saved_id,
                    marker["px"],
                    marker["py"],
                    marker["x"],
                    marker["y"],
                    marker["z"],
                    marker["distance"]
                )
            )

        log_text = "\n".join(log_lines)

        # Publish as normal ROS string topic
        self.pose_log_pub.publish(log_text)

        # Publish as RViz text marker
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.ns = "aruco_detection_log"
        marker_msg.id = 0
        marker_msg.type = Marker.TEXT_VIEW_FACING
        marker_msg.action = Marker.ADD

        # Position of the text in RViz
        marker_msg.pose.position.x = 0.0
        marker_msg.pose.position.y = -0.8
        marker_msg.pose.position.z = 1.5

        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.z = 0.12

        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0

        marker_msg.text = log_text

        self.rviz_log_pub.publish(marker_msg)

    def find_aruco(self, frame):
        # Draw the centre HUD on every frame
        if self.SHOW_CROSSHAIR:
            self.draw_crosshair(frame)

        # Detect ArUco markers in the image and estimate their pose
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers in the current frame
        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                self.aruco_params
            )

            corners, ids, _ = detector.detectMarkers(gray)

        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        # If no markers are detected, reset payload confirmation count and move to next frame
        if ids is None or len(corners) == 0:
            self.payload_seen_count = 0
            return frame

        # Flatten the ids array for easier processing and logging
        ids = ids.flatten()

        # 3D marker corner positions based on real marker size
        half = self.MARKER_LENGTH / 2

        object_points = np.array([
            [-half,  half, 0.0],   # top-left
            [ half,  half, 0.0],   # top-right
            [ half, -half, 0.0],   # bottom-right
            [-half, -half, 0.0]    # bottom-left
        ], dtype=np.float32)

        payload_marker_seen_this_frame = False

        for marker_corners, marker_id in zip(corners, ids):
            marker_id = int(marker_id)

            pts = marker_corners.reshape((4, 2)).astype(np.float32)
            pts_int = pts.astype(int)

            # Draw marker borders
            cv2.polylines(
                frame,
                [pts_int],
                True,
                (0, 255, 0),
                2
            )

            # Marker centre point
            cX = int(np.mean(pts[:, 0]))
            cY = int(np.mean(pts[:, 1]))

            cv2.circle(
                frame,
                (cX, cY),
                4,
                (0, 0, 255),
                -1
            )

            # Make a single ID label for each marker
            cv2.putText(
                frame,
                "ID: {}".format(marker_id),
                (cX + 8, cY - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            if self.camera_matrix is None or self.dist_coeffs is None:
                rospy.logwarn_throttle(2.0, "Waiting for camera info...")
                continue

            frame_h, frame_w = frame.shape[:2]

            camera_matrix = self.camera_matrix.copy()

            # Camera info appears to be for 640x480, while image stream is 416x416
            scale_x = frame_w / 640.0
            scale_y = frame_h / 480.0

            camera_matrix[0, 0] *= scale_x  # fx
            camera_matrix[1, 1] *= scale_y  # fy
            camera_matrix[0, 2] *= scale_x  # cx
            camera_matrix[1, 2] *= scale_y  # cy

            success, rvec, tvec = cv2.solvePnP(
                object_points,
                pts,
                camera_matrix,
                self.dist_coeffs
            )

            if not success:
                rospy.logwarn_throttle(
                    1.0,
                    "solvePnP failed for marker ID {}".format(marker_id)
                )
                continue

            # Draw the 3D pose axis for each marker
            if hasattr(cv2, "drawFrameAxes"):
                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.MARKER_LENGTH * 0.5
                )

            # Convert the translation vector to x, y, z coordinates in metres
            raw_x = float(tvec[0])
            raw_y = float(tvec[1])
            raw_z = float(tvec[2])
            distance = float(np.linalg.norm(tvec))

            # Display convention for targeting:
            # x positive = marker right of camera centre
            # y positive = marker above camera centre, y is flipped because image coordinates fill from top-left
            # z positive = marker distance forward from the camera
            x = raw_x
            y = -raw_y
            z = raw_z

            # Trigger payload only after pose estimation succeeds
            if marker_id == self.PAYLOAD_TRIGGER_ID:
                payload_marker_seen_this_frame = True
                self.trigger_payload(marker_id, distance)

            # For logging detected marker poses
            self.update_detection_log(marker_id, cX, cY, x, y, z, distance)

            # Small HUD box beside each detected marker
            if self.SHOW_HUD:
                self.draw_marker_hud(frame, cX, cY, marker_id, distance, x, y, z)

            # Keep distance visible even when the HUD is disabled
            if self.SHOW_DISTANCE_TEXT and not self.SHOW_HUD:
                cv2.putText(
                    frame,
                    "Dist: {:.2f} m".format(distance),
                    (cX + 8, cY + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (0, 255, 255),
                    1
                )

                cv2.putText(
                    frame,
                    "x:{:.2f} y:{:.2f} z:{:.2f}".format(x, y, z),
                    (cX + 8, cY + 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (0, 255, 255),
                    1
                )

            rospy.loginfo_throttle(
                0.5,
                "Aruco ID {} pose: x={:.3f} m, y={:.3f} m, z={:.3f} m, dist={:.2f} m".format(
                    marker_id,
                    x,
                    y,
                    z,
                    distance
                )
            )

        # If other markers are visible but ID 32 is not, reset the payload confirmation count
        if not payload_marker_seen_this_frame and not self.payload_deployed:
            self.payload_seen_count = 0

        # Move on to next frame
        return frame

    def publish_to_ros(self, frame):
        # Convert processed OpenCV image back to ROS CompressedImage and publish
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"

        # Encode the processed image as JPEG for ROS CompressedImage
        success, encoded_image = cv2.imencode('.jpg', frame)

        if not success:
            rospy.logwarn("Failed to encode processed ArUco image")
            return

        msg_out.data = np.array(encoded_image).tobytes()
        self.aruco_pub.publish(msg_out)

    def shutdown(self):
        rospy.loginfo("Stopping servo signals")

        try:
            self.M1.detach()
            self.M2.detach()
        except Exception as e:
            rospy.logwarn("Servo shutdown issue: {}".format(e))


if __name__ == "__main__":
    rospy.init_node("aruco_detector")

    detector = ArucoDetector()
    rospy.on_shutdown(detector.shutdown)

    rospy.spin()