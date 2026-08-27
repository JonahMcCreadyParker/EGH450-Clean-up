#!/usr/bin/env python3

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_matrix


class ArucoDetector():

    # ROS Topics
    FRAME_SUB_TOPIC = "/depthai_node/image/compressed"
    YOLO_FRAME_TOPIC = "/yolo/image/compressed"
    CAMERA_INFO_TOPIC = "/depthai_node/camera/camera_info"

    OUTPUT_TOPIC = "/processed_vision/image/compressed"

    POSE_LOG_TOPIC = "/aruco/pose_log"
    RVIZ_LOG_TOPIC = "/aruco/detection_log_text"
    ROI_TOPIC = "/guidance/roi"

    # OptiTrack localisation
    UAV_POSE_TOPIC = "/mavros/vision_pose/pose"
    WORLD_POSITION_TOPIC = "/aruco/world_position"

    # Camera position relative to OptiTrack rigid body origin
    # x = forward, y = left, z = up
    CAMERA_OFFSET = np.array([
        -0.10,     # Camera is 10 cm behind
         0.00,
        -0.15      # Camera is 15 cm below
    ])

    # ArUco setup
    ARUCO_DICT = cv2.aruco.DICT_5X5_100
    MARKER_LENGTH = 0.200  # metres

    # HUD setup
    SHOW_CROSSHAIR = True
    SHOW_DISTANCE_TEXT = True

    CROSSHAIR_SIZE = 18
    CROSSHAIR_GAP = 6


    def __init__(self):

        self.camera_matrix = None
        self.dist_coeffs = None

        self.detected_marker_log = {}

        self.uav_pose = None

        # Latest raw and YOLO images
        self.latest_raw_frame = None

        self.br = CvBridge()

        # ArUco detector setup
        if hasattr(
            cv2.aruco,
            "getPredefinedDictionary"
        ):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(
                self.ARUCO_DICT
            )
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(
                self.ARUCO_DICT
            )

        if hasattr(
            cv2.aruco,
            "DetectorParameters_create"
        ):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # Camera calibration
        self.camera_info_sub = rospy.Subscriber(
            self.CAMERA_INFO_TOPIC,
            CameraInfo,
            self.camera_info_callback
        )

        # Clean camera image
        self.frame_sub = rospy.Subscriber(
            self.FRAME_SUB_TOPIC,
            CompressedImage,
            self.raw_frame_callback,
            queue_size=1
        )

        # YOLO image
        self.yolo_frame_sub = rospy.Subscriber(
            self.YOLO_FRAME_TOPIC,
            CompressedImage,
            self.yolo_frame_callback,
            queue_size=1
        )

        # OptiTrack UAV pose
        self.uav_pose_sub = rospy.Subscriber(
            self.UAV_POSE_TOPIC,
            PoseStamped,
            self.uav_pose_callback
        )

        # Final processed image
        self.aruco_pub = rospy.Publisher(
            self.OUTPUT_TOPIC,
            CompressedImage,
            queue_size=10
        )

        # ArUco pose log
        self.pose_log_pub = rospy.Publisher(
            self.POSE_LOG_TOPIC,
            String,
            queue_size=10
        )

        # RViz text marker
        self.rviz_log_pub = rospy.Publisher(
            self.RVIZ_LOG_TOPIC,
            Marker,
            queue_size=10
        )

        # ROI
        self.roi_pub = rospy.Publisher(
            self.ROI_TOPIC,
            PoseStamped,
            queue_size=1
        )

        # Marker world position
        self.world_position_pub = rospy.Publisher(
            self.WORLD_POSITION_TOPIC,
            PointStamped,
            queue_size=10
        )

        self.roi_triggered = False

        rospy.loginfo("ArUco detector started")
        rospy.loginfo(
            "Raw input: {}".format(
                self.FRAME_SUB_TOPIC
            )
        )
        rospy.loginfo(
            "YOLO input: {}".format(
                self.YOLO_FRAME_TOPIC
            )
        )
        rospy.loginfo(
            "Output: {}".format(
                self.OUTPUT_TOPIC
            )
        )


    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(
            msg.K,
            dtype=np.float64
        ).reshape((3, 3))

        self.dist_coeffs = np.array(
            msg.D,
            dtype=np.float64
        )

        rospy.loginfo_once(
            "Camera info received"
        )


    def uav_pose_callback(self, msg):
        self.uav_pose = msg


    def raw_frame_callback(self, msg_in):
        try:
            self.latest_raw_frame = (
                self.br.compressed_imgmsg_to_cv2(
                    msg_in
                )
            )

        except CvBridgeError as e:
            rospy.logerr(e)


    def yolo_frame_callback(self, msg_in):
        try:
            yolo_frame = (
                self.br.compressed_imgmsg_to_cv2(
                    msg_in
                )
            )

        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Use clean image for ArUco detection
        if self.latest_raw_frame is None:
            frame = yolo_frame.copy()
        else:
            frame = self.latest_raw_frame.copy()

        # Add ArUco onto YOLO image
        self.find_aruco(
            frame,
            yolo_frame
        )

        self.publish_to_ros(
            yolo_frame
        )


    def camera_to_world(
        self,
        raw_x,
        raw_y,
        raw_z
    ):

        if self.uav_pose is None:
            return None

        # Camera optical frame -> UAV body frame
        # Camera looks down, top of image points towards UAV front
        marker_body = np.array([
            -raw_y,
            -raw_x,
            -raw_z
        ])

        marker_body += self.CAMERA_OFFSET

        pose = self.uav_pose.pose

        uav_world = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])

        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

        rotation = quaternion_matrix(
            quaternion
        )[:3, :3]

        marker_world = (
            uav_world
            + rotation.dot(marker_body)
        )

        return marker_world


    def publish_world_position(
        self,
        marker_world
    ):

        msg = PointStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.point.x = float(marker_world[0])
        msg.point.y = float(marker_world[1])
        msg.point.z = float(marker_world[2])

        self.world_position_pub.publish(
            msg
        )


    def draw_crosshair(self, frame):
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


    def update_detection_log(
        self,
        marker_id,
        cX,
        cY,
        x,
        y,
        z,
        distance
    ):

        self.detected_marker_log[int(marker_id)] = {
            "px": cX,
            "py": cY,
            "x": x,
            "y": y,
            "z": z,
            "distance": distance
        }

        log_lines = [
            "Seen ArUco markers:"
        ]

        for saved_id in sorted(
            self.detected_marker_log.keys()
        ):
            marker = self.detected_marker_log[
                saved_id
            ]

            log_lines.append(
                "ID {}: pixel=({}, {}), "
                "x={:.2f}, y={:.2f}, z={:.2f}, "
                "dist={:.2f}m".format(
                    saved_id,
                    marker["px"],
                    marker["py"],
                    marker["x"],
                    marker["y"],
                    marker["z"],
                    marker["distance"]
                )
            )

        log_text = "\n".join(
            log_lines
        )

        self.pose_log_pub.publish(
            log_text
        )

        # RViz text marker
        marker_msg = Marker()

        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = rospy.Time.now()

        marker_msg.ns = "aruco_detection_log"
        marker_msg.id = 0

        marker_msg.type = Marker.TEXT_VIEW_FACING
        marker_msg.action = Marker.ADD

        marker_msg.pose.position.x = 0.0
        marker_msg.pose.position.y = -0.8
        marker_msg.pose.position.z = 1.5

        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.z = 0.12

        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0

        marker_msg.text = log_text

        self.rviz_log_pub.publish(
            marker_msg
        )


    def find_aruco(
        self,
        frame,
        output_frame
    ):

        if self.SHOW_CROSSHAIR:
            self.draw_crosshair(
                output_frame
            )

        # Marker detection uses clean camera frame
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        if hasattr(
            cv2.aruco,
            "ArucoDetector"
        ):
            detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                self.aruco_params
            )

            corners, ids, _ = detector.detectMarkers(
                gray
            )

        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        if ids is None or len(corners) == 0:
            return

        ids = ids.flatten()

        half = self.MARKER_LENGTH / 2

        object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)

        for marker_corners, marker_id in zip(
            corners,
            ids
        ):
            pts = marker_corners.reshape(
                (4, 2)
            ).astype(np.float32)

            frame_h, frame_w = frame.shape[:2]
            output_h, output_w = output_frame.shape[:2]

            # YOLO image is a centre crop of the raw image
            crop_size = min(frame_w, frame_h)
            crop_x = (frame_w - crop_size) / 2.0
            crop_y = (frame_h - crop_size) / 2.0
            scale_x = output_w / float(crop_size)
            scale_y = output_h / float(crop_size)

            output_pts = pts.copy()
            output_pts[:, 0] = (output_pts[:, 0] - crop_x) * scale_x
            output_pts[:, 1] = (output_pts[:, 1] - crop_y) * scale_y
            pts_int = output_pts.astype(int)

            # Draw marker border
            cv2.polylines(
                output_frame,
                [pts_int],
                True,
                (0, 255, 0),
                2
            )

            cX = int(
                np.mean(pts[:, 0])
            )

            cY = int(
                np.mean(pts[:, 1])
            )

            output_cX = int(
                (cX - crop_x) * scale_x
            )

            output_cY = int(
                (cY - crop_y) * scale_y
            )

            cv2.circle(
                output_frame,
                (output_cX, output_cY),
                4,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                output_frame,
                "ID: {}".format(marker_id),
                (output_cX + 8, output_cY - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            if (
                self.camera_matrix is None
                or self.dist_coeffs is None
            ):
                rospy.logwarn_throttle(
                    2.0,
                    "Waiting for camera info..."
                )
                continue

            camera_matrix = (
                self.camera_matrix.copy()
            )

            output_camera_matrix = camera_matrix.copy()

            output_camera_matrix[0, 0] *= scale_x
            output_camera_matrix[1, 1] *= scale_y

            output_camera_matrix[0, 2] = (
                output_camera_matrix[0, 2] - crop_x
            ) * scale_x

            output_camera_matrix[1, 2] = (
                output_camera_matrix[1, 2] - crop_y
            ) * scale_y

            success, rvec, tvec = (
                cv2.solvePnP(
                    object_points,
                    pts,
                    camera_matrix,
                    self.dist_coeffs
                )
            )

            if not success:
                rospy.logwarn_throttle(
                    1.0,
                    "solvePnP failed for marker Id {}".format(
                        marker_id
                    )
                )
                continue

            if hasattr(
                cv2,
                "drawFrameAxes"
            ):
                cv2.drawFrameAxes(
                    output_frame,
                    output_camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.MARKER_LENGTH * 0.5
                )

            raw_x = float(tvec[0])
            raw_y = float(tvec[1])
            raw_z = float(tvec[2])

            distance = float(
                np.linalg.norm(tvec)
            )

            # Display convention
            x = raw_x
            y = -raw_y
            z = raw_z

            marker_world = (
                self.camera_to_world(
                    raw_x,
                    raw_y,
                    raw_z
                )
            )

            if marker_world is not None:
                self.publish_world_position(
                    marker_world
                )

                rospy.loginfo_throttle(
                    0.5,
                    "ArUco ID {} world: "
                    "x={:.3f}, y={:.3f}, z={:.3f}".format(
                        marker_id,
                        marker_world[0],
                        marker_world[1],
                        marker_world[2]
                    )
                )

            self.update_detection_log(
                marker_id,
                cX,
                cY,
                x,
                y,
                z,
                distance
            )

            if marker_world is not None:
                self.publish_test_roi(
                    marker_id,
                    marker_world
                )

            if self.SHOW_DISTANCE_TEXT:
                cv2.putText(
                    output_frame,
                    "Dist: {:.2f} m".format(
                        distance
                    ),
                    (output_cX + 8, output_cY + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (0, 255, 255),
                    1
                )

                cv2.putText(
                    output_frame,
                    "x:{:.2f} y:{:.2f} z:{:.2f}".format(
                        x,
                        y,
                        z
                    ),
                    (output_cX + 8, output_cY + 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (0, 255, 255),
                    1
                )


    # ROI addition for Jonah
    def publish_test_roi(
        self,
        marker_id,
        marker_world
    ):

        if self.roi_triggered:
            return

        roi_msg = PoseStamped()

        roi_msg.header.stamp = rospy.Time.now()
        roi_msg.header.frame_id = "map"

        # Use detected marker world position
        roi_msg.pose.position.x = float(marker_world[0])
        roi_msg.pose.position.y = float(marker_world[1])
        roi_msg.pose.position.z = 1.5

        # No rotation
        roi_msg.pose.orientation.x = 0.0
        roi_msg.pose.orientation.y = 0.0
        roi_msg.pose.orientation.z = 0.0
        roi_msg.pose.orientation.w = 1.0

        self.roi_pub.publish(
            roi_msg
        )

        self.roi_triggered = True

        rospy.logwarn(
            "ArUco ID {} detected - ROI diversion requested "
            "to ({:.2f}, {:.2f}, 1.5)".format(
                marker_id,
                marker_world[0],
                marker_world[1]
            )
        )


    def publish_to_ros(
        self,
        frame
    ):

        msg_out = CompressedImage()

        msg_out.header.stamp = rospy.Time.now()
        msg_out.header.frame_id = "oak_rgb_camera"
        msg_out.format = "jpeg"

        success, encoded_image = cv2.imencode(
            ".jpg",
            frame
        )

        if not success:
            rospy.logwarn(
                "Failed to encode processed image"
            )
            return

        msg_out.data = (
            encoded_image.tobytes()
        )

        self.aruco_pub.publish(
            msg_out
        )


if __name__ == "__main__":
    rospy.init_node("aruco_detector")

    detector = ArucoDetector()

    rospy.spin()