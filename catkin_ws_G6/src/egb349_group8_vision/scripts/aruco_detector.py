#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import math
import message_filters

from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion


class ArucoDetector():

    # ROS topics
    FRAME_SUB_TOPIC = "/depthai_node/image/compressed"
    YOLO_FRAME_TOPIC = "/yolo/image/compressed"
    CAMERA_INFO_TOPIC = "/depthai_node/camera/camera_info"

    OUTPUT_TOPIC = "/processed_vision/image/compressed"

    POSE_LOG_TOPIC = "/aruco/pose_log"
    RVIZ_LOG_TOPIC = "/aruco/detection_log_text"

    ROI_TOPIC = "/guidance/roi"
    ROI_TYPE_TOPIC = "/guidance/roi_type"
    ROI_ENABLE_TOPIC = "/guidance/roi_enable"

    UAV_POSE_TOPIC = "/mavros/vision_pose/pose"
    WORLD_POSITION_TOPIC = "/aruco/world_position"

    # Camera position relative to OptiTrack rigid body origin
    # x = forward, y = left, z = up
    CAMERA_OFFSET = np.array([
        0.10,
        0.00,
        -0.16
    ])

    # ArUco
    ARUCO_DICT = cv2.aruco.DICT_5X5_100
    MARKER_LENGTH = 0.200
    LANDING_FRAMES_REQUIRED = 2

    # HUD
    SHOW_CROSSHAIR = True
    SHOW_DISTANCE_TEXT = True
    SHOW_WORLD_AXES = True

    CROSSHAIR_SIZE = 24
    CROSSHAIR_GAP = 8


    def __init__(self):

        self.camera_matrix = None
        self.dist_coeffs = None

        self.detected_marker_log = {}

        self.uav_pose = None
        self.roi_enabled = False
        self.roi_triggered = False

        self.landing_detection_count = 0

        self.landing_aruco_id = int(
            rospy.get_param(
                "~landing_aruco_id",
                16
            )
        )

        self.br = CvBridge()

        # ArUco dictionary
        if hasattr(
            cv2.aruco,
            "getPredefinedDictionary"
        ):

            self.aruco_dict = (
                cv2.aruco.getPredefinedDictionary(
                    self.ARUCO_DICT
                )
            )

        else:

            self.aruco_dict = (
                cv2.aruco.Dictionary_get(
                    self.ARUCO_DICT
                )
            )

        # Use OpenCV's default ArUco parameters.
        # No additional tuning for now.
        if hasattr(
            cv2.aruco,
            "DetectorParameters_create"
        ):

            self.aruco_params = (
                cv2.aruco.DetectorParameters_create()
            )

        else:

            self.aruco_params = (
                cv2.aruco.DetectorParameters()
            )

        # Camera calibration
        self.camera_info_sub = rospy.Subscriber(
            self.CAMERA_INFO_TOPIC,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )

        # Synchronised raw + YOLO images.
        #
        # Both are stamped identically by
        # dai_yolo_publisher, so the ArUco detection
        # and displayed YOLO frame represent the same
        # camera frame.
        self.raw_frame_sub = (
            message_filters.Subscriber(
                self.FRAME_SUB_TOPIC,
                CompressedImage,
                queue_size=1
            )
        )

        self.yolo_frame_sub = (
            message_filters.Subscriber(
                self.YOLO_FRAME_TOPIC,
                CompressedImage,
                queue_size=1
            )
        )

        self.image_sync = (
            message_filters.TimeSynchronizer(
                [
                    self.raw_frame_sub,
                    self.yolo_frame_sub
                ],
                queue_size=2
            )
        )

        self.image_sync.registerCallback(
            self.vision_callback
        )

        # UAV pose
        self.uav_pose_sub = rospy.Subscriber(
            self.UAV_POSE_TOPIC,
            PoseStamped,
            self.uav_pose_callback,
            queue_size=1
        )

        # ROI enable
        self.roi_enable_sub = rospy.Subscriber(
            self.ROI_ENABLE_TOPIC,
            Bool,
            self.roi_enable_callback,
            queue_size=1
        )

        # Publishers
        self.aruco_pub = rospy.Publisher(
            self.OUTPUT_TOPIC,
            CompressedImage,
            queue_size=1
        )

        self.pose_log_pub = rospy.Publisher(
            self.POSE_LOG_TOPIC,
            String,
            queue_size=10
        )

        self.rviz_log_pub = rospy.Publisher(
            self.RVIZ_LOG_TOPIC,
            Marker,
            queue_size=1
        )

        self.roi_pub = rospy.Publisher(
            self.ROI_TOPIC,
            PoseStamped,
            queue_size=1
        )

        self.roi_type_pub = rospy.Publisher(
            self.ROI_TYPE_TOPIC,
            String,
            queue_size=1
        )

        self.world_position_pub = rospy.Publisher(
            self.WORLD_POSITION_TOPIC,
            PointStamped,
            queue_size=1
        )

        rospy.loginfo(
            "ArUco detector started"
        )

        rospy.loginfo(
            "Synchronising {} + {}".format(
                self.FRAME_SUB_TOPIC,
                self.YOLO_FRAME_TOPIC
            )
        )

        rospy.loginfo(
            "Landing ArUco ID: {}".format(
                self.landing_aruco_id
            )
        )


    def camera_info_callback(
        self,
        msg
    ):

        self.camera_matrix = np.array(
            msg.K,
            dtype=np.float64
        ).reshape(
            (3, 3)
        )

        self.dist_coeffs = np.array(
            msg.D,
            dtype=np.float64
        )

        rospy.loginfo_once(
            "Camera info received: {}x{}".format(
                msg.width,
                msg.height
            )
        )


    def uav_pose_callback(
        self,
        msg
    ):

        self.uav_pose = msg


    def roi_enable_callback(
        self,
        msg
    ):

        self.roi_enabled = msg.data

        if self.roi_enabled:

            rospy.loginfo(
                "ArUco ROI publication enabled"
            )


    def vision_callback(
        self,
        raw_msg,
        yolo_msg
    ):

        try:

            raw_frame = (
                self.br.compressed_imgmsg_to_cv2(
                    raw_msg
                )
            )

            yolo_frame = (
                self.br.compressed_imgmsg_to_cv2(
                    yolo_msg
                )
            )

        except CvBridgeError as e:

            rospy.logerr(e)

            return

        # Detect on the clean raw image, but draw
        # onto the synchronized YOLO image.
        self.find_aruco(
            raw_frame,
            yolo_frame
        )

        self.publish_to_ros(
            yolo_frame,
            yolo_msg.header.stamp
        )


    def camera_to_world(
        self,
        raw_x,
        raw_y,
        raw_z
    ):

        if self.uav_pose is None:
            return None

        pose = self.uav_pose.pose

        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

        _, _, yaw = euler_from_quaternion(
            quaternion
        )

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Camera orientation:
        # image top    = UAV +X
        # image bottom = UAV -X
        # image left   = UAV +Y
        # image right  = UAV -Y
        #
        # OpenCV:
        # raw_x = image right
        # raw_y = image down
        # raw_z = optical axis
        body_x = (
            -raw_y
            + self.CAMERA_OFFSET[0]
        )

        body_y = (
            -raw_x
            + self.CAMERA_OFFSET[1]
        )

        world_x = (
            pose.position.x
            + cos_yaw * body_x
            - sin_yaw * body_y
        )

        world_y = (
            pose.position.y
            + sin_yaw * body_x
            + cos_yaw * body_y
        )

        return np.array([
            world_x,
            world_y,
            0.0
        ])


    def publish_world_position(
        self,
        marker_world
    ):

        msg = PointStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.point.x = float(
            marker_world[0]
        )

        msg.point.y = float(
            marker_world[1]
        )

        msg.point.z = float(
            marker_world[2]
        )

        self.world_position_pub.publish(
            msg
        )


    def draw_crosshair(
        self,
        frame
    ):

        height, width = frame.shape[:2]

        centre_x = width // 2
        centre_y = height // 2

        colour = (0, 255, 255)

        cv2.line(
            frame,
            (
                centre_x - self.CROSSHAIR_SIZE,
                centre_y
            ),
            (
                centre_x - self.CROSSHAIR_GAP,
                centre_y
            ),
            colour,
            2
        )

        cv2.line(
            frame,
            (
                centre_x + self.CROSSHAIR_GAP,
                centre_y
            ),
            (
                centre_x + self.CROSSHAIR_SIZE,
                centre_y
            ),
            colour,
            2
        )

        cv2.line(
            frame,
            (
                centre_x,
                centre_y - self.CROSSHAIR_SIZE
            ),
            (
                centre_x,
                centre_y - self.CROSSHAIR_GAP
            ),
            colour,
            2
        )

        cv2.line(
            frame,
            (
                centre_x,
                centre_y + self.CROSSHAIR_GAP
            ),
            (
                centre_x,
                centre_y + self.CROSSHAIR_SIZE
            ),
            colour,
            2
        )


    def draw_world_axes(
        self,
        frame
    ):

        height, width = frame.shape[:2]

        # Larger compass for 960x540
        origin_x = width - 100
        origin_y = 100

        radius = 72
        axis_length = 50

        yellow = (0, 255, 255)

        overlay = frame.copy()

        cv2.circle(
            overlay,
            (origin_x, origin_y),
            radius,
            yellow,
            -1
        )

        cv2.addWeighted(
            overlay,
            0.10,
            frame,
            0.90,
            0,
            frame
        )

        cv2.circle(
            frame,
            (origin_x, origin_y),
            radius,
            yellow,
            2,
            cv2.LINE_AA
        )

        # +X top
        cv2.arrowedLine(
            frame,
            (origin_x, origin_y),
            (
                origin_x,
                origin_y - axis_length
            ),
            yellow,
            3,
            cv2.LINE_AA,
            tipLength=0.25
        )

        # +Y left
        cv2.arrowedLine(
            frame,
            (origin_x, origin_y),
            (
                origin_x - axis_length,
                origin_y
            ),
            yellow,
            3,
            cv2.LINE_AA,
            tipLength=0.25
        )

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.60
        thickness = 2

        cv2.putText(
            frame,
            "+X",
            (
                origin_x - 16,
                origin_y - axis_length - 10
            ),
            font,
            font_scale,
            yellow,
            thickness,
            cv2.LINE_AA
        )

        cv2.putText(
            frame,
            "-X",
            (
                origin_x - 16,
                origin_y + axis_length + 24
            ),
            font,
            font_scale,
            yellow,
            thickness,
            cv2.LINE_AA
        )

        cv2.putText(
            frame,
            "+Y",
            (
                origin_x - axis_length - 40,
                origin_y + 7
            ),
            font,
            font_scale,
            yellow,
            thickness,
            cv2.LINE_AA
        )

        cv2.putText(
            frame,
            "-Y",
            (
                origin_x + axis_length + 8,
                origin_y + 7
            ),
            font,
            font_scale,
            yellow,
            thickness,
            cv2.LINE_AA
        )

        cv2.circle(
            frame,
            (origin_x, origin_y),
            5,
            yellow,
            -1
        )


    def update_detection_log(
        self,
        marker_id,
        cX,
        cY,
        marker_world,
        distance
    ):

        if marker_world is None:
            return

        self.detected_marker_log[
            int(marker_id)
        ] = {
            "px": cX,
            "py": cY,
            "x": float(marker_world[0]),
            "y": float(marker_world[1]),
            "z": float(marker_world[2]),
            "distance": distance
        }

        log_lines = [
            "Seen ArUco markers:"
        ]

        for saved_id in sorted(
            self.detected_marker_log.keys()
        ):

            marker = (
                self.detected_marker_log[
                    saved_id
                ]
            )

            log_lines.append(
                "ID {}: pixel=({}, {}), "
                "world x={:.2f}, y={:.2f}, "
                "z={:.2f}, dist={:.2f}m".format(
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

        marker_msg = Marker()

        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = rospy.Time.now()

        marker_msg.ns = "aruco_detection_log"
        marker_msg.id = 0

        marker_msg.type = (
            Marker.TEXT_VIEW_FACING
        )

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

        if self.SHOW_WORLD_AXES:
            self.draw_world_axes(
                output_frame
            )

        # ArUco detection is performed on the clean,
        # synchronized raw frame.
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        if hasattr(
            cv2.aruco,
            "ArucoDetector"
        ):

            detector = (
                cv2.aruco.ArucoDetector(
                    self.aruco_dict,
                    self.aruco_params
                )
            )

            corners, ids, _ = (
                detector.detectMarkers(
                    gray
                )
            )

        else:

            corners, ids, _ = (
                cv2.aruco.detectMarkers(
                    gray,
                    self.aruco_dict,
                    parameters=self.aruco_params
                )
            )

        landing_seen_this_frame = False

        if ids is None or len(corners) == 0:

            self.landing_detection_count = 0

            return

        ids = ids.flatten()

        half = (
            self.MARKER_LENGTH / 2
        )

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
            ).astype(
                np.float32
            )

            pts_int = pts.astype(int)

            # Larger marker outline for 960x540
            cv2.polylines(
                output_frame,
                [pts_int],
                True,
                (0, 255, 0),
                3
            )

            cX = int(
                np.mean(
                    pts[:, 0]
                )
            )

            cY = int(
                np.mean(
                    pts[:, 1]
                )
            )

            cv2.circle(
                output_frame,
                (cX, cY),
                5,
                (0, 0, 255),
                -1
            )

            # Larger marker ID
            cv2.putText(
                output_frame,
                "ARUCO ID {}".format(
                    marker_id
                ),
                (
                    cX + 12,
                    cY - 12
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0),
                2,
                cv2.LINE_AA
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

            success, rvec, tvec = (
                cv2.solvePnP(
                    object_points,
                    pts,
                    self.camera_matrix,
                    self.dist_coeffs
                )
            )

            if not success:
                continue

            if hasattr(
                cv2,
                "drawFrameAxes"
            ):

                cv2.drawFrameAxes(
                    output_frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.MARKER_LENGTH * 0.5
                )

            raw_x = float(tvec[0])
            raw_y = float(tvec[1])
            raw_z = float(tvec[2])

            distance = float(
                np.linalg.norm(
                    tvec
                )
            )

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

            self.update_detection_log(
                marker_id,
                cX,
                cY,
                marker_world,
                distance
            )

            if (
                int(marker_id)
                == self.landing_aruco_id
            ):

                landing_seen_this_frame = True

                self.landing_detection_count += 1

                cv2.putText(
                    output_frame,
                    "LANDING {}/{}".format(
                        min(
                            self.landing_detection_count,
                            self.LANDING_FRAMES_REQUIRED
                        ),
                        self.LANDING_FRAMES_REQUIRED
                    ),
                    (
                        cX + 12,
                        cY + 72
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA
                )

                if (
                    self.landing_detection_count
                    >= self.LANDING_FRAMES_REQUIRED
                ):

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
                    (
                        cX + 12,
                        cY + 24
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA
                )

                cv2.putText(
                    output_frame,
                    "x:{:.2f} y:{:.2f} z:{:.2f}".format(
                        raw_x,
                        -raw_y,
                        raw_z
                    ),
                    (
                        cX + 12,
                        cY + 48
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA
                )

        if not landing_seen_this_frame:

            self.landing_detection_count = 0


    def publish_test_roi(
        self,
        marker_id,
        marker_world
    ):

        if not self.roi_enabled:
            return

        if self.roi_triggered:
            return

        roi_msg = PoseStamped()

        roi_msg.header.stamp = rospy.Time.now()
        roi_msg.header.frame_id = "map"

        roi_msg.pose.position.x = float(
            marker_world[0]
        )

        roi_msg.pose.position.y = float(
            marker_world[1]
        )

        roi_msg.pose.position.z = 2.0

        roi_msg.pose.orientation.w = 1.0

        self.roi_type_pub.publish(
            "A"
        )

        self.roi_pub.publish(
            roi_msg
        )

        self.roi_triggered = True

        rospy.logwarn(
            "ArUco ID {} detected - "
            "ROI diversion requested to "
            "({:.2f}, {:.2f}, 2.0)".format(
                marker_id,
                marker_world[0],
                marker_world[1]
            )
        )


    def publish_to_ros(
        self,
        frame,
        stamp
    ):

        success, encoded_image = cv2.imencode(
            ".jpg",
            frame,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                90
            ]
        )

        if not success:

            rospy.logwarn(
                "Failed to encode processed image"
            )

            return

        msg_out = CompressedImage()

        # Preserve synchronized source timestamp
        msg_out.header.stamp = stamp
        msg_out.header.frame_id = "oak_rgb_camera"

        msg_out.format = "jpeg"
        msg_out.data = encoded_image.tobytes()

        self.aruco_pub.publish(
            msg_out
        )


if __name__ == "__main__":

    rospy.init_node(
        "aruco_detector"
    )

    detector = ArucoDetector()

    rospy.spin()