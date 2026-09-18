#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import math

from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion


class ArucoDetector():

    # ROS Topics
    FRAME_SUB_TOPIC = "/depthai_node/image/compressed"
    YOLO_FRAME_TOPIC = "/yolo/image/compressed"
    CAMERA_INFO_TOPIC = "/depthai_node/camera/camera_info"

    OUTPUT_TOPIC = "/processed_vision/image/compressed"

    POSE_LOG_TOPIC = "/aruco/pose_log"
    RVIZ_LOG_TOPIC = "/aruco/detection_log_text"
    ROI_TOPIC = "/guidance/roi"
    ROI_TYPE_TOPIC = "/guidance/roi_type"
    ROI_ENABLE_TOPIC = "/guidance/roi_enable"

    # OptiTrack localisation
    UAV_POSE_TOPIC = "/mavros/vision_pose/pose"
    WORLD_POSITION_TOPIC = "/aruco/world_position"

    # Camera position relative to OptiTrack rigid body origin
    # x = forward, y = left, z = up
    CAMERA_OFFSET = np.array([
        0.10,      # Camera is 10 cm in front
        0.00,
        -0.16      # Camera is 16 cm below
    ])

    # ArUco setup
    ARUCO_DICT = cv2.aruco.DICT_5X5_100
    MARKER_LENGTH = 0.200  # metres

    # Require several detections before using the landing marker
    # This lets us loosen ArUco detection without making landing unsafe
    LANDING_FRAMES_REQUIRED = 3

    # HUD setup
    SHOW_CROSSHAIR = True
    SHOW_DISTANCE_TEXT = True
    SHOW_WORLD_AXES = True

    CROSSHAIR_SIZE = 18
    CROSSHAIR_GAP = 6


    def __init__(self):

        self.camera_matrix = None
        self.dist_coeffs = None

        self.detected_marker_log = {}

        self.uav_pose = None

        # ROI publication starts disabled
        self.roi_enabled = False

        # ArUco marker used as landing target
        self.landing_aruco_id = int(
            rospy.get_param(
                "~landing_aruco_id",
                6
            )
        )

        # Number of consecutive frames containing the landing marker
        self.landing_detection_count = 0

        # Latest raw and YOLO images
        self.latest_raw_frame = None

        self.br = CvBridge()

        # ArUco detector setup
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(
                self.ARUCO_DICT
            )
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(
                self.ARUCO_DICT
            )

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # -------------------------------------------------------------
        # More permissive ArUco detector settings
        # -------------------------------------------------------------

        # Search across a much wider range of local threshold sizes.
        # Helps with uneven lighting, distance and partial shadows.
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 101
        self.aruco_params.adaptiveThreshWinSizeStep = 4

        # Allow much smaller markers in the frame.
        # Default/current value was 0.015.
        self.aruco_params.minMarkerPerimeterRate = 0.008

        # Allow very large markers too.
        self.aruco_params.maxMarkerPerimeterRate = 6.0

        # Permit somewhat less-perfect quadrilateral shapes.
        # Useful when viewed at an angle or during UAV motion.
        self.aruco_params.polygonalApproxAccuracyRate = 0.05

        # Allow candidate corners to be closer together.
        if hasattr(
            self.aruco_params,
            "minCornerDistanceRate"
        ):
            self.aruco_params.minCornerDistanceRate = 0.02

        # Allow markers closer to the edge of the image.
        if hasattr(
            self.aruco_params,
            "minDistanceToBorder"
        ):
            self.aruco_params.minDistanceToBorder = 1

        # Allow nearby marker candidates.
        if hasattr(
            self.aruco_params,
            "minMarkerDistanceRate"
        ):
            self.aruco_params.minMarkerDistanceRate = 0.01

        # Subpixel refinement improves corner accuracy after detection.
        self.aruco_params.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
        )

        if hasattr(
            self.aruco_params,
            "cornerRefinementWinSize"
        ):
            self.aruco_params.cornerRefinementWinSize = 7

        if hasattr(
            self.aruco_params,
            "cornerRefinementMaxIterations"
        ):
            self.aruco_params.cornerRefinementMaxIterations = 50

        if hasattr(
            self.aruco_params,
            "cornerRefinementMinAccuracy"
        ):
            self.aruco_params.cornerRefinementMinAccuracy = 0.05

        # Use more pixels when decoding each marker cell.
        if hasattr(
            self.aruco_params,
            "perspectiveRemovePixelPerCell"
        ):
            self.aruco_params.perspectiveRemovePixelPerCell = 8

        # Use slightly more of each cell during decoding.
        if hasattr(
            self.aruco_params,
            "perspectiveRemoveIgnoredMarginPerCell"
        ):
            self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.05

        # Permit a noisier black border.
        if hasattr(
            self.aruco_params,
            "maxErroneousBitsInBorderRate"
        ):
            self.aruco_params.maxErroneousBitsInBorderRate = 0.45

        # Allow more dictionary error correction.
        if hasattr(
            self.aruco_params,
            "errorCorrectionRate"
        ):
            self.aruco_params.errorCorrectionRate = 0.80

        # Detect markers with reversed black/white polarity too.
        if hasattr(
            self.aruco_params,
            "detectInvertedMarker"
        ):
            self.aruco_params.detectInvertedMarker = True

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

        # Listen for Guidance permission to publish ROI commands
        self.roi_enable_sub = rospy.Subscriber(
            self.ROI_ENABLE_TOPIC,
            Bool,
            self.roi_enable_callback
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

        # ROI target type
        self.roi_type_pub = rospy.Publisher(
            self.ROI_TYPE_TOPIC,
            String,
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
        rospy.loginfo("Raw input: {}".format(self.FRAME_SUB_TOPIC))
        rospy.loginfo("YOLO input: {}".format(self.YOLO_FRAME_TOPIC))
        rospy.loginfo("Output: {}".format(self.OUTPUT_TOPIC))
        rospy.loginfo(
            "Landing ArUco ID: {}".format(
                self.landing_aruco_id
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


    def roi_enable_callback(self, msg):

        self.roi_enabled = msg.data

        if self.roi_enabled:
            rospy.loginfo(
                "ArUco ROI publication enabled"
            )


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

        # image top    = UAV +X
        # image bottom = UAV -X
        # image left   = UAV +Y
        # image right  = UAV -Y
        #
        # OpenCV camera coordinates:
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

        marker_world = np.array([
            world_x,
            world_y,
            0.0
        ])

        return marker_world


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


    def draw_crosshair(self, frame):

        height, width = frame.shape[:2]

        centre_x = width // 2
        centre_y = height // 2

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
            (0, 255, 255),
            1
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
            (0, 255, 255),
            1
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
            (0, 255, 255),
            1
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
            (0, 255, 255),
            1
        )


    def draw_world_axes(self, frame):

        height, width = frame.shape[:2]

        origin_x = width - 70
        origin_y = 70
        axis_length = 35

        yellow = (0, 255, 255)

        # Transparent compass background
        overlay = frame.copy()

        cv2.circle(
            overlay,
            (origin_x, origin_y),
            50,
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

        # Compass outline
        cv2.circle(
            frame,
            (origin_x, origin_y),
            50,
            yellow,
            1,
            cv2.LINE_AA
        )

        # +X = top
        cv2.arrowedLine(
            frame,
            (origin_x, origin_y),
            (
                origin_x,
                origin_y - axis_length
            ),
            yellow,
            2,
            cv2.LINE_AA,
            tipLength=0.25
        )

        cv2.putText(
            frame,
            "+X",
            (
                origin_x - 10,
                origin_y - axis_length - 6
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            yellow,
            1,
            cv2.LINE_AA
        )

        # +Y = left
        cv2.arrowedLine(
            frame,
            (origin_x, origin_y),
            (
                origin_x - axis_length,
                origin_y
            ),
            yellow,
            2,
            cv2.LINE_AA,
            tipLength=0.25
        )

        cv2.putText(
            frame,
            "+Y",
            (
                origin_x - axis_length - 23,
                origin_y + 4
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            yellow,
            1,
            cv2.LINE_AA
        )

        # -Y = right
        cv2.putText(
            frame,
            "-Y",
            (
                origin_x + axis_length + 4,
                origin_y + 4
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            yellow,
            1,
            cv2.LINE_AA
        )

        # -X = bottom
        cv2.putText(
            frame,
            "-X",
            (
                origin_x - 10,
                origin_y + axis_length + 15
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            yellow,
            1,
            cv2.LINE_AA
        )

        # Centre
        cv2.circle(
            frame,
            (origin_x, origin_y),
            3,
            yellow,
            -1,
            cv2.LINE_AA
        )

        cv2.putText(
            frame,
            "WORLD",
            (
                origin_x - 10,
                origin_y + 65
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            yellow,
            1,
            cv2.LINE_AA
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
                "world x={:.2f}, y={:.2f}, z={:.2f}, "
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

        marker_msg = Marker()

        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = (
            rospy.Time.now()
        )

        marker_msg.ns = (
            "aruco_detection_log"
        )

        marker_msg.id = 0

        marker_msg.type = (
            Marker.TEXT_VIEW_FACING
        )

        marker_msg.action = (
            Marker.ADD
        )

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

        # Detection is performed on the clean camera image
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

        # Keep track of whether the landing marker
        # is visible in this particular frame
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
            ).astype(np.float32)

            pts_int = pts.astype(int)

            cv2.polylines(
                output_frame,
                [pts_int],
                True,
                (0, 255, 0),
                2
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

            output_cX = cX
            output_cY = cY

            cv2.circle(
                output_frame,
                (
                    output_cX,
                    output_cY
                ),
                4,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                output_frame,
                "ID: {}".format(
                    marker_id
                ),
                (
                    output_cX + 8,
                    output_cY - 8
                ),
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
                    camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.MARKER_LENGTH * 0.5
                )

            raw_x = float(
                tvec[0]
            )

            raw_y = float(
                tvec[1]
            )

            raw_z = float(
                tvec[2]
            )

            distance = float(
                np.linalg.norm(
                    tvec
                )
            )

            # Display camera-relative pose
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
                marker_world,
                distance
            )

            # Landing marker needs several consecutive detections
            if (
                int(marker_id)
                == self.landing_aruco_id
            ):

                landing_seen_this_frame = True

                self.landing_detection_count += 1

                cv2.putText(
                    output_frame,
                    "Landing confirm: {}/{}".format(
                        min(
                            self.landing_detection_count,
                            self.LANDING_FRAMES_REQUIRED
                        ),
                        self.LANDING_FRAMES_REQUIRED
                    ),
                    (
                        output_cX + 8,
                        output_cY + 49
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (0, 255, 255),
                    1
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

                    else:

                        self.publish_test_roi(
                            marker_id,
                            [-2.0, 1.0, 2.0]
                        )

            if self.SHOW_DISTANCE_TEXT:

                cv2.putText(
                    output_frame,
                    "Dist: {:.2f} m".format(
                        distance
                    ),
                    (
                        output_cX + 8,
                        output_cY + 15
                    ),
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
                    (
                        output_cX + 8,
                        output_cY + 32
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (0, 255, 255),
                    1
                )

        # If the landing marker disappears, restart confirmation
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

        roi_msg.header.stamp = (
            rospy.Time.now()
        )

        roi_msg.header.frame_id = "map"

        roi_msg.pose.position.x = float(
            marker_world[0]
        )

        roi_msg.pose.position.y = float(
            marker_world[1]
        )

        roi_msg.pose.position.z = 2.0

        roi_msg.pose.orientation.x = 0.0
        roi_msg.pose.orientation.y = 0.0
        roi_msg.pose.orientation.z = 0.0
        roi_msg.pose.orientation.w = 1.0

        self.roi_type_pub.publish(
            "A"
        )

        self.roi_pub.publish(
            roi_msg
        )

        self.roi_triggered = True

        rospy.logwarn(
            "ArUco ID {} detected - ROI diversion requested "
            "to ({:.2f}, {:.2f}, 2.0)".format(
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

        msg_out.header.stamp = (
            rospy.Time.now()
        )

        msg_out.header.frame_id = (
            "oak_rgb_camera"
        )

        msg_out.format = "jpeg"

        success, encoded_image = (
            cv2.imencode(
                ".jpg",
                frame
            )
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

    rospy.init_node(
        "aruco_detector"
    )

    detector = ArucoDetector()

    rospy.spin()