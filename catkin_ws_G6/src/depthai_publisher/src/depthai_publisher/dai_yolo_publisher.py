#!/usr/bin/env python3

import json
import time
import cv2
import depthai as dai
import rospy

from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String

# This is the detector for backapck and person's

class YoloDetector():
    # These are top level settings that are easy to adjust

    # ROS topics
    RAW_TOPIC = "/depthai_node/image/compressed"
    OUTPUT_TOPIC = "/yolo/image/compressed"
    DETECTION_TOPIC = "/yolo/detections"
    CAMERA_INFO_TOPIC = "/depthai_node/camera/camera_info"

    # YOLO settings
    CONFIDENCE = 0.70
    IOU = 0.50
    CAMERA_FPS = 15.0
    JPEG_QUALITY = 85


    def __init__(self):
        # Model files are set in the launch file
        self.blob_path = rospy.get_param("~blob_path")
        self.config_path = rospy.get_param("~config_path")

        # Allow the main settings to also be changed from the launch file
        self.confidence = float(
            rospy.get_param("~confidence", self.CONFIDENCE)
        )
        self.iou = float(
            rospy.get_param("~iou", self.IOU)
        )
        self.camera_fps = float(
            rospy.get_param("~camera_fps", self.CAMERA_FPS)
        )
        self.jpeg_quality = int(
            rospy.get_param("~jpeg_quality", self.JPEG_QUALITY)
        )

        # ROS topics can also be changed from the launch file
        self.raw_topic = rospy.get_param("~raw_topic", self.RAW_TOPIC)
        self.output_topic = rospy.get_param(
            "~annotated_topic", self.OUTPUT_TOPIC
        )
        self.detection_topic = rospy.get_param(
            "~detection_topic", self.DETECTION_TOPIC
        )

        # Read model config
        with open(self.config_path, "r") as config_file:
            self.config = json.load(config_file)

        self.labels = self.get_labels()
        self.input_width, self.input_height = self.get_input_size()

        # Raw OAK-D image output
        self.raw_pub = rospy.Publisher(
            self.raw_topic,
            CompressedImage,
            queue_size=1
        )

        # YOLO processed image output
        self.yolo_pub = rospy.Publisher(
            self.output_topic,
            CompressedImage,
            queue_size=1
        )

        # Text output of current detections
        self.detection_pub = rospy.Publisher(
            self.detection_topic,
            String,
            queue_size=10
        )

        # Camera calibration output
        self.camera_info_pub = rospy.Publisher(
            self.CAMERA_INFO_TOPIC,
            CameraInfo,
            queue_size=1
        )

        # Build the OAK-D pipeline
        self.pipeline = self.create_pipeline()

        rospy.loginfo("YOLO detector started")
        rospy.loginfo("Model: {}".format(self.blob_path))
        rospy.loginfo(
            "Input size: {}x{}".format(
                self.input_width,
                self.input_height
            )
        )
        rospy.loginfo("Classes: {}".format(", ".join(self.labels)))


    def get_labels(self):
        # New Luxonis config format
        labels = self.config.get("mappings", {}).get("labels", [])

        if labels:
            return labels

        # Older Luxonis config format
        heads = self.config.get("model", {}).get("heads", [])

        for head in heads:
            labels = head.get("metadata", {}).get("classes", [])

            if labels:
                return labels

        raise ValueError("No YOLO class labels found")


    def get_input_size(self):
        # New Luxonis config format - for example 416x416
        size = self.config.get("nn_config", {}).get("input_size")

        if size:
            width, height = size.split("x")
            return int(width), int(height)

        # Older Luxonis config format
        inputs = self.config.get("model", {}).get("inputs", [])

        if inputs:
            shape = inputs[0].get("shape", [])

            # Shape is NCHW
            if len(shape) == 4:
                return int(shape[3]), int(shape[2])

        raise ValueError("Could not find YOLO input size")


    def create_pipeline(self):
        # Main DepthAI pipeline
        pipeline = dai.Pipeline()

        # OAK-D RGB camera
        camera = pipeline.create(dai.node.ColorCamera)
        camera.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P
        )
        camera.setPreviewSize(
            self.input_width,
            self.input_height
        )
        camera.setPreviewKeepAspectRatio(False)
        camera.setInterleaved(False)
        camera.setColorOrder(
            dai.ColorCameraProperties.ColorOrder.BGR
        )
        camera.setFps(self.camera_fps)

        # YOLO runs directly on the OAK-D Myriad X
        yolo = pipeline.create(dai.node.YoloDetectionNetwork)
        yolo.setBlobPath(self.blob_path)
        yolo.setConfidenceThreshold(self.confidence)
        yolo.setIouThreshold(self.iou)
        yolo.setNumClasses(len(self.labels))
        yolo.setCoordinateSize(4)
        yolo.setNumInferenceThreads(2)

        yolo.input.setBlocking(False)
        yolo.input.setQueueSize(1)

        camera.preview.link(yolo.input)

        # Send camera image back to the Pi
        image_output = pipeline.create(dai.node.XLinkOut)
        image_output.setStreamName("frames")
        yolo.passthrough.link(image_output.input)

        # Send YOLO detections back to the Pi
        detection_output = pipeline.create(dai.node.XLinkOut)
        detection_output.setStreamName("detections")
        yolo.out.link(detection_output.input)

        return pipeline


    def draw_detections(self, frame, detections):
        # Draw YOLO detections onto the image
        height, width = frame.shape[:2]
        detection_log = []

        for detection in detections:
            x1 = int(detection.xmin * width)
            y1 = int(detection.ymin * height)
            x2 = int(detection.xmax * width)
            y2 = int(detection.ymax * height)

            class_id = int(detection.label)
            confidence = float(detection.confidence)

            if class_id < len(self.labels):
                label = self.labels[class_id]
            else:
                label = "Unknown"

            # Centre of detected object
            cX = (x1 + x2) // 2
            cY = (y1 + y2) // 2

            # Save detection as text for ROS
            detection_log.append(
                "{}:{:.2f}@({},{})".format(
                    label,
                    confidence,
                    cX,
                    cY
                )
            )

            # Detection box
            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2
            )

            # Detection centre point
            cv2.circle(
                frame,
                (cX, cY),
                4,
                (0, 255, 255),
                -1
            )

            # Class and confidence
            cv2.putText(
                frame,
                "{} {:.2f}".format(label, confidence),
                (x1, max(y1 - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2
            )

        return detection_log


    def publish_image(self, publisher, frame):
        # Convert OpenCV image to ROS CompressedImage
        success, encoded_image = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )

        if not success:
            rospy.logwarn("Failed to encode YOLO image")
            return

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "oak_rgb_camera"
        msg.format = "jpeg"
        msg.data = encoded_image.tobytes()

        publisher.publish(msg)


    def run(self):
        # Start the DepthAI pipeline on the OAK-D
        with dai.Device(self.pipeline) as device:

            # Read RGB camera calibration for the ArUco detector
            calibration = device.readCalibration()

            intrinsics = calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A,
                640,
                480
            )

            distortion = calibration.getDistortionCoefficients(
                dai.CameraBoardSocket.CAM_A
            )

            camera_info = CameraInfo()
            camera_info.width = 640
            camera_info.height = 480
            camera_info.distortion_model = "plumb_bob"
            camera_info.D = list(distortion)
            camera_info.K = [
                intrinsics[0][0], intrinsics[0][1], intrinsics[0][2],
                intrinsics[1][0], intrinsics[1][1], intrinsics[1][2],
                intrinsics[2][0], intrinsics[2][1], intrinsics[2][2]
            ]

            frame_queue = device.getOutputQueue(
                "frames",
                maxSize=1,
                blocking=False
            )

            detection_queue = device.getOutputQueue(
                "detections",
                maxSize=1,
                blocking=False
            )

            rospy.loginfo(
                "OAK-D connected - YOLO running on Myriad X"
            )

            last_time = time.time()
            fps = 0.0

            while not rospy.is_shutdown():
                frame = frame_queue.get().getCvFrame()
                detections = detection_queue.get().detections

                processed_frame = frame.copy()

                detection_log = self.draw_detections(
                    processed_frame,
                    detections
                )

                # Calculate current processing FPS
                current_time = time.time()
                frame_time = current_time - last_time
                last_time = current_time

                if frame_time > 0:
                    fps = 1.0 / frame_time

                cv2.putText(
                    processed_frame,
                    "Detections: {} | FPS: {:.1f}".format(
                        len(detection_log),
                        fps
                    ),
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                # Publish raw and processed camera images
                self.publish_image(self.raw_pub, frame)
                self.publish_image(self.yolo_pub, processed_frame)

                # Publish camera calibration for ArUco pose estimation
                camera_info.header.stamp = rospy.Time.now()
                camera_info.header.frame_id = "oak_rgb_camera"
                self.camera_info_pub.publish(camera_info)

                # Publish detection text
                detection_text = "; ".join(detection_log)
                self.detection_pub.publish(detection_text)

                if detection_log:
                    rospy.loginfo_throttle(
                        1.0,
                        "YOLO detections: {}".format(detection_text)
                    )


if __name__ == "__main__":
    rospy.init_node("dai_yolo_publisher")

    try:
        detector = YoloDetector()
        detector.run()

    except Exception as e:
        rospy.logerr("YOLO detector error: {}".format(e))