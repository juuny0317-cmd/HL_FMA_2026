"""Section-gated traffic-light and pedestrian YOLO perception.

The integrated seven-class weight is retained as supplied, but inference is
restricted to traffic classes in TRAFFIC and pedestrian in DUMMY.  The model's
S-obstacle and end-point classes are never requested or published.
"""

from __future__ import annotations

import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String, UInt8

from hl_ku_interfaces.msg import PedestrianFrame

from .mission import LIGHT_UNKNOWN
from .pedestrian import pedestrian_evidence, pedestrian_stop_candidates
from .traffic_light import (
    Detection,
    INTEGRATED_YOLO_NAMES,
    normalize_model_names,
    ros_light_for_class_name,
    select_target_detection,
    traffic_signal_mask,
    valid_traffic_detections,
)
from .yolo_gate import class_ids_for_mode, mode_for_route_zone


class YoloPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_perception")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("traffic_confidence_threshold", 0.50)
        self.declare_parameter("pedestrian_confidence_threshold", 0.50)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "")
        self.declare_parameter("traffic_roi", [0.20, 0.00, 0.80, 0.60])
        self.declare_parameter("pedestrian_roi", [0.00, 0.00, 1.00, 1.00])
        self.declare_parameter("pedestrian_minimum_box_height_ratio", 0.50)
        self.declare_parameter("traffic_minimum_box_area_px2", 36.0)
        self.declare_parameter("edge_margin_ratio", 0.02)
        self.declare_parameter("traffic_target_point", [0.50, 0.20])
        self.declare_parameter("maximum_target_distance_ratio", 0.45)
        self.declare_parameter("maximum_processing_fps", 10.0)
        self.declare_parameter("debug_image_enabled", True)

        self._bridge = CvBridge()
        self._model = None
        self._names: tuple[str, ...] = ()
        self._mode = "OFF"
        self._last_processing_sec: float | None = None
        self._last_target_center: tuple[float, float] | None = None

        self._light_pub = self.create_publisher(UInt8, "/perception/traffic_light", 10)
        self._light_conf_pub = self.create_publisher(
            Float32, "/perception/traffic_confidence", 10
        )
        self._traffic_mask_pub = self.create_publisher(
            UInt8, "/perception/traffic_signal_mask", 10
        )
        self._pedestrian_pub = self.create_publisher(
            PedestrianFrame, "/perception/pedestrian_frame", 10
        )
        self._active_pub = self.create_publisher(
            Bool, "/perception/yolo_active", 10
        )
        self._mode_pub = self.create_publisher(
            String, "/perception/yolo_mode", 10
        )
        self._inference_pub = self.create_publisher(
            Float32, "/perception/yolo_inference_ms", 10
        )
        self._overlay_pub = (
            self.create_publisher(Image, "/perception/yolo_overlay", 2)
            if bool(self.get_parameter("debug_image_enabled").value)
            else None
        )

        self._load_model()
        self.create_subscription(
            String, "/planning/route_zone", self._on_route_zone, 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_timer(0.10, self._publish_status)

    @staticmethod
    def _message(message_type, value):
        message = message_type()
        message.data = value
        return message

    def _load_model(self) -> None:
        path = str(self.get_parameter("model_path").value).strip()
        if not path:
            self.get_logger().error("integrated YOLO model_path is empty")
            return
        try:
            from ultralytics import YOLO

            model = YOLO(path)
            names = normalize_model_names(model.names, INTEGRATED_YOLO_NAMES)
        except Exception as error:
            self.get_logger().error(f"failed to load integrated YOLO: {error}")
            return
        self._model = model
        self._names = names
        self.get_logger().info(
            "integrated YOLO loaded; runtime classes are traffic and pedestrian only: "
            f"{path}"
        )

    def _on_route_zone(self, message: String) -> None:
        mode = mode_for_route_zone(message.data)
        if mode != self._mode:
            self.get_logger().info(f"YOLO mode {self._mode} -> {mode}")
            self._mode = mode
            self._last_target_center = None

    def _publish_traffic(self, light: int, confidence: float) -> None:
        self._light_pub.publish(self._message(UInt8, int(light)))
        self._light_conf_pub.publish(self._message(Float32, float(confidence)))

    def _publish_status(self) -> None:
        active = self._model is not None and self._mode != "OFF"
        self._active_pub.publish(self._message(Bool, active))
        self._mode_pub.publish(self._message(String, self._mode))
        if not active:
            # Keep the perception aggregate fresh without creating a fake
            # inference frame for the traffic voter.
            self._publish_traffic(LIGHT_UNKNOWN, 0.0)

    def _pedestrian_detections(
        self,
        detections: list[Detection],
        width: int,
        height: int,
    ) -> list[Detection]:
        return list(
            pedestrian_stop_candidates(
                detections,
                width,
                height,
                self.get_parameter("pedestrian_roi").value,
                float(
                    self.get_parameter(
                        "pedestrian_minimum_box_height_ratio"
                    ).value
                ),
            )
        )

    def _on_image(self, message: Image) -> None:
        if self._model is None or self._mode == "OFF":
            return
        now_sec = time.monotonic()
        maximum_fps = float(self.get_parameter("maximum_processing_fps").value)
        if (
            maximum_fps > 0.0
            and self._last_processing_sec is not None
            and now_sec - self._last_processing_sec < 1.0 / maximum_fps
        ):
            return
        self._last_processing_sec = now_sec
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            started = time.monotonic()
            traffic_threshold = float(
                self.get_parameter("traffic_confidence_threshold").value
            )
            pedestrian_threshold = float(
                self.get_parameter("pedestrian_confidence_threshold").value
            )
            result = self._model.predict(
                source=image,
                conf=min(traffic_threshold, pedestrian_threshold),
                iou=float(self.get_parameter("iou_threshold").value),
                device=str(self.get_parameter("device").value).strip() or None,
                classes=class_ids_for_mode(self._names, self._mode),
                verbose=False,
            )[0]
            inference_ms = (time.monotonic() - started) * 1000.0
            boxes = result.boxes
            xyxy = boxes.xyxy.cpu().tolist() if boxes is not None else []
            confidences = boxes.conf.cpu().tolist() if boxes is not None else []
            class_ids = boxes.cls.cpu().tolist() if boxes is not None else []
            detections = [
                Detection(
                    class_name=self._names[int(class_id)],
                    confidence=float(confidence),
                    x1=float(coordinates[0]),
                    y1=float(coordinates[1]),
                    x2=float(coordinates[2]),
                    y2=float(coordinates[3]),
                )
                for coordinates, confidence, class_id in zip(
                    xyxy, confidences, class_ids
                )
                if 0 <= int(class_id) < len(self._names)
            ]
            height, width = image.shape[:2]
            selected = None
            pedestrian_detected = False
            pedestrian_confidence = 0.0

            if self._mode == "TRAFFIC":
                preferred = self._last_target_center or tuple(
                    float(value)
                    for value in self.get_parameter("traffic_target_point").value
                )
                valid = valid_traffic_detections(
                    detections,
                    width,
                    height,
                    self.get_parameter("traffic_roi").value,
                    float(self.get_parameter("traffic_minimum_box_area_px2").value),
                    float(self.get_parameter("edge_margin_ratio").value),
                    preferred,
                    float(
                        self.get_parameter("maximum_target_distance_ratio").value
                    ),
                )
                self._traffic_mask_pub.publish(
                    self._message(UInt8, traffic_signal_mask(valid, traffic_threshold))
                )
                selected = select_target_detection(
                    detections,
                    width,
                    height,
                    self.get_parameter("traffic_roi").value,
                    float(self.get_parameter("traffic_minimum_box_area_px2").value),
                    float(self.get_parameter("edge_margin_ratio").value),
                    preferred,
                    float(
                        self.get_parameter("maximum_target_distance_ratio").value
                    ),
                )
                if selected is None:
                    self._last_target_center = None
                    self._publish_traffic(LIGHT_UNKNOWN, 0.0)
                else:
                    light = ros_light_for_class_name(
                        selected.class_name,
                        selected.confidence,
                        traffic_threshold,
                    )
                    self._publish_traffic(
                        light,
                        selected.confidence if light != LIGHT_UNKNOWN else 0.0,
                    )
                    center_x, center_y = selected.center
                    self._last_target_center = (center_x / width, center_y / height)
            elif self._mode == "PEDESTRIAN":
                self._publish_traffic(LIGHT_UNKNOWN, 0.0)
                pedestrians = self._pedestrian_detections(
                    detections, width, height
                )
                pedestrian_detected, pedestrian_confidence = pedestrian_evidence(
                    pedestrians, pedestrian_threshold
                )
                frame = PedestrianFrame()
                frame.header = message.header
                frame.detected = pedestrian_detected
                frame.max_confidence = float(pedestrian_confidence)
                self._pedestrian_pub.publish(frame)

            self._inference_pub.publish(self._message(Float32, inference_ms))
            if self._overlay_pub is not None:
                self._publish_overlay(
                    image,
                    detections,
                    selected,
                    pedestrian_detected,
                    message,
                )
        except Exception as error:
            self.get_logger().error(f"integrated YOLO inference failed: {error}")
            self._publish_traffic(LIGHT_UNKNOWN, 0.0)

    def _publish_overlay(
        self,
        image,
        detections: list[Detection],
        selected: Detection | None,
        pedestrian_detected: bool,
        source: Image,
    ) -> None:
        debug = image.copy()
        colors = {
            "green": (0, 255, 0),
            "left": (0, 255, 255),
            "red": (0, 0, 255),
            "yellow": (0, 215, 255),
            "pedestrian": (255, 0, 255),
        }
        for detection in detections:
            color = colors.get(detection.class_name, (180, 180, 180))
            p1 = (int(detection.x1), int(detection.y1))
            p2 = (int(detection.x2), int(detection.y2))
            cv2.rectangle(debug, p1, p2, color, 3 if detection is selected else 2)
            cv2.putText(
                debug,
                f"{detection.class_name} {detection.confidence:.2f}",
                (p1[0], max(18, p1[1] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
        label = self._mode
        if self._mode == "PEDESTRIAN":
            label += " STOP" if pedestrian_detected else " CLEAR"
        cv2.putText(
            debug, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
            (255, 255, 255), 2,
        )
        output = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        output.header = source.header
        self._overlay_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
