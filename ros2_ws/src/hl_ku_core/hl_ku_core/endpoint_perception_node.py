"""Two-class end-lane detector with default-F1 and one-way F2 latch."""

from __future__ import annotations

import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int8, String, UInt8

from .endpoint_selection import (
    EndpointDetection,
    EndpointRouteLatch,
    endpoint_route_candidate,
    normalize_endpoint_names,
)


class EndpointPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("endpoint_perception")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "")
        self.declare_parameter("maximum_processing_fps", 10.0)
        self.declare_parameter("debug_image_enabled", True)
        self.declare_parameter("activation_start_s_m", 643.7623)
        self.declare_parameter("activation_end_s_m", 696.5)

        self._bridge = CvBridge()
        self._model = None
        self._names: tuple[str, ...] = ()
        self._zone = "NORMAL"
        self._route_s = 0.0
        self._last_route_s: float | None = None
        self._last_processing_sec: float | None = None
        self._latch = EndpointRouteLatch()

        self._selection_pub = self.create_publisher(
            String, "/mission/end_lane_selection", 10
        )
        self._allowed_lane_pub = self.create_publisher(
            Int8, "/perception/allowed_lane", 10
        )
        self._signal_pub = self.create_publisher(
            UInt8, "/perception/end_lane_signal", 10
        )
        self._confidence_pub = self.create_publisher(
            Float32, "/perception/end_lane_confidence", 10
        )
        self._overlay_pub = (
            self.create_publisher(Image, "/perception/yolo_overlay", 2)
            if bool(self.get_parameter("debug_image_enabled").value)
            else None
        )

        self._load_model()
        self.create_subscription(
            String, "/planning/route_zone", self._on_zone, 10
        )
        self.create_subscription(
            Float32, "/planning/route_s", self._on_route_s, 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_timer(0.10, self._publish_selection)

    @staticmethod
    def _message(message_type, value):
        message = message_type()
        message.data = value
        return message

    def _load_model(self) -> None:
        path = str(self.get_parameter("model_path").value).strip()
        if not path:
            self.get_logger().error("endpoint YOLO model_path is empty")
            return
        try:
            from ultralytics import YOLO

            model = YOLO(path)
            names = normalize_endpoint_names(model.names)
        except Exception as error:
            self.get_logger().error(f"failed to load endpoint YOLO: {error}")
            return
        self._model = model
        self._names = names
        self.get_logger().info(f"endpoint_2class_v3 loaded: {path}")

    def _on_zone(self, message: String) -> None:
        self._zone = message.data.strip().upper()

    def _on_route_s(self, message: Float32) -> None:
        route_s = float(message.data)
        if self._last_route_s is not None and route_s + 5.0 < self._last_route_s:
            self._latch.reset()
            self.get_logger().info("new route run detected; endpoint selection reset to F1")
        self._last_route_s = route_s
        self._route_s = route_s

    def _publish_selection(self) -> None:
        active = self._active()
        selected = self._latch.selected_final
        message = {
            "mission": "END_LANE",
            "selected_final": selected,
            "selected_route": f"F{selected}",
            "locked": self._latch.locked,
            "decision": self._latch.last_decision,
            "confidence": self._latch.confidence,
            "active": active,
            "route_s_m": self._route_s,
        }
        self._selection_pub.publish(
            self._message(String, json.dumps(message, separators=(",", ":")))
        )
        if active:
            # The existing perception contract uses -1/+1 for the two lanes.
            # Path geometry, rather than this sign, performs the actual change.
            self._allowed_lane_pub.publish(
                self._message(Int8, -1 if selected == 1 else 1)
            )
            self._signal_pub.publish(self._message(UInt8, 1))
            self._confidence_pub.publish(
                self._message(Float32, float(self._latch.confidence))
            )
        else:
            self._allowed_lane_pub.publish(self._message(Int8, 0))
            self._signal_pub.publish(self._message(UInt8, 0))
            self._confidence_pub.publish(self._message(Float32, 0.0))

    def _active(self) -> bool:
        start_s = float(self.get_parameter("activation_start_s_m").value)
        end_s = float(self.get_parameter("activation_end_s_m").value)
        return self._zone == "END_LANE" or start_s <= self._route_s <= end_s

    def _on_image(self, message: Image) -> None:
        if self._model is None or not self._active():
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
            result = self._model.predict(
                source=image,
                conf=float(self.get_parameter("confidence_threshold").value),
                iou=float(self.get_parameter("iou_threshold").value),
                device=str(self.get_parameter("device").value).strip() or None,
                verbose=False,
            )[0]
            boxes = result.boxes
            xyxy = boxes.xyxy.cpu().tolist() if boxes is not None else []
            confidences = boxes.conf.cpu().tolist() if boxes is not None else []
            class_ids = boxes.cls.cpu().tolist() if boxes is not None else []
            detections = [
                EndpointDetection(
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
            decision = endpoint_route_candidate(detections, image.shape[0])
            if self._latch.update(decision):
                self.get_logger().info(
                    "endpoint ROUTE_2 latched: red signal is left of green arrow"
                )
            self._publish_selection()
            if self._overlay_pub is not None:
                self._publish_overlay(image, detections, message)
        except Exception as error:
            self.get_logger().error(f"endpoint YOLO inference failed: {error}")

    def _publish_overlay(
        self,
        image,
        detections: list[EndpointDetection],
        source: Image,
    ) -> None:
        debug = image.copy()
        colors = {"green_arrow": (0, 255, 0), "red_signal": (0, 0, 255)}
        for detection in detections:
            color = colors[detection.class_name]
            p1 = (int(detection.x1), int(detection.y1))
            p2 = (int(detection.x2), int(detection.y2))
            cv2.rectangle(debug, p1, p2, color, 2)
            cv2.putText(
                debug,
                f"{detection.class_name} {detection.confidence:.2f}",
                (p1[0], max(18, p1[1] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
        selected = self._latch.selected_final
        suffix = " LOCKED" if self._latch.locked else " DEFAULT"
        cv2.putText(
            debug,
            f"END LANE F{selected}{suffix}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
        )
        output = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        output.header = source.header
        self._overlay_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EndpointPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
