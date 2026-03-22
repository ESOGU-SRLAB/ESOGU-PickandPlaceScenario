#!/usr/bin/env python3
"""
Gazebo → YOLO Pipeline Testi

Bu script:
1. /sim/image (preprocessing sonrası) VE /image (ham Gazebo) topic'lerinden birer frame yakalar
2. Her iki görüntüyü /tmp/ altına kaydeder
3. Her iki görüntü üzerinde YOLO çalıştırır
4. Sonuçları karşılaştırır

Kullanım:
    Gazebo + preprocessing çalışırken:
    python3 ~/colcon_ws/src/yolo_ros/capture_and_test.py
"""

import sys
import cv2
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CaptureNode(Node):
    def __init__(self):
        super().__init__("capture_and_test")
        self.cv_bridge = CvBridge()
        self.raw_received = False
        self.sim_received = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Her iki topic'i dinle
        self.sub_raw = self.create_subscription(Image, "/image", self.raw_cb, qos)
        self.sub_sim = self.create_subscription(Image, "/sim/image", self.sim_cb, qos)
        self.get_logger().info("Görüntüler bekleniyor...")
        self.get_logger().info("  /image (ham Gazebo)")
        self.get_logger().info("  /sim/image (preprocessing sonrası)")

    def raw_cb(self, msg: Image):
        if self.raw_received:
            return
        self.raw_received = True
        self.get_logger().info(f"[/image] encoding={msg.encoding}, boyut={msg.width}x{msg.height}")

        # Passthrough olarak al
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite("/tmp/gazebo_raw_frame.png", img)
        self.get_logger().info("Ham frame kaydedildi: /tmp/gazebo_raw_frame.png")

        # Encoding rgb8 ise BGR'ye çevir ve onu da kaydet
        if msg.encoding in ("rgb8", "RGB8"):
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imwrite("/tmp/gazebo_raw_as_bgr.png", img_bgr)
            self.get_logger().info("RGB→BGR çevrili: /tmp/gazebo_raw_as_bgr.png")

        self._run_yolo("/tmp/gazebo_raw_frame.png", "RAW (/image)")
        self._check_done()

    def sim_cb(self, msg: Image):
        if self.sim_received:
            return
        self.sim_received = True
        self.get_logger().info(f"[/sim/image] encoding={msg.encoding}, boyut={msg.width}x{msg.height}")

        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite("/tmp/gazebo_sim_frame.png", img)
        self.get_logger().info("Preprocessed frame kaydedildi: /tmp/gazebo_sim_frame.png")

        self._run_yolo("/tmp/gazebo_sim_frame.png", "PREPROCESSED (/sim/image)")
        self._check_done()

    def _run_yolo(self, image_path, label):
        try:
            from ultralytics import YOLO
            model = YOLO("/home/emirhan/colcon_ws/src/yolo_ros/best.pt")

            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"YOLO sonuçları — {label}")
            self.get_logger().info(f"{'='*50}")

            results = model.predict(source=image_path, conf=0.3, device="cpu", verbose=False)
            for r in results:
                if len(r.boxes) == 0:
                    self.get_logger().info("  ❌ Hiç tespit yok!")
                for box in r.boxes:
                    cls_name = model.names[int(box.cls)]
                    conf = float(box.conf)
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    self.get_logger().info(
                        f"  → {cls_name} ({conf:.2f}) [{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]"
                    )

                annotated = r.plot()
                out_path = image_path.replace(".png", "_yolo.png")
                cv2.imwrite(out_path, annotated)
                self.get_logger().info(f"  Annotated: {out_path}")

        except ImportError:
            self.get_logger().warn("ultralytics bulunamadı!")

    def _check_done(self):
        if self.raw_received and self.sim_received:
            self.get_logger().info("\n" + "="*50)
            self.get_logger().info("HER İKİ GÖRÜNTÜ ALINDI")
            self.get_logger().info("Kaydedilen dosyalar:")
            self.get_logger().info("  /tmp/gazebo_raw_frame.png      — Ham Gazebo")
            self.get_logger().info("  /tmp/gazebo_raw_frame_yolo.png — Ham + YOLO")
            self.get_logger().info("  /tmp/gazebo_sim_frame.png      — Preprocessed")
            self.get_logger().info("  /tmp/gazebo_sim_frame_yolo.png — Preprocessed + YOLO")
            if self.raw_received:
                self.get_logger().info("  /tmp/gazebo_raw_as_bgr.png     — RGB→BGR çevrilmiş")
            self.get_logger().info("="*50)
            self.get_logger().info("Ctrl+C ile çıkabilirsiniz.")


def main():
    rclpy.init()
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
