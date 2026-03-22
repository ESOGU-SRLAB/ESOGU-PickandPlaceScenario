#!/usr/bin/env python3
"""
Gazebo Görüntü Teşhis Scripti

Gazebo kamerasından gelen görüntünün encoding ve renk kanalı bilgilerini
analiz eder. Renk kanalı sorununu tespit etmeye yardımcı olur.

Kullanım:
    1. Gazebo simülasyonunu başlatın
    2. Bu scripti çalıştırın:
       python3 diagnose_gazebo_image.py

Script şunları yapar:
    - /image topic'inden bir görüntü alır
    - Encoding bilgisini yazdırır
    - Orijinal ve BGR-dönüştürülmüş görüntüyü kaydeder
    - Her iki görüntü üzerinde YOLO çalıştırır ve sonuçları karşılaştırır
"""

import sys
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class DiagnoseNode(Node):
    def __init__(self):
        super().__init__("diagnose_gazebo_image")
        self.cv_bridge = CvBridge()
        self.received = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.sub = self.create_subscription(Image, "/image", self.cb, qos)
        self.get_logger().info("Gazebo görüntüsü bekleniyor (/image)...")

    def cb(self, msg: Image):
        if self.received:
            return
        self.received = True

        self.get_logger().info("=" * 60)
        self.get_logger().info("GAZEBO GÖRÜNTÜ TEŞHİSİ")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Encoding    : {msg.encoding}")
        self.get_logger().info(f"Boyut       : {msg.width} x {msg.height}")
        self.get_logger().info(f"Step        : {msg.step}")
        self.get_logger().info(f"Big endian  : {msg.is_bigendian}")

        # Orijinal (passthrough)
        img_raw = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite("/tmp/gazebo_raw.png", img_raw)
        self.get_logger().info(f"Ham görüntü kaydedildi: /tmp/gazebo_raw.png")

        # BGR dönüşümü
        img_bgr = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite("/tmp/gazebo_bgr.png", img_bgr)
        self.get_logger().info(f"BGR görüntü kaydedildi: /tmp/gazebo_bgr.png")

        # RGB dönüşümü (kanal ters çevrilmiş)
        img_rgb_swapped = cv2.cvtColor(img_raw, cv2.COLOR_RGB2BGR)
        cv2.imwrite("/tmp/gazebo_rgb_swapped.png", img_rgb_swapped)
        self.get_logger().info(f"RGB→BGR çevrilmiş kaydedildi: /tmp/gazebo_rgb_swapped.png")

        # Kanal analizi
        b, g, r = cv2.split(img_raw)
        self.get_logger().info(f"Kanal ortalamaları (raw) - CH0: {b.mean():.1f}, CH1: {g.mean():.1f}, CH2: {r.mean():.1f}")

        b2, g2, r2 = cv2.split(img_bgr)
        self.get_logger().info(f"Kanal ortalamaları (bgr) - B: {b2.mean():.1f}, G: {g2.mean():.1f}, R: {r2.mean():.1f}")

        # Renk kanalı sırası tespiti
        if msg.encoding in ("rgb8", "RGB8"):
            self.get_logger().warn(
                "⚠️  Encoding 'rgb8' → Gazebo RGB üretiyor. "
                "YOLO BGR beklediğinden RENK KANALI ÇEVRİMİ GEREKLİ!"
            )
        elif msg.encoding in ("bgr8", "BGR8"):
            self.get_logger().info(
                "✅ Encoding 'bgr8' → Doğrudan YOLO'ya gönderilebilir."
            )
        else:
            self.get_logger().warn(
                f"⚠️  Beklenmeyen encoding: '{msg.encoding}'. Manuel kontrol gerekli."
            )

        # YOLO ile test (opsiyonel)
        try:
            from ultralytics import YOLO
            model_path = "/home/emirhan/colcon_ws/src/yolo_ros/best.pt"
            model = YOLO(model_path)

            self.get_logger().info("\n--- YOLO Tespiti: Ham görüntü (passthrough) ---")
            results_raw = model.predict(source=img_raw, conf=0.3, device="cpu", verbose=False)
            for r in results_raw:
                for box in r.boxes:
                    cls_name = model.names[int(box.cls)]
                    conf = float(box.conf)
                    self.get_logger().info(f"  {cls_name} ({conf:.2f})")
                if len(r.boxes) == 0:
                    self.get_logger().info("  Tespit yok!")
                annotated = r.plot()
                cv2.imwrite("/tmp/gazebo_detect_raw.png", annotated)

            self.get_logger().info("\n--- YOLO Tespiti: BGR dönüştürülmüş ---")
            results_bgr = model.predict(source=img_bgr, conf=0.3, device="cpu", verbose=False)
            for r in results_bgr:
                for box in r.boxes:
                    cls_name = model.names[int(box.cls)]
                    conf = float(box.conf)
                    self.get_logger().info(f"  {cls_name} ({conf:.2f})")
                if len(r.boxes) == 0:
                    self.get_logger().info("  Tespit yok!")
                annotated = r.plot()
                cv2.imwrite("/tmp/gazebo_detect_bgr.png", annotated)

            self.get_logger().info("\nTespit sonuçları /tmp/ altına kaydedildi.")

        except ImportError:
            self.get_logger().warn("ultralytics bulunamadı, YOLO testi atlandı.")

        self.get_logger().info("=" * 60)
        self.get_logger().info("Teşhis tamamlandı. Ctrl+C ile çıkabilirsiniz.")


def main():
    rclpy.init()
    node = DiagnoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
