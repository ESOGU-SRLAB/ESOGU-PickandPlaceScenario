#!/usr/bin/env python3
"""Yeşil maskeleme testi."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

def mask_green(img_bgr):
    """Yeşil pikselleri gri ile değiştir."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([35, 80, 80])
    upper = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    result = img_bgr.copy()
    result[mask > 0] = [128, 128, 128]
    return result

class SaveAndTest(Node):
    def __init__(self):
        super().__init__("save_test")
        self.bridge = CvBridge()
        self.done = False
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(Image, "/sim/image", self.cb, qos)
        self.get_logger().info("Frame bekleniyor...")

    def cb(self, msg):
        if self.done: return
        self.done = True

        img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info(f"Alındı: {msg.width}x{msg.height}")

        model = YOLO("/home/emirhan/colcon_ws/src/yolo_ros/best.pt")

        # Test 1: Ham
        r1 = model.predict(source=img_bgr, conf=0.3, device="cpu", verbose=False)
        print(f"\nHAM: {len(r1[0].boxes)} tespit")
        for box in r1[0].boxes:
            print(f"  {model.names[int(box.cls)]} ({float(box.conf):.2f})")

        # Test 2: Yeşil maskeleme
        img_masked = mask_green(img_bgr)
        cv2.imwrite("/tmp/test_green_masked.png", img_masked)
        r2 = model.predict(source=img_masked, conf=0.3, device="cpu", verbose=False)
        print(f"\nYEŞİL MASKELİ: {len(r2[0].boxes)} tespit")
        for box in r2[0].boxes:
            print(f"  {model.names[int(box.cls)]} ({float(box.conf):.2f})")
        cv2.imwrite("/tmp/test_green_masked_yolo.png", r2[0].plot())

        # Test 3: Yeşil maskeleme + imgsz 1080
        r3 = model.predict(source=img_masked, conf=0.3, device="cpu",
                           imgsz=(1088, 1920), verbose=False)
        print(f"\nYEŞİL MASKELİ + 1080p: {len(r3[0].boxes)} tespit")
        for box in r3[0].boxes:
            print(f"  {model.names[int(box.cls)]} ({float(box.conf):.2f})")

        print(f"\nDosya: /tmp/test_green_masked.png")
        rclpy.shutdown()

rclpy.init()
rclpy.spin(SaveAndTest())
