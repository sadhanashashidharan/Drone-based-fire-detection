# scripts/fire_detect.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

# Calibration showed max_seen=16129 from a distance
# When drone flies DIRECTLY over cube at z=1.5 it will be much higher
# Set to 10000 — safely above far-away noise, triggers when drone is close
FIRE_AREA_THRESHOLD = 10000


def beep():
    os.system('echo -e "\\a"')


class FireDetect(Node):
    def __init__(self):
        super().__init__('fire_detect')
        self.bridge     = CvBridge()
        self.last_state = False

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10)
        self.pub = self.create_publisher(Bool, '/fire_detected', 10)

        self.get_logger().info(f'FireDetect ready — threshold={FIRE_AREA_THRESHOLD}px')

    def detect_fire(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        orange = cv2.inRange(hsv, np.array([ 8, 100, 100]), np.array([25, 255, 255]))
        red_lo = cv2.inRange(hsv, np.array([  0, 100, 100]), np.array([10, 255, 255]))
        red_hi = cv2.inRange(hsv, np.array([165, 100, 100]), np.array([180, 255, 255]))
        mask   = orange | red_lo | red_hi

        k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = int(max((cv2.contourArea(c) for c in contours), default=0))
        return best_area >= FIRE_AREA_THRESHOLD, best_area

    def callback(self, msg):
        frame          = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected, area = self.detect_fire(frame)

        self.get_logger().info(
            f'area={area:6d}  {"*** FIRE ***" if detected else "clear"}',
            throttle_duration_sec=0.5)

        if detected and not self.last_state:
            self.get_logger().warn('FIRE DETECTED!')
            beep()
        if not detected and self.last_state:
            self.get_logger().info('Fire out of range')

        self.last_state = detected
        out      = Bool()
        out.data = detected
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FireDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
