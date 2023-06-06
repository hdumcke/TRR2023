import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge
import os

class StopLineDetector(Node):

    def __init__(self):
        super().__init__('stop_line_detector')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.pub_img = self.create_publisher(Image, "/opencv_tests/images", 10)
        self.cvb = CvBridge()
        self.contour_threshold = 1000
        self.check_frequency = 2.0
        self.stop_timeout = 2.0
        self.round_count = 1
        self.reset_counter()

        # lower boundary RED color range values; Hue (0 - 10)
        self.lower1 = np.array([0, 100, 20])
        self.upper1 = np.array([10, 255, 255])
        # upper boundary RED color range values; Hue (160 - 180)
        self.lower2 = np.array([160,100,20])
        self.upper2 = np.array([179,255,255])

    def reset_counter(self):
        self.last_seen = self.get_clock().now().nanoseconds
        self.last_check = self.get_clock().now().nanoseconds
        self.prepare_to_stop = False
        self.round_count = 1

    def listener_callback(self, msg):
        if not os.path.isfile("/tmp/RUN"):
            self.reset_counter()
            return
        self.check_stop_now()
        if self.get_clock().now().nanoseconds - self.last_check < (1 / self.check_frequency) * 1e+9:
            return
        line_detected = False
        #import pdb
        #pdb.set_trace()
        image_input = self.cvb.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        img_hsv=cv2.cvtColor(image_input, cv2.COLOR_BGR2HSV)
        lower_mask = cv2.inRange(img_hsv, self.lower1, self.upper1)
        upper_mask = cv2.inRange(img_hsv, self.lower2, self.upper2)
        full_mask = lower_mask + upper_mask;
        out_img = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
        contours, hierarchy = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > self.contour_threshold:
                line_detected = True
            if M['m00'] > 10:
                cv2.drawContours(out_img, contour, -1, (255,255,0), 1)
                cv2.putText(out_img, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)

        self.set_stop_timer(line_detected)
        out_msg = self.cvb.cv2_to_imgmsg(out_img)
        out_msg.header.frame_id = msg.header.frame_id
        if out_msg.encoding == '8UC3':
            out_msg.encoding = 'bgr8'
        self.pub_img.publish(out_msg)

    def set_stop_timer(self, is_red):
        if is_red:
            self.last_seen = self.get_clock().now().nanoseconds
            self.prepare_to_stop = True

    def check_stop_now(self):
        if self.prepare_to_stop and (self.get_clock().now().nanoseconds - self.last_seen > self.stop_timeout * 1e+9):
            self.round_count -= 1
            if self.round_count == 0:
                os.remove("/tmp/RUN")
                print("STOP")
            else:
                self.prepare_to_stop = False


def main(args=None):
    rclpy.init(args=args)

    node = StopLineDetector()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
