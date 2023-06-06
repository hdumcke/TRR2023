import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage

import numpy as np
import cv2
from cv_bridge import CvBridge

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.pub_img = self.create_publisher(Image, "/opencv_tests/images", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_compressed_img = self.create_publisher(CompressedImage, "/opencv_tests/images/compressed", 10)
        self.cvb = CvBridge()
        self.image_input = None
        self.image_output = None
        self.lower_bgr_values = np.array([40,  40,  40])
        self.upper_bgr_values = np.array([255, 255, 255])
        self.max_velocity = 0.2
        self.current_velocity = 0.0
        self.change_rate = 5

    def listener_callback(self, msg):
        self.image_input = self.cvb.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.segmentation4(msg.height, msg.width)
        out_msg = self.cvb.cv2_to_imgmsg(self.image_output)
        out_msg.header.frame_id = msg.header.frame_id
        if out_msg.encoding == '8UC3':
            out_msg.encoding = 'bgr8'
        self.pub_img.publish(out_msg)

    def segmentation1(self, height, width):
        crop = self.image_input[height//2:height, 0:width]
        mask = cv2.inRange(crop, self.lower_bgr_values, self.upper_bgr_values)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > 100.0:
                self.get_logger().info("%s" % M['m00'])
                if (M['m00'] > 13000.0):
                    cv2.drawContours(crop, contour, -1, (255,255,0), 1)
                    cv2.putText(crop, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
                else:
                    cv2.drawContours(crop, contour, -1, (255,0,255), 1)
                    cv2.putText(crop, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
        self.image_output = crop

    def segmentation2(self, height, width):
        crop = self.image_input[height//2:height, 0:width]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        self.image_output = cv2.Canny(gray,100,200)

    def segmentation3(self, height, width):
        crop = self.image_input[height//2:height, 0:width]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
        canny = cv2.Canny(thresh,100,200)
        self.image_output = canny

    def segmentation4(self, height, width):
        follow_x = 0
        follow_y = 0
        crop = self.image_input[height//2:height, 0:width]
        hc, wc, dc= crop.shape
        mid_x = wc//2
        mid_y = hc//2
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
        thresh = cv2.bitwise_not(thresh)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            M = cv2.moments(contour)
            #self.get_logger().info("%s" % M['m00'])
            if M['m00'] > 100.0:
                if (M['m00'] > 20000.0):
                    cv2.drawContours(crop, contour, -1, (255,255,0), 1)
                    #cv2.putText(crop, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    #cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
                    follow_x = int(M["m10"]/M["m00"])
                    follow_y = int(M["m01"]/M["m00"])
                else:
                    cv2.drawContours(crop, contour, -1, (255,0,255), 1)
                    cv2.putText(crop, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
        cv2.circle(crop, (mid_x, mid_y), 5, (0,0, 255), 7)
        cv2.circle(crop, (follow_x, follow_y), 5, (0,255,0), 7)
        self.image_output = crop
        self.follow(mid_x, mid_y, follow_x, follow_y)

    def follow(self, mid_x, mid_y, follow_x, follow_y):
        KP = 0.8
        error = float(follow_x -  mid_x) / float(mid_x)
        #self.get_logger().info("%s %s %0.2f" % (mid_x, follow_x, error))
        self.current_velocity += self.max_velocity / self.change_rate
        if self.current_velocity > self.max_velocity:
            self.current_velocity = self.max_velocity
        msg = Twist()
        msg.linear.x = self.current_velocity
        msg.angular.z = error * -KP
        self.pub_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    lf = LineFollower()

    rclpy.spin(lf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
