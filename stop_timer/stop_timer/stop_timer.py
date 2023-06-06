import rclpy
from rclpy.node import Node
import os

class StopTimer(Node):

    def __init__(self):
        super().__init__('stop_timer')
        self.reset_timer()
        self.duration = 5  # duration in seconds
        timer = self.create_timer(1, self.timer_callback)

    def reset_timer(self):
        self.time_start = self.get_clock().now().nanoseconds

    def timer_callback(self):
         if not os.path.isfile("/tmp/RUN"):
             self.reset_timer()
             return
         if self.get_clock().now().nanoseconds - self.time_start > self.duration * 1e+9:
             os.remove("/tmp/RUN")
             print("STOP")

def main(args=None):
    rclpy.init(args=args)

    node = StopTimer()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
