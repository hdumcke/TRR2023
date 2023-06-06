import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import os

class ButtonMonitor(Node):

    def __init__(self):
        super().__init__('botton_monitor')
        self.ButPin = 12
        self._button_pressed = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ButPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.frequency = 5
        timer = self.create_timer(1 / self.frequency, self.timer_callback)

    def timer_callback(self):
        status_file = "/tmp/RUN"
        if self._button_pressed == 0 and GPIO.input(self.ButPin):
            self._button_pressed = 10
            if os.path.isfile(status_file):
                os.remove(status_file)
                self.get_logger().info("STOP")
            else:
                open(status_file, 'a').close()
                self.get_logger().info("RUN")
        if self._button_pressed:
            self._button_pressed -= 1


def main(args=None):
    rclpy.init(args=args)

    node = ButtonMonitor()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

