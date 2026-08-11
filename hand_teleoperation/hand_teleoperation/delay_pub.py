import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class DelayPublisher(Node):
    def __init__(self):
        super().__init__('delay_publisher')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        msg = String()
        now_ms = int(time.time()*1000)
        msg.data = str(now_ms)
        self.pub.publish(msg)
        self.get_logger().info(f'sent {msg.data}')

def main():
    rclpy.init()
    node = DelayPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
