import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

from gpiozero import PWMOutputDevice


class TorchesControl(Node):

    def __init__(self):
        super().__init__('torches_subscriber')
        self.subscription = self.create_subscription(
            Int64,
            'torches_intensity',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.torches_PIN = 13
        self.torches = PWMOutputDevice(self.torches_PIN, frequency = 200)
        self.get_logger().info('Torches node correctly initialized')

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #convert from 0-100 to PWM values
        # with frequency = 200 I observe dimming between 20 and 40
        if msg.data < 0:
            msg.data = 0
        elif msg.data > 100:
            msg.data = 100
        command = msg.data/100/5 + 0.2
        #send command over PWM
        self.torches.value = command

def main(args=None):
    rclpy.init(args=args)

    torches_subscriber = TorchesControl()

    rclpy.spin(torches_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    torches_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
