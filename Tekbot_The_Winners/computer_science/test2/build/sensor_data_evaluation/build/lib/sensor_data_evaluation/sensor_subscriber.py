import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.create_subscription(Float32, 'sensor_data/temperature', self.temp_callback, 10)
        self.create_subscription(Float32, 'sensor_data/humidity', self.hum_callback, 10)
        self.create_subscription(Float32, 'sensor_data/pressure', self.pres_callback, 10)

    def temp_callback(self, msg):
        value = msg.data
        if 15 <= value <= 35:
            self.get_logger().info(f'Temp OK : {value:.2f} °C')
        else:
            self.get_logger().error(f'Temp hors plage  : {value:.2f} °C')

    def hum_callback(self, msg):
        value = msg.data
        if 30 <= value <= 70:
            self.get_logger().info(f'Humidité OK : {value:.2f} %')
        else:
            self.get_logger().error(f'Humidité hors plage : {value:.2f} %')

    def pres_callback(self, msg):
        value = msg.data
        if 950 <= value <= 1050:
            self.get_logger().info(f'Pression OK : {value:.2f} hPa')
        else:
            self.get_logger().error(f'Pression hors plage  : {value:.2f} hPa')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
