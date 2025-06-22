import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Publishers pour chaque capteur
        self.temp_pub = self.create_publisher(Float32, 'sensor_data/temperature', 10)
        self.hum_pub = self.create_publisher(Float32, 'sensor_data/humidity', 10)
        self.pres_pub = self.create_publisher(Float32, 'sensor_data/pressure', 10)

        # Publisher pour le GUI (message groupé)
        self.gui_pub = self.create_publisher(String, 'sensor_topic', 10)

        # Timer pour publier régulièrement
        self.timer = self.create_timer(0.5, self.publish_data)

    def publish_data(self):
        temp = random.uniform(10.0, 40.0)
        hum = random.uniform(20.0, 80.0)
        pres = random.uniform(900.0, 1100.0)

        # Publication séparée
        self.temp_pub.publish(Float32(data=temp))
        self.hum_pub.publish(Float32(data=hum))
        self.pres_pub.publish(Float32(data=pres))

        # Publication groupée pour GUI
        gui_msg = f"Temp:{temp:.2f},Hum:{hum:.2f},Pres:{pres:.2f}"
        self.gui_pub.publish(String(data=gui_msg))

        self.get_logger().info(f'Published - Temp: {temp:.2f} | Hum: {hum:.2f} | Pres: {pres:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
