import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraDriver(Node):
    """
    Lee frames de /dev/videoX y publica sensor_msgs/Image en /camera/image_raw.
    Parámetros:
      - device: path del dispositivo (default: /dev/video0)
      - width, height: resolución deseada
      - fps: tasa de publicación (usada para el temporizador)
    """
    def __init__(self):
        super().__init__('camera_driver')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos)
        self.bridge = CvBridge()

        dev = self.get_parameter('device').get_parameter_value().string_value
        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)

        self.cap = cv2.VideoCapture(dev)
        if not self.cap.isOpened():
            self.get_logger().error(f'No se pudo abrir {dev}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

        fps = float(self.get_parameter('fps').value)
        self.timer = self.create_timer(max(1.0 / fps, 1e-3), self._publish_frame)
        self.get_logger().info(f'Publicando {dev} -> /camera/image_raw @ {fps} FPS')

    def _publish_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Frame drop')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = CameraDriver()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'cap') and node.cap:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
