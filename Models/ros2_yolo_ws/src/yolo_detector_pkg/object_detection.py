import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    """
    Suscribe a /camera/image_raw, ejecuta YOLO y publica:
      - /detections (vision_msgs/Detection2DArray)
      - /detections/image (sensor_msgs/Image) [opcional, imagen anotada]
    """
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameters('', [
            ('model_path', 'yolov8n.pt'),
            ('conf', 0.25),
            ('image_topic', '/camera/image_raw'),
            ('publish_debug_image', True)
        ])

        self.model = YOLO(self.get_parameter('model_path').value)
        self.model.fuse()  # ligera optimización

        self.bridge = CvBridge()

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = QoSHistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.image_cb, sensor_qos
        )
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', 10)

        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        if self.publish_debug:
            self.pub_img = self.create_publisher(Image, '/detections/image', 10)

        self.get_logger().info('YOLO detector inicializado')

    def image_cb(self, msg: Image):
        # Imagen BGR de OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inferencia
        conf = float(self.get_parameter('conf').value)
        result = self.model.predict(frame, conf=conf, verbose=False)[0]

        out = Detection2DArray()
        out.header = msg.header

        # Dibujado opcional sobre copia del frame
        if self.publish_debug:
            dbg = frame.copy()

        for b in result.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            cls = int(b.cls[0]) if b.cls is not None else -1
            sc  = float(b.conf[0]) if b.conf is not None else 0.0

            det = Detection2D()
            # bbox como centro + tamaño (formato vision_msgs)
            det.bbox.center.x = (x1 + x2) / 2.0
            det.bbox.center.y = (y1 + y2) / 2.0
            det.bbox.center.theta = 0.0
            det.bbox.size_x = (x2 - x1)
            det.bbox.size_y = (y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls)  # id de clase COCO
            hyp.hypothesis.score = sc
            det.results.append(hyp)
            out.detections.append(det)

            if self.publish_debug:
                cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f'{cls}:{sc:.2f}'
                cv2.putText(dbg, label, (int(x1), int(max(0, y1-5))),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

        # Publicaciones
        self.pub_det.publish(out)
        if self.publish_debug:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))

def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
