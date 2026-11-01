import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import tf2_ros
from geometry_msgs.msg import TransformStamped

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.get_logger().info('YOLO 検出ノード起動 (world 直結モード)...')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        results = self.model(cv_image, conf=0.01, verbose=False)
        annotated_frame = results[0].plot()

        boxes = results[0].boxes
        if len(boxes) > 0:
            box = boxes[0]
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = self.model.names[cls_id]

            xyxy = box.xyxy[0].cpu().numpy()
            u = float((xyxy[0] + xyxy[2]) / 2.0)
            v = float((xyxy[1] + xyxy[3]) / 2.0)

            # カメラ座標系から world 座標系への直感変換 (カメラ配置依存のオフセット)
            z_cam = 0.8
            x_cam = (u - 320.0) * z_cam / 554.25
            y_cam = (v - 240.0) * z_cam / 554.25

            # world 座標系としての絶対位置
            world_x = 0.5 + x_cam
            world_y = 0.0 + y_cam
            world_z = 0.05  # 卓上高さ

            self.get_logger().info(f'[YOLO Detected: {class_name}] world pos -> X:{world_x:.2f}m, Y:{world_y:.2f}m, Z:{world_z:.2f}m')

            # 親フレームを world にして TF 発行
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'detected_target'
            t.transform.translation.x = float(world_x)
            t.transform.translation.y = float(world_y)
            t.transform.translation.z = float(world_z)
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

        cv2.imshow('YOLOv8 Detection', annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
