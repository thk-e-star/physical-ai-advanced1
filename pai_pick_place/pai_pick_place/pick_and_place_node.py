import rclpy
from rclpy.node import Node
import tf2_ros
import math
import subprocess

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 0.5秒周期でTFを監視
        self.timer = self.create_timer(0.5, self.control_loop)
        self.executed = False
        self.get_logger().info('【最新版 Pick & Place】TF検出ループ開始...')

    def control_loop(self):
        if self.executed:
            return

        try:
            # 最新のTFを取得 (Time(seconds=0) で最新値を要求)
            trans = self.tf_buffer.lookup_transform(
                'world', 
                'detected_target', 
                rclpy.time.Time()
            )
            x = float(trans.transform.translation.x)
            y = float(trans.transform.translation.y)
            z = float(trans.transform.translation.z)

            theta1 = float(math.atan2(y, x))
            self.get_logger().info(f'★【TF2 受信成功】Target: X={x:.3f}, Y={y:.3f}, Z={z:.3f}')
            self.get_logger().info(f'★ 目標旋回角度: {theta1:.3f} rad')

            # CLI 方式で確実に Gazebo へ送信
            cmd = [
                'ros2', 'topic', 'pub', '--once',
                '/arm_controller/joint_trajectory',
                'trajectory_msgs/msg/JointTrajectory',
                f'{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: "world"}}, joint_names: ["joint1", "joint2", "joint3"], points: [{{positions: [{theta1:.3f}, 0.5, -0.5], time_from_start: {{sec: 1, nanosec: 0}}}}]}}'
            ]
            
            self.get_logger().info('★ CLI コマンドを発行中...')
            subprocess.run(cmd)
            self.executed = True
            self.get_logger().info('★ アーム駆動完了！')

        except Exception:
            self.get_logger().info('TF: detected_target を待機中...', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
