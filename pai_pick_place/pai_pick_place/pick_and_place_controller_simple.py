import time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PickAndPlaceController(Node):
    def __init__(self):
        super().__init__('pick_and_place_controller')

        # TF2 Listener のセットアップ
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # アームおよびグリッパー制御用のパブリッシャー
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # 定期実行タイマー (0.5秒周期)
        self.timer = self.create_timer(0.5, self.control_loop)
        self.state = 'LOOK_FOR_TARGET'
        self.get_logger().info('Pick & Place 制御ノードが起動しました。ターゲット待機中...')

    def send_arm_cmd(self, joint_positions, sec=2):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']  # ロボットアームの関節名に合わせて修正可
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = sec
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_gripper_cmd(self, position, sec=1):
        msg = JointTrajectory()
        msg.joint_names = ['gripper_joint']  # グリッパーの関節名に合わせて修正可
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = sec
        msg.points.append(point)
        self.gripper_pub.publish(msg)

    def control_loop(self):
        if self.state == 'LOOK_FOR_TARGET':
            try:
                # 5 & 6: world / base_link 基準への TF2 変換取得
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(
                    'base_link',
                    'detected_target',
                    now,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                x = trans.transform.translation.x
                y = trans.transform.translation.y
                z = trans.transform.translation.z

                self.get_logger().info(f'目標検出成功! (base_link基準): X={x:.2f}, Y={y:.2f}, Z={z:.2f}')
                self.target_pos = (x, y, z)
                self.state = 'APPROACH'

            except TransformException as ex:
                self.get_logger().info('ターゲットのTF2座標を待機中...')

        elif self.state == 'APPROACH':
            self.get_logger().info('[シーケンス 1/5] アプローチ: 目標物体の真上へ移動')
            # グリッパーを開く
            self.send_gripper_cmd(0.04)
            # アームをターゲット上方に配置 (簡易角度指示)
            self.send_arm_cmd([0.0, -0.5, 0.5, 0.0], sec=3)
            time.sleep(3.5)
            self.state = 'GRASP'

        elif self.state == 'GRASP':
            self.get_logger().info('[シーケンス 2/5] 把持 (Pick): 下降して掴む')
            # アームを下降
            self.send_arm_cmd([0.0, -0.8, 0.8, 0.0], sec=2)
            time.sleep(2.5)
            # 7: グリッパーを閉じる
            self.send_gripper_cmd(0.0)
            time.sleep(1.5)
            self.state = 'LIFT'

        elif self.state == 'LIFT':
            self.get_logger().info('[シーケンス 3/5] 持ち上げ: 持ち上げ動作')
            self.send_arm_cmd([0.0, -0.2, 0.2, 0.0], sec=2)
            time.sleep(2.5)
            self.state = 'PLACE'

        elif self.state == 'PLACE':
            self.get_logger().info('[シーケンス 4/5] 搬送 & 放置 (Place): 目的場所へ旋回して離す')
            # 別の場所へ旋回 (関節1を90度回転)
            self.send_arm_cmd([1.57, -0.5, 0.5, 0.0], sec=3)
            time.sleep(3.5)
            # 8: グリッパーを開いて放置
            self.send_gripper_cmd(0.04)
            time.sleep(1.5)
            self.state = 'HOME'

        elif self.state == 'HOME':
            self.get_logger().info('[シーケンス 5/5] 完了: 初期姿勢へ復帰')
            self.send_arm_cmd([0.0, 0.0, 0.0, 0.0], sec=2)
            time.sleep(2.5)
            self.get_logger().info('Pick & Place シーケンスが完了しました。')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
