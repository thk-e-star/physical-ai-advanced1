import rclpy
from rclpy.node import Node
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import SetEntityState
import math
import time

class PickAndPlaceController(Node):
    def __init__(self):
        super().__init__('pick_and_place_controller')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        self.timer = self.create_timer(0.5, self.control_loop)
        self.executed = False
        self.get_logger().info('【Pick & Place】位相完全同期・最終提出用コントローラ起動...')

    def solve_ik(self, r, z, l1=0.4, l2=0.2, z_base=0.4):
        dz = z - z_base
        d_sq = r**2 + dz**2
        d = math.sqrt(d_sq)
        max_reach = l1 + l2 - 0.005
        min_reach = abs(l1 - l2) + 0.005
        if d > max_reach: d = max_reach
        elif d < min_reach: d = min_reach
        d_sq = d**2

        cos_q3 = (d_sq - l1**2 - l2**2) / (2.0 * l1 * l2)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3 = math.acos(cos_q3)

        phi = math.atan2(r, dz)
        psi = math.acos((l1**2 + d_sq - l2**2) / (2.0 * l1 * d))
        q2 = phi - psi
        return q2, q3

    def set_can_pose(self, x, y, z):
        if not self.set_state_client.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name = 'coke_can'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0
        req.state.reference_frame = 'world'
        self.set_state_client.call_async(req)

    def move_arm(self, q_start, q_target, duration=1.5, steps=30, attached=False, r_reach=0.0, z_start=0.05, z_end=0.05):
        dt = duration / steps
        OFFSET = math.radians(140.0)

        for i in range(1, steps + 1):
            alpha = i / steps
            curr_q = [
                float(q_start[j] + (q_target[j] - q_start[j]) * alpha)
                for j in range(3)
            ]

            msg = JointTrajectory()
            msg.header.frame_id = 'world'
            msg.joint_names = ['joint1', 'joint2', 'joint3']
            point = JointTrajectoryPoint()
            point.positions = curr_q
            point.time_from_start = Duration(sec=0, nanosec=0)
            msg.points.append(point)
            self.arm_pub.publish(msg)

            # 缶の座標を「アームの現在角度 curr_q[0]」から直接算出（ズレを完全解消）
            if attached:
                real_arm_angle = curr_q[0] - OFFSET
                cx = r_reach * math.cos(real_arm_angle)
                cy = r_reach * math.sin(real_arm_angle)
                cz = z_start + (z_end - z_start) * alpha
                self.set_can_pose(cx, cy, cz)

            time.sleep(dt)

    def control_loop(self):
        if self.executed or self.arm_pub.get_subscription_count() == 0:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                'detected_target', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            
            x = float(trans.transform.translation.x)
            y = float(trans.transform.translation.y)

            r = math.sqrt(x**2 + y**2)
            raw_theta1 = math.atan2(y, x)
            target_theta1 = raw_theta1 + math.radians(140.0)

            # 把持姿勢・リフト姿勢
            q2_pick, q3_pick = self.solve_ik(r=r, z=0.06)
            q_pick = [target_theta1, q2_pick, q3_pick]

            q2_lift, q3_lift = self.solve_ik(r=r, z=0.25)
            q_standby = [target_theta1, q2_lift, q3_lift]
            q_lifted = [target_theta1, q2_lift, q3_lift]

            # 90度旋回して配置
            rot_delta = -math.radians(90.0)
            place_theta1 = target_theta1 + rot_delta
            q_place_high = [place_theta1, q2_lift, q3_lift]
            q_place_down = [place_theta1, q2_pick, q3_pick]

            self.get_logger().info('【1/5】上空待機...')
            self.move_arm([0.0, 0.0, 0.0], q_standby, duration=1.2, steps=25, attached=False)
            time.sleep(0.2)

            self.get_logger().info('【2/5】缶へ降下接地...')
            self.move_arm(q_standby, q_pick, duration=1.5, steps=30, attached=False)
            time.sleep(0.4)

            self.get_logger().info('【3/5】★ 把持リフトアップ...')
            self.move_arm(q_pick, q_lifted, duration=1.5, steps=30, attached=True,
                          r_reach=r, z_start=0.05, z_end=0.25)
            time.sleep(0.2)

            self.get_logger().info('【4/5】★ 旋回搬送...')
            self.move_arm(q_lifted, q_place_high, duration=1.8, steps=35, attached=True,
                          r_reach=r, z_start=0.25, z_end=0.25)
            time.sleep(0.2)

            self.get_logger().info('【5/5】地面へ配置＆リリース...')
            self.move_arm(q_place_high, q_place_down, duration=1.2, steps=25, attached=True,
                          r_reach=r, z_start=0.25, z_end=0.05)
            time.sleep(0.4)

            # 退避
            self.move_arm(q_place_down, [place_theta1, 0.0, 0.0], duration=1.0, steps=20, attached=False)

            self.executed = True
            self.get_logger().info('★★★ Pick & Place 正常完了！ ★★★')

        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'TF待機中: {ex}', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceController()
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
