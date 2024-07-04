import math
import rclpy
from dataclasses import dataclass
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped, Twist


@dataclass
class WheelState:
    setpoint: float
    reading: float
    output: float


@dataclass
class RobotOdometry:
    x_pos: float
    y_pos: float
    theta: float
    v: float
    w: float


@dataclass
class RobotData:
    left_wheel: WheelState
    right_wheel: WheelState
    odometry: RobotOdometry


class RobotBridge(Node):
    def __init__(self):
        super().__init__("robot_bridge")

        self.subscription = self.create_subscription(
            Float32MultiArray,
            "robot_state",
            self.listener_callback,
            10,
        )
        self.publisher = self.create_publisher(
            Odometry,
            "odom",
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg:Float32MultiArray):
        left_wheel = WheelState(msg.data[0], msg.data[1], msg.data[2])
        right_wheel = WheelState(msg.data[3], msg.data[4], msg.data[5])
        odometry = RobotOdometry(*msg.data[6:])
        state = RobotData(left_wheel, right_wheel, odometry)
        # self.get_logger().info(f"data received: {state}")
        orientation = self.quaternion_from_euler(0, 0, state.odometry.theta)

        # transforms
        timestamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "/odom"
        t.child_frame_id = "/base_link"
        t.transform.translation.x = state.odometry.x_pos
        t.transform.translation.y = state.odometry.y_pos
        t.transform.translation.z = 0.0325
        t.transform.rotation = orientation

        # odometry
        odom_msg = Odometry()
        odom_msg.header.frame_id = "/odom"
        odom_msg.child_frame_id = "/base_link"
        odom_msg.header.stamp = timestamp

        odom_msg.pose.pose.position.x = state.odometry.x_pos
        odom_msg.pose.pose.position.y = state.odometry.y_pos
        odom_msg.pose.pose.position.z = 0.0325
        odom_msg.pose.pose.orientation = orientation
        odom_msg.twist.twist.linear.x = state.odometry.v
        odom_msg.twist.twist.angular.z = state.odometry.w

        # broadcast and publish
        self.tf_broadcaster.sendTransform(t)
        self.publisher.publish(odom_msg)


    @staticmethod    
    def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q


def main(args=None):
    rclpy.init(args=args)
    robot_bridge = RobotBridge()
    rclpy.spin(robot_bridge)

    # destroy node
    robot_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
