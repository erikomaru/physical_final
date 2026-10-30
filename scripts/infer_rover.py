#!/usr/bin/env python3
"""ROS 2 runtime for a trained Pai/LeRobot ACT mobile-base policy."""
import argparse
import math
import time

import rclpy
import torch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from lerobot.common.policies.act.modeling_act import ACTPolicy
from rclpy.node import Node


class RoverInference(Node):
    def __init__(self, model_path, goal_x, goal_y, speed_scale):
        super().__init__("rover_act_inference")
        self.goal_x, self.goal_y = goal_x, goal_y
        self.speed_scale = speed_scale
        self.policy = ACTPolicy.from_pretrained(model_path).to("cpu")
        self.policy.eval()
        self.get_logger().info(f"Loaded ACT policy from {model_path}")
        self.latest = None
        self.last_status = 0.0
        self.waiting_logged = False
        self.publisher = self.create_publisher(Twist, "/model/rover/cmd_vel", 10)
        self.create_subscription(Odometry, "/model/rover/odometry", self.on_odom, 10)
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info("Waiting for /model/rover/odometry; inference is running")

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        state = [p.x, p.y, math.sin(yaw), math.cos(yaw)]
        env = [self.goal_x - p.x, self.goal_y - p.y]
        self.latest = {"observation.state": torch.tensor(state, dtype=torch.float32).unsqueeze(0),
                       "observation.environment_state": torch.tensor(env, dtype=torch.float32).unsqueeze(0)}

    def step(self):
        cmd = Twist()
        if self.latest is None:
            self.publisher.publish(cmd)
            if not self.waiting_logged:
                self.get_logger().warning("No odometry received yet; check Gazebo/ROS bridge")
                self.waiting_logged = True
            return
        self.waiting_logged = False
        dx, dy = self.latest["observation.environment_state"][0].tolist()
        distance = math.hypot(dx, dy)
        if distance < 0.8:
            self.get_logger().info("Goal reached; stopping.")
            self.publisher.publish(cmd)
            self.timer.cancel()
            return
        with torch.no_grad():
            action = self.policy.select_action(self.latest).cpu().flatten().tolist()
        # Scale learned forward speed while respecting the SDF diff-drive limit (1.2 m/s).
        # Keep reverse speed and turn rate unchanged to avoid destabilizing the learned route.
        cmd.linear.x = max(-0.35, min(1.2, action[0] * self.speed_scale))
        cmd.angular.z = max(-1.2, min(1.2, action[1]))
        self.publisher.publish(cmd)
        now = time.monotonic()
        if now - self.last_status >= 1.0:
            self.get_logger().info(
                f"Inference active: goal distance={distance:.2f} m, "
                f"cmd_vel linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}"
            )
            self.last_status = now

    def destroy_node(self):
        if rclpy.ok():
            self.publisher.publish(Twist())
        super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model-path", required=True)
    ap.add_argument("--goal-x", type=float, default=-5.8)
    ap.add_argument("--goal-y", type=float, default=-9.5)
    ap.add_argument("--speed-scale", type=float, default=2.0,
                    help="Multiply learned forward speed (default: 2.0; capped at 1.2 m/s)")
    args = ap.parse_args()
    if args.speed_scale <= 0:
        ap.error("--speed-scale must be greater than 0")
    rclpy.init()
    node = RoverInference(args.model_path, args.goal_x, args.goal_y, args.speed_scale)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
