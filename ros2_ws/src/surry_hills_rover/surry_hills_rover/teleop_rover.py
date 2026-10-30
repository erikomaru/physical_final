#!/usr/bin/env python3
"""Discrete arrow-key teleop; each key press runs one measured motion step."""
from collections import deque
import os
import select
import subprocess
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import UInt32MultiArray


# One tap moves about 0.6 m or turns about 29 degrees.
LINEAR_SPEED = 1.0
TURN_SPEED = 1.0
MOVE_SECONDS = 0.60
TURN_SECONDS = 0.50
WORLD = "surry_hills_osm_simple"
START_POSE = (0.0, -0.20, 0.33)


class RoverTeleop(Node):
    def __init__(self):
        super().__init__("rover_teleop")
        self.publisher = self.create_publisher(Twist, "/model/rover/cmd_vel", 10)
        self.count_publisher = self.create_publisher(UInt32MultiArray, "/model/rover/key_counts", 10)
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.counts = {"up": 0, "down": 0, "left": 0, "right": 0}
        self.queue = deque()
        self.active = (0.0, 0.0)
        self.active_until = 0.0
        self.timer = self.create_timer(0.02, self.tick)
        self.publish_counts()
        print("矢印キー1回ごとに1ステップ: 上/下=約0.6m、左/右=約29度 | x=緑の開始位置へ戻る | Esc/Ctrl-Cで終了", flush=True)

    def read_key(self):
        # os.read avoids Python stdin buffering arrow-key escape sequences.
        if not select.select([self.fd], [], [], 0)[0]:
            return None
        key = os.read(self.fd, 1).decode(errors="ignore")
        if key != "\x1b":
            return key
        tail = b""
        deadline = time.monotonic() + 0.03
        while len(tail) < 2 and time.monotonic() < deadline:
            if select.select([self.fd], [], [], 0.003)[0]:
                tail += os.read(self.fd, 2 - len(tail))
        return "\x1b" + tail.decode(errors="ignore")

    def publish_counts(self):
        # Order: up, down, left, right. Bag this topic with /cmd_vel.
        msg = UInt32MultiArray()
        msg.data = [self.counts[k] for k in ("up", "down", "left", "right")]
        self.count_publisher.publish(msg)

    def tick(self):
        key = self.read_key()
        moves = {
            "\x1b[A": ("up", LINEAR_SPEED, 0.0, MOVE_SECONDS),
            "\x1b[B": ("down", -LINEAR_SPEED, 0.0, MOVE_SECONDS),
            "\x1b[D": ("left", 0.0, TURN_SPEED, TURN_SECONDS),
            "\x1b[C": ("right", 0.0, -TURN_SPEED, TURN_SECONDS),
        }
        if key in moves:
            name, linear, angular, duration = moves[key]
            self.counts[name] += 1
            self.queue.append((linear, angular, duration))
            self.publish_counts()
            print("押下数 上:{up} 下:{down} 左:{left} 右:{right}".format(**self.counts), flush=True)
        elif key in ("x", "X"):
            self.queue.clear()
            self.active = (0.0, 0.0)
            self.publisher.publish(Twist())
            if self.reset_to_start():
                self.counts = {"up": 0, "down": 0, "left": 0, "right": 0}
                self.publish_counts()
                print("ロボットを緑の開始位置へ戻しました。押下数をリセットしました。", flush=True)
        elif key == "\x1b":
            self.queue.clear()
            self.active = (0.0, 0.0)
            self.publisher.publish(Twist())
            rclpy.shutdown()
            return

        now = time.monotonic()
        if now >= self.active_until:
            if self.queue:
                linear, angular, duration = self.queue.popleft()
                self.active = (linear, angular)
                self.active_until = now + duration
            else:
                self.active = (0.0, 0.0)
                self.active_until = now
        command = Twist()
        command.linear.x, command.angular.z = self.active
        self.publisher.publish(command)

    def reset_to_start(self):
        x, y, z = START_POSE
        request = (f'name: "rover", position: {{x: {x}, y: {y}, z: {z}}}, '
                   "orientation: {x: 0, y: 0, z: 0, w: 1}")
        try:
            result = subprocess.run(
                ["ign", "service", "-s", f"/world/{WORLD}/set_pose",
                 "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
                 "--req", request, "--timeout", "2000"],
                capture_output=True, text=True, timeout=3, check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f"Gazebo pose reset failed: {exc}")
            return False
        if result.returncode != 0 or "data: true" not in result.stdout.lower():
            self.get_logger().error(f"Gazebo pose reset failed: {result.stdout} {result.stderr}")
            return False
        return True

    def destroy_node(self):
        if rclpy.ok():
            self.publisher.publish(Twist())
        print("キー押下数 (上, 下, 左, 右): " + ", ".join(str(self.counts[k]) for k in ("up", "down", "left", "right")), flush=True)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main():
    if not sys.stdin.isatty():
        raise SystemExit("矢印キー操作には対話ターミナルが必要です。")
    rclpy.init()
    node = RoverTeleop()
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
