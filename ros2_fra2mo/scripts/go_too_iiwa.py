#! /usr/bin/env python3

import os
import yaml
import math
import time

import rclpy
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def quat_from_yaw(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main():
    rclpy.init()

    node = rclpy.create_node("go_to_iiwa_manual_forward")

    pub_handover = node.create_publisher(Bool, "/fra2mo/ready_for_handover", 10)
    pub_cmdvel = node.create_publisher(Twist, "/fra2mo/cmd_vel", 10)

    navigator = BasicNavigator()

    # -------------------------
    # Load target from YAML
    # -------------------------
    config_path = os.path.join(
        os.getenv("HOME"),
        "ros2_ws/src/ros2_fra2mo/config/iiwa_goal.yaml"
    )

    target_x, target_y = 0.0, 0.0
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            data = yaml.safe_load(f)
            target_x = float(data["iiwa_goal"]["x"])
            target_y = float(data["iiwa_goal"]["y"])
        node.get_logger().info(f"Target loaded -> X: {target_x:.3f}, Y: {target_y:.3f}")
    else:
        node.get_logger().warn(f"File {config_path} not found! Using (0,0).")

    # -------------------------
    # Nav2 approach (puoi lasciarlo conservativo)
    # -------------------------
    node.get_logger().info("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()

    # Offset SOLO per arrivare “in zona” (non è il limite finale)
    offset_x = -0.4
    offset_y = 0.1

    goal_x = target_x + offset_x
    goal_y = target_y + offset_y

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(goal_x)
    goal_pose.pose.position.y = float(goal_y)
    goal_pose.pose.position.z = 0.0

    # orienta verso il target così poi il "forward" va nella direzione giusta
    yaw = math.atan2(target_y - goal_y, target_x - goal_x)
    qz, qw = quat_from_yaw(yaw)
    goal_pose.pose.orientation.z = float(qz)
    goal_pose.pose.orientation.w = float(qw)

    node.get_logger().info(f"[NAV] Going to ({goal_x:.3f}, {goal_y:.3f}) facing target...")
    navigator.goToPose(goal_pose)

    # Qui NON cancelliamo presto: lasciamo finire o timeout
    timeout_s = 180.0
    while not navigator.isTaskComplete():
        fb = navigator.getFeedback()
        if fb and Duration.from_msg(fb.navigation_time) > Duration(seconds=timeout_s):
            node.get_logger().warn("[NAV] Timeout. Canceling Nav2 task.")
            navigator.cancelTask()
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    nav_res = navigator.getResult()
    node.get_logger().info(f"[NAV] Result: {nav_res}")

    # -------------------------
    # Settle stop (evita rotazioni residue)
    # -------------------------
    stop = Twist()
    stop.linear.x = 0.0
    stop.angular.z = 0.0

    t_end = time.time() + 1.0
    while rclpy.ok() and time.time() < t_end:
        pub_cmdvel.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    # -------------------------
    # Manual forward until ArUco seen
    # -------------------------
    aruco_seen = {"ok": False}

    def aruco_cb(_msg: PoseStamped):
        aruco_seen["ok"] = True

    sub = node.create_subscription(PoseStamped, "/fra2mo/aruco_single/pose", aruco_cb, 10)

    forward_speed = 0.07     # m/s
    max_distance = 0.60      # m (fail-safe)
    max_time = 10.0          # s (fail-safe)

    node.get_logger().info("[MANUAL] Driving forward until ArUco is visible...")

    moved = 0.0
    t0 = time.time()
    last = time.time()

    cmd = Twist()
    cmd.linear.x = float(forward_speed)
    cmd.angular.z = 0.0

    while rclpy.ok() and not aruco_seen["ok"]:
        now = time.time()
        dt = now - last
        last = now

        if (now - t0) > max_time:
            node.get_logger().warn("[MANUAL] Timeout: ArUco not seen.")
            break

        moved += forward_speed * dt
        if moved > max_distance:
            node.get_logger().warn("[MANUAL] Max distance reached without seeing ArUco.")
            break

        pub_cmdvel.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)

    # stop
    for _ in range(5):
        pub_cmdvel.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_subscription(sub)

    if not aruco_seen["ok"]:
        node.get_logger().warn("[MANUAL] Not sending handover (ArUco not visible).")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("[MANUAL] ArUco visible! Sending handover...")

    msg = Bool()
    msg.data = True
    for _ in range(5):
        pub_handover.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("[INFO] /fra2mo/ready_for_handover = True sent")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


