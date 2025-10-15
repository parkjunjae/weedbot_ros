#!/usr/bin/env python3

import math, yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from pymap3d import geodetic2enu
from ament_index_python.packages import get_package_share_directory

class GPSGoalSender(Node):
    def __init__(self, lat, lon, yaw_deg, datum_cfg_path=None):
        super().__init__('gps_goal_sender')

        # Load datum config
        if datum_cfg_path:
            cfg_path = Path(datum_cfg_path)
        else:
            cfg_path = Path(get_package_share_directory('gps_goal_sender')) / 'config' / 'datum.yaml'
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.datum = cfg['datum']
        self.enu_map = cfg['enu_to_map']
        
        self.yaw_off = math.radians(self.enu_map.get('yaw_deg', 0.0))
        self.tx = self.enu_map.get('x_offset', 0.0)
        self.ty = self.enu_map.get('y_offset', 0.0)

        self.lat = lat
        self.lon = lon
        self.yaw_deg = yaw_deg

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def enu_to_map(self, x_e, y_n):
        c, s = math.cos(self.yaw_off), math.sin(self.yaw_off)
        x_m = c * x_e - s * y_n + self.tx
        y_m = s * x_e + c * y_n + self.ty
        return x_m, y_m

    def llh_to_map(self, lat, lon, alt=0.0):
        e, n, u = geodetic2enu(lat, lon, alt,
                               self.datum['lat'], self.datum['lon'], self.datum.get('alt', 0.0))
        return self.enu_to_map(e, n)

    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available")
            return

        x_m, y_m = self.llh_to_map(self.lat, self.lon, 0.0)
        yaw = math.radians(self.yaw_deg)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x_m
        goal.pose.pose.position.y = y_m
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f"[gps_goal_sender] lat={self.lat:.6f}, lon={self.lon:.6f} "
                               f"→ map=({x_m:.2f}, {y_m:.2f}), yaw={self.yaw_deg:.1f}°")
        f = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if not gh or not gh.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted; waiting for result...")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        self.get_logger().info(f"Result: {rf.result().result if rf.result() else 'no result'}")

def main():
    rclpy.init()
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--lat', type=float, required=True)
    ap.add_argument('--lon', type=float, required=True)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--datum', type=str, help='override datum.yaml path')
    args = ap.parse_args()

    node = GPSGoalSender(args.lat, args.lon, args.yaw, args.datum)
    node.send_goal()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

