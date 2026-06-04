#!/usr/bin/env python3
"""Small CLI helper for BlueROV2 path controller services.

Examples:
  ros2 run bluerov2_path_control mission_client goto --x 1 --y 0 --z -0.5 --yaw 0
  ros2 run bluerov2_path_control mission_client circle --cx 0 --cy 0 --z -0.5 --radius 1 --period 80 --turns 1 --yaw-mode tangent
  ros2 run bluerov2_path_control mission_client stop
  ros2 run bluerov2_path_control mission_client estop --on
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool, Trigger

from bluerov2_path_interfaces.srv import FollowCircle, FollowSpiral, FollowWaypoints, GoTo


def yaw_to_quat(yaw):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def make_pose(x, y, z, yaw):
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation = yaw_to_quat(float(yaw))
    return p


def call_service(node, srv_type, name, request):
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"Service '{name}' not available")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        raise RuntimeError(f"Service '{name}' timed out")
    return future.result()


def main(argv=None):
    parser = argparse.ArgumentParser(description="BlueROV2 path controller mission client")
    parser.add_argument("--ns", default="/path_controller", help="Controller service namespace")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("goto")
    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--z", type=float, required=True)
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--speed", type=float, default=0.12)
    p.add_argument("--hold", action="store_true", default=True)

    p = sub.add_parser("waypoints")
    p.add_argument("--points", nargs="+", required=True, help="Each point: x,y,z,yaw")
    p.add_argument("--speed", type=float, default=0.12)
    p.add_argument("--loop", action="store_true")
    p.add_argument("--hold", action="store_true", default=True)

    p = sub.add_parser("circle")
    p.add_argument("--cx", type=float, required=True)
    p.add_argument("--cy", type=float, required=True)
    p.add_argument("--z", type=float, required=True)
    p.add_argument("--radius", type=float, required=True)
    p.add_argument("--period", type=float, default=80.0)
    p.add_argument("--turns", type=float, default=1.0)
    p.add_argument("--clockwise", action="store_true")
    p.add_argument("--yaw-mode", default="fixed", choices=["fixed", "tangent"])
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--hold", action="store_true", default=True)

    p = sub.add_parser("spiral")
    p.add_argument("--cx", type=float, required=True)
    p.add_argument("--cy", type=float, required=True)
    p.add_argument("--z", type=float, required=True)
    p.add_argument("--r0", type=float, default=0.0)
    p.add_argument("--r1", type=float, required=True)
    p.add_argument("--duration", type=float, default=160.0)
    p.add_argument("--turns", type=float, default=2.0)
    p.add_argument("--clockwise", action="store_true")
    p.add_argument("--yaw-mode", default="fixed", choices=["fixed", "tangent"])
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--hold", action="store_true", default=True)

    sub.add_parser("stop")
    p = sub.add_parser("estop")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--on", action="store_true")
    g.add_argument("--off", action="store_true")

    args = parser.parse_args(argv)
    ns = args.ns.rstrip("/")

    rclpy.init()
    node = rclpy.create_node("mission_client")
    try:
        if args.cmd == "goto":
            req = GoTo.Request()
            req.x, req.y, req.z, req.yaw = args.x, args.y, args.z, args.yaw
            req.speed = args.speed
            req.hold_at_goal = args.hold
            res = call_service(node, GoTo, f"{ns}/go_to", req)
        elif args.cmd == "waypoints":
            req = FollowWaypoints.Request()
            for item in args.points:
                vals = [float(x) for x in item.split(",")]
                if len(vals) != 4:
                    raise ValueError("Waypoint must be x,y,z,yaw")
                req.waypoints.append(make_pose(*vals))
            req.speed = args.speed
            req.loop = args.loop
            req.hold_at_end = args.hold
            res = call_service(node, FollowWaypoints, f"{ns}/follow_waypoints", req)
        elif args.cmd == "circle":
            req = FollowCircle.Request()
            req.center_x, req.center_y, req.z = args.cx, args.cy, args.z
            req.radius, req.period, req.turns = args.radius, args.period, args.turns
            req.clockwise = args.clockwise
            req.yaw_mode = args.yaw_mode
            req.yaw = args.yaw
            req.hold_at_end = args.hold
            res = call_service(node, FollowCircle, f"{ns}/follow_circle", req)
        elif args.cmd == "spiral":
            req = FollowSpiral.Request()
            req.center_x, req.center_y, req.z = args.cx, args.cy, args.z
            req.radius_start, req.radius_end = args.r0, args.r1
            req.duration, req.turns = args.duration, args.turns
            req.clockwise = args.clockwise
            req.yaw_mode = args.yaw_mode
            req.yaw = args.yaw
            req.hold_at_end = args.hold
            res = call_service(node, FollowSpiral, f"{ns}/follow_spiral", req)
        elif args.cmd == "stop":
            res = call_service(node, Trigger, f"{ns}/stop", Trigger.Request())
        elif args.cmd == "estop":
            req = SetBool.Request()
            req.data = bool(args.on)
            res = call_service(node, SetBool, f"{ns}/emergency_stop", req)
        else:
            raise ValueError(args.cmd)
        print(res)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
