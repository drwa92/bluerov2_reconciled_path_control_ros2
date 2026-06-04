#!/usr/bin/env python3
"""Direct-thruster BlueROV2 path-following controller for DAVE/Gazebo.

The node exposes services for go-to, waypoint, circle, spiral, and generic
trajectory missions.  It publishes six direct thrust commands and therefore
should be used with ArduSub disabled when direct actuator ownership is desired.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from bluerov2_path_control.hydrodynamics import FourDofWrenchObserver
from bluerov2_path_control.reconciliation import ProjectedWrenchReconciler

from bluerov2_path_interfaces.srv import (
    FollowCircle,
    FollowSpiral,
    FollowTrajectory,
    FollowWaypoints,
    GoTo,
)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    # ROS quaternion: x,y,z,w
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def pose_to_xyzyaw(pose: Pose) -> np.ndarray:
    return np.array(
        [pose.position.x, pose.position.y, pose.position.z, yaw_from_quat(pose.orientation)],
        dtype=float,
    )


def smoothstep(s: float) -> Tuple[float, float]:
    """Return h(s), dh/ds with zero slope at endpoints."""
    s = min(1.0, max(0.0, s))
    return 3.0 * s * s - 2.0 * s * s * s, 6.0 * s * (1.0 - s)


@dataclass
class Mission:
    kind: str = "idle"
    start_time: float = 0.0
    start_pose: np.ndarray = field(default_factory=lambda: np.zeros(4))
    target_pose: np.ndarray = field(default_factory=lambda: np.zeros(4))
    duration: float = 1.0
    speed: float = 0.12
    hold_at_end: bool = True
    loop: bool = False
    waypoints: List[np.ndarray] = field(default_factory=list)
    waypoint_index: int = 0
    center: np.ndarray = field(default_factory=lambda: np.zeros(2))
    radius: float = 1.0
    radius_start: float = 0.0
    radius_end: float = 1.0
    period: float = 60.0
    turns: float = 1.0
    clockwise: bool = False
    yaw_mode: str = "fixed"
    fixed_yaw: float = 0.0


class BlueROV2PathController(Node):
    def __init__(self) -> None:
        super().__init__("path_controller")

        # Topics and model.
        self.declare_parameter("model_name", "bluerov2")
        self.declare_parameter("odom_topic", "/model/bluerov2/odometry")
        self.declare_parameter(
            "cmd_topic_template", "/model/bluerov2/joint/thruster{idx}_joint/cmd_thrust"
        )
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("use_sim_time", True)

        # Allocation.
        default_b = [
            -0.70710678, -0.70710678, 0.70710678, 0.70710678, 0.0, 0.0,
            -0.70710678, 0.70710678, -0.70710678, 0.70710678, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, -1.0, -1.0,
            -1.0, 1.0, 1.0, -1.0, 0.0, 0.0,
        ]
        self.declare_parameter("allocation_matrix_flat", default_b)
        self.declare_parameter("thruster_max", 40.0)
        self.declare_parameter("thruster_min", -40.0)
        self.declare_parameter("thruster_rate_max", 80.0)
        self.declare_parameter("allocation_rho", 0.03)
        self.declare_parameter("allocation_iterations", 30)

        self.declare_parameter("tau_xy_max", 25.0)
        self.declare_parameter("tau_z_max", 45.0)
        self.declare_parameter("tau_yaw_max", 8.0)

        # Controller.
        self.declare_parameter("controller_mode", "pid")
        self.declare_parameter("use_ref_velocity", True)
        self.declare_parameter("velocity_frame", "body")
        self.declare_parameter("kp_xy", 8.0)
        self.declare_parameter("kd_xy", 8.0)
        self.declare_parameter("ki_xy", 0.0)
        self.declare_parameter("ixy_max", 2.0)
        self.declare_parameter("kp_z", 35.0)
        self.declare_parameter("kd_z", 12.0)
        self.declare_parameter("ki_z", 6.0)
        self.declare_parameter("iz_max", 3.0)
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("kd_yaw", 1.5)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("iyaw_max", 1.0)
        self.declare_parameter("ks_smc_xy", 1.0)
        self.declare_parameter("ks_smc_z", 1.0)
        self.declare_parameter("ks_smc_yaw", 0.5)
        self.declare_parameter("smc_slope", 3.0)
        self.declare_parameter("lambda_xy", 0.6)
        self.declare_parameter("lambda_z", 0.8)
        self.declare_parameter("lambda_yaw", 0.5)

        # Mission behavior.
        self.declare_parameter("default_speed", 0.12)
        self.declare_parameter("default_goal_tolerance", 0.12)
        self.declare_parameter("default_yaw_tolerance", 0.08)
        self.declare_parameter("hold_at_start", True)
        self.declare_parameter("stop_thrusters_on_idle", True)
        self.declare_parameter("debug_period_sec", 2.0)
        self.declare_parameter("log_to_csv", True)
        self.declare_parameter("log_dir", "~/dave_ws/results_bluerov2_path_control")

        # Optional advanced modules: model-aided wrench reconstruction and
        # projected virtual-wrench reconciliation. Disabled by default so this
        # package remains a clean path controller unless explicitly enabled.
        self.declare_parameter("enable_wrench_reconciliation", False)
        self.declare_parameter("hydro_model", "heavy")
        self.declare_parameter("vehicle_mass", 10.0)
        self.declare_parameter("vehicle_izz", 0.269)
        self.declare_parameter("wrench_observer_alpha", 0.20)
        self.declare_parameter("wrench_derivative_clip", 5.0)
        self.declare_parameter("recon_gain", 0.08)
        self.declare_parameter("rhat_bound_xy", 8.0)
        self.declare_parameter("rhat_bound_z", 12.0)
        self.declare_parameter("rhat_bound_yaw", 8.0)
        self.declare_parameter("rhat_rate_limit", 0.8)
        self.declare_parameter("sensor_gate_min", 0.25)
        self.declare_parameter("sensor_confidence", 1.0)

        self.model_name = str(self.get_parameter("model_name").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_template = str(self.get_parameter("cmd_topic_template").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / max(self.rate_hz, 1.0)

        b_flat = list(self.get_parameter("allocation_matrix_flat").value)
        self.B = np.array(b_flat, dtype=float).reshape(4, 6)
        self.u_prev = np.zeros(6, dtype=float)
        self.i_err = np.zeros(4, dtype=float)

        self.pose = None  # [x,y,z,yaw]
        self.vel_body = np.zeros(4, dtype=float)  # [u,v,w,r]
        self.have_odom = False
        self.emergency = False
        self.mission = Mission(kind="idle")
        self.hold_pose: Optional[np.ndarray] = None
        self.last_debug_time = -1e9
        self.wrench_reconciliation_enabled = bool(self.get_parameter("enable_wrench_reconciliation").value)

        self.wrench_observer = FourDofWrenchObserver(
            model_name=str(self.get_parameter("hydro_model").value),
            mass=float(self.get_parameter("vehicle_mass").value),
            izz=float(self.get_parameter("vehicle_izz").value),
            lowpass_alpha=float(self.get_parameter("wrench_observer_alpha").value),
            derivative_clip=float(self.get_parameter("wrench_derivative_clip").value),
        )
        recon_bounds = np.array([
            float(self.get_parameter("rhat_bound_xy").value),
            float(self.get_parameter("rhat_bound_xy").value),
            float(self.get_parameter("rhat_bound_z").value),
            float(self.get_parameter("rhat_bound_yaw").value),
        ])
        self.reconciler = ProjectedWrenchReconciler(
            gain=float(self.get_parameter("recon_gain").value),
            bounds=recon_bounds,
            rate_limit=float(self.get_parameter("rhat_rate_limit").value),
            sensor_gate_min=float(self.get_parameter("sensor_gate_min").value),
        )
        self.last_tau_est = np.zeros(4, dtype=float)
        self.last_raw_residual = np.zeros(4, dtype=float)
        self.last_comp_residual = np.zeros(4, dtype=float)
        self.last_tau_target = np.zeros(4, dtype=float)
        self.last_sensor_confidence = float(self.get_parameter("sensor_confidence").value)

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.thruster_pubs = []
        for idx in range(1, 7):
            topic = cmd_template.format(idx=idx)
            self.thruster_pubs.append(self.create_publisher(Float64, topic, 10))

        # Advanced diagnostics/reconciliation topics.
        self.pub_tau_est = self.create_publisher(Float64MultiArray, "reconciliation/estimated_wrench", 10)
        self.pub_tau_nom = self.create_publisher(Float64MultiArray, "reconciliation/nominal_wrench", 10)
        self.pub_raw_res = self.create_publisher(Float64MultiArray, "reconciliation/raw_residual", 10)
        self.pub_comp_res = self.create_publisher(Float64MultiArray, "reconciliation/compensated_residual", 10)
        self.pub_rhat = self.create_publisher(Float64MultiArray, "reconciliation/rhat", 10)

        # Mission services.
        self.create_service(GoTo, "go_to", self.srv_go_to)
        self.create_service(FollowWaypoints, "follow_waypoints", self.srv_waypoints)
        self.create_service(FollowCircle, "follow_circle", self.srv_circle)
        self.create_service(FollowSpiral, "follow_spiral", self.srv_spiral)
        self.create_service(FollowTrajectory, "follow_trajectory", self.srv_trajectory)
        self.create_service(Trigger, "stop", self.srv_stop)
        self.create_service(SetBool, "emergency_stop", self.srv_emergency)
        self.create_service(SetBool, "enable_reconciliation", self.srv_enable_reconciliation)
        self.create_service(Trigger, "reset_reconciliation", self.srv_reset_reconciliation)

        self.csv_file = None
        self.csv_writer = None
        if bool(self.get_parameter("log_to_csv").value):
            log_dir = os.path.expanduser(str(self.get_parameter("log_dir").value))
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, "path_controller_log.csv")
            self.csv_file = open(path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "t", "mission", "x_ref", "y_ref", "z_ref", "yaw_ref",
                "x", "y", "z", "yaw", "ex", "ey", "ez", "eyaw",
                "tau_x", "tau_y", "tau_z", "tau_yaw",
                "tau_target_x", "tau_target_y", "tau_target_z", "tau_target_yaw",
                "tau_est_x", "tau_est_y", "tau_est_z", "tau_est_yaw",
                "raw_res_x", "raw_res_y", "raw_res_z", "raw_res_yaw",
                "rhat_x", "rhat_y", "rhat_z", "rhat_yaw",
                "comp_res_x", "comp_res_y", "comp_res_z", "comp_res_yaw",
                "sensor_confidence", "recon_enabled", "recon_update_enabled",
                "u1", "u2", "u3", "u4", "u5", "u6",
            ])
            self.get_logger().info(f"Logging to {path}")

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            "BlueROV2 path controller started. Services: go_to, follow_waypoints, "
            "follow_circle, follow_spiral, follow_trajectory, stop, emergency_stop."
        )

    # ---------------- ROS callbacks ----------------
    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pose = np.array([p.x, p.y, p.z, yaw], dtype=float)
        tw = msg.twist.twist
        # DAVE /model odometry twist behaves like body velocity in our sign tests.
        self.vel_body = np.array([tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z], dtype=float)
        if not self.have_odom:
            self.have_odom = True
            if bool(self.get_parameter("hold_at_start").value):
                self.hold_pose = self.pose.copy()
                self.mission = Mission(kind="hold", target_pose=self.hold_pose.copy())
            self.get_logger().info(
                f"Received odometry. Initial pose: x={self.pose[0]:.2f}, y={self.pose[1]:.2f}, "
                f"z={self.pose[2]:.2f}, yaw={self.pose[3]:.2f}"
            )

    # ---------------- Services ----------------
    def _not_ready(self, response):
        response.accepted = False
        response.message = "No odometry received yet."
        return response

    def srv_go_to(self, request, response):
        if not self.have_odom:
            return self._not_ready(response)
        target = np.array([request.x, request.y, request.z, wrap_pi(request.yaw)], dtype=float)
        speed = float(request.speed) if request.speed > 0.0 else float(self.get_parameter("default_speed").value)
        dist = float(np.linalg.norm(target[:3] - self.pose[:3]))
        duration = max(dist / max(speed, 1e-3), 2.0)
        self.mission = Mission(
            kind="goto", start_time=self.now_sec(), start_pose=self.pose.copy(),
            target_pose=target, duration=duration, speed=speed,
            hold_at_end=bool(request.hold_at_goal), fixed_yaw=target[3]
        )
        self.reset_integrators()
        response.accepted = True
        response.message = f"Go-to accepted. Duration {duration:.1f} s."
        self.get_logger().info(response.message)
        return response

    def srv_waypoints(self, request, response):
        if not self.have_odom:
            return self._not_ready(response)
        if len(request.waypoints) == 0:
            response.accepted = False
            response.message = "Waypoint list is empty."
            return response
        wps = [pose_to_xyzyaw(p) for p in request.waypoints]
        speed = float(request.speed) if request.speed > 0.0 else float(self.get_parameter("default_speed").value)
        self.mission = Mission(
            kind="waypoints", start_time=self.now_sec(), start_pose=self.pose.copy(),
            target_pose=wps[0].copy(), speed=speed, waypoints=wps, waypoint_index=0,
            loop=bool(request.loop), hold_at_end=bool(request.hold_at_end)
        )
        self._start_waypoint_segment()
        self.reset_integrators()
        response.accepted = True
        response.message = f"Waypoint mission accepted with {len(wps)} waypoints."
        self.get_logger().info(response.message)
        return response

    def srv_circle(self, request, response):
        if not self.have_odom:
            return self._not_ready(response)
        if request.radius <= 0.0 or request.period <= 0.0:
            response.accepted = False
            response.message = "Circle radius and period must be positive."
            return response
        self.mission = Mission(
            kind="circle", start_time=self.now_sec(), start_pose=self.pose.copy(),
            center=np.array([request.center_x, request.center_y], dtype=float),
            radius=float(request.radius), period=float(request.period), turns=float(request.turns),
            clockwise=bool(request.clockwise), yaw_mode=(request.yaw_mode or "fixed"),
            fixed_yaw=wrap_pi(float(request.yaw)), target_pose=np.array([
                request.center_x + request.radius, request.center_y, request.z, wrap_pi(request.yaw)
            ], dtype=float), hold_at_end=bool(request.hold_at_end)
        )
        self.reset_integrators()
        response.accepted = True
        response.message = "Circle mission accepted."
        self.get_logger().info(response.message)
        return response

    def srv_spiral(self, request, response):
        if not self.have_odom:
            return self._not_ready(response)
        if request.turns <= 0.0 or request.duration <= 0.0:
            response.accepted = False
            response.message = "Spiral turns and duration must be positive."
            return response
        self.mission = Mission(
            kind="spiral", start_time=self.now_sec(), start_pose=self.pose.copy(),
            center=np.array([request.center_x, request.center_y], dtype=float),
            radius_start=float(request.radius_start), radius_end=float(request.radius_end),
            duration=float(request.duration), turns=float(request.turns), clockwise=bool(request.clockwise),
            yaw_mode=(request.yaw_mode or "fixed"), fixed_yaw=wrap_pi(float(request.yaw)),
            target_pose=np.array([request.center_x + request.radius_end, request.center_y, request.z, wrap_pi(request.yaw)], dtype=float),
            hold_at_end=bool(request.hold_at_end)
        )
        self.reset_integrators()
        response.accepted = True
        response.message = "Spiral mission accepted."
        self.get_logger().info(response.message)
        return response

    def srv_trajectory(self, request, response):
        # Convenience wrapper for scripts.  For production use, prefer the typed services above.
        traj = request.trajectory_type.strip().lower()
        p = list(request.params)
        try:
            if traj == "square":
                if len(p) < 4:
                    raise ValueError("square params=[side, segment_time, z, yaw]")
                side, seg, z, yaw = p[:4]
                # Build four waypoint poses relative to current position.
                wps = []
                x0, y0 = self.pose[0], self.pose[1]
                coords = [(x0 + side, y0), (x0 + side, y0 + side), (x0, y0 + side), (x0, y0)]
                for x, y in coords:
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z
                    pose.orientation = yaw_to_quat(yaw)
                    wps.append(pose)
                fake_req = type("Req", (), {})()
                fake_req.waypoints = wps
                fake_req.speed = side / max(seg, 1e-3)
                fake_req.loop = False
                fake_req.hold_at_end = request.hold_at_end
                return self.srv_waypoints(fake_req, response)
            if traj == "circle":
                if len(p) < 8:
                    raise ValueError("circle params=[cx,cy,z,radius,period,turns,clockwise,yaw]")
                fake_req = type("Req", (), {})()
                fake_req.center_x, fake_req.center_y, fake_req.z = p[0], p[1], p[2]
                fake_req.radius, fake_req.period, fake_req.turns = p[3], p[4], p[5]
                fake_req.clockwise = bool(round(p[6]))
                fake_req.yaw = p[7]
                fake_req.yaw_mode = request.yaw_mode or "fixed"
                fake_req.hold_at_end = request.hold_at_end
                return self.srv_circle(fake_req, response)
            if traj == "spiral":
                if len(p) < 9:
                    raise ValueError("spiral params=[cx,cy,z,r0,r1,duration,turns,clockwise,yaw]")
                fake_req = type("Req", (), {})()
                fake_req.center_x, fake_req.center_y, fake_req.z = p[0], p[1], p[2]
                fake_req.radius_start, fake_req.radius_end = p[3], p[4]
                fake_req.duration, fake_req.turns = p[5], p[6]
                fake_req.clockwise = bool(round(p[7]))
                fake_req.yaw = p[8]
                fake_req.yaw_mode = request.yaw_mode or "fixed"
                fake_req.hold_at_end = request.hold_at_end
                return self.srv_spiral(fake_req, response)
            raise ValueError(f"Unsupported trajectory_type '{traj}'")
        except Exception as exc:
            response.accepted = False
            response.message = str(exc)
            return response

    def srv_stop(self, request, response):
        if self.have_odom:
            self.hold_pose = self.pose.copy()
            self.mission = Mission(kind="hold", target_pose=self.hold_pose.copy())
            self.reset_integrators()
            response.success = True
            response.message = "Mission stopped. Holding current pose."
        else:
            self.mission = Mission(kind="idle")
            self.publish_thrusters(np.zeros(6))
            response.success = True
            response.message = "Mission stopped. No odometry; thrusters zeroed."
        self.get_logger().warn(response.message)
        return response

    def srv_emergency(self, request, response):
        self.emergency = bool(request.data)
        if self.emergency:
            self.mission = Mission(kind="idle")
            self.publish_thrusters(np.zeros(6))
            response.success = True
            response.message = "Emergency stop engaged. Thrusters zeroed."
            self.get_logger().error(response.message)
        else:
            if self.have_odom:
                self.hold_pose = self.pose.copy()
                self.mission = Mission(kind="hold", target_pose=self.hold_pose.copy())
            response.success = True
            response.message = "Emergency stop cleared. Holding current pose."
            self.get_logger().warn(response.message)
        return response

    def srv_enable_reconciliation(self, request, response):
        self.wrench_reconciliation_enabled = bool(request.data)
        response.success = True
        response.message = (
            "Projected virtual-wrench reconciliation enabled."
            if self.wrench_reconciliation_enabled
            else "Projected virtual-wrench reconciliation disabled."
        )
        self.get_logger().warn(response.message)
        return response

    def srv_reset_reconciliation(self, request, response):
        self.reconciler.reset()
        self.wrench_observer.reset()
        self.last_tau_est[:] = 0.0
        self.last_raw_residual[:] = 0.0
        self.last_comp_residual[:] = 0.0
        response.success = True
        response.message = "Reconciliation and wrench observer reset."
        self.get_logger().warn(response.message)
        return response

    # ---------------- Mission reference generation ----------------
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def reset_integrators(self) -> None:
        self.i_err[:] = 0.0

    def _start_waypoint_segment(self) -> None:
        m = self.mission
        target = m.waypoints[m.waypoint_index]
        dist = float(np.linalg.norm(target[:3] - self.pose[:3]))
        m.start_time = self.now_sec()
        m.start_pose = self.pose.copy()
        m.target_pose = target.copy()
        m.duration = max(dist / max(m.speed, 1e-3), 2.0)

    def reference(self, t: float) -> Tuple[np.ndarray, np.ndarray, float, bool]:
        """Return pose_ref[4], vel_ref_world[4], elapsed, done."""
        m = self.mission
        if m.kind == "idle":
            if self.hold_pose is not None and not bool(self.get_parameter("stop_thrusters_on_idle").value):
                return self.hold_pose.copy(), np.zeros(4), 0.0, False
            return self.pose.copy(), np.zeros(4), 0.0, False

        if m.kind == "hold":
            return m.target_pose.copy(), np.zeros(4), 0.0, False

        elapsed = t - m.start_time

        if m.kind == "goto" or m.kind == "waypoints":
            s = elapsed / max(m.duration, 1e-6)
            h, dhds = smoothstep(s)
            ref = m.start_pose + h * (m.target_pose - m.start_pose)
            ref[3] = wrap_pi(m.start_pose[3] + h * wrap_pi(m.target_pose[3] - m.start_pose[3]))
            vel = dhds / max(m.duration, 1e-6) * (m.target_pose - m.start_pose)
            vel[3] = dhds / max(m.duration, 1e-6) * wrap_pi(m.target_pose[3] - m.start_pose[3])
            done = s >= 1.0
            if done and m.kind == "waypoints":
                tol = float(self.get_parameter("default_goal_tolerance").value)
                if np.linalg.norm(self.pose[:3] - m.target_pose[:3]) <= max(tol, 0.03):
                    m.waypoint_index += 1
                    if m.waypoint_index >= len(m.waypoints):
                        if m.loop:
                            m.waypoint_index = 0
                        else:
                            return ref, np.zeros(4), elapsed, True
                    self._start_waypoint_segment()
                    return self.reference(t)
            return ref, vel, elapsed, done and m.kind == "goto"

        if m.kind == "circle":
            sign = -1.0 if m.clockwise else 1.0
            omega = sign * 2.0 * math.pi / max(m.period, 1e-6)
            theta = omega * elapsed
            x = m.center[0] + m.radius * math.cos(theta)
            y = m.center[1] + m.radius * math.sin(theta)
            vx = -m.radius * omega * math.sin(theta)
            vy = m.radius * omega * math.cos(theta)
            yaw = m.fixed_yaw
            yaw_rate = 0.0
            if m.yaw_mode.strip().lower() == "tangent":
                yaw = math.atan2(vy, vx)
                yaw_rate = omega
            z = m.target_pose[2]
            total = math.inf if m.turns <= 0.0 else abs(m.turns * m.period)
            done = elapsed >= total
            return np.array([x, y, z, yaw]), np.array([vx, vy, 0.0, yaw_rate]), elapsed, done

        if m.kind == "spiral":
            s = min(1.0, max(0.0, elapsed / max(m.duration, 1e-6)))
            sign = -1.0 if m.clockwise else 1.0
            theta = sign * 2.0 * math.pi * m.turns * s
            dtheta_dt = sign * 2.0 * math.pi * m.turns / max(m.duration, 1e-6)
            radius = m.radius_start + s * (m.radius_end - m.radius_start)
            dr_dt = (m.radius_end - m.radius_start) / max(m.duration, 1e-6)
            c, ss = math.cos(theta), math.sin(theta)
            x = m.center[0] + radius * c
            y = m.center[1] + radius * ss
            vx = dr_dt * c - radius * dtheta_dt * ss
            vy = dr_dt * ss + radius * dtheta_dt * c
            yaw = m.fixed_yaw
            yaw_rate = 0.0
            if m.yaw_mode.strip().lower() == "tangent":
                yaw = math.atan2(vy, vx)
                yaw_rate = dtheta_dt
            z = m.target_pose[2]
            done = elapsed >= m.duration
            return np.array([x, y, z, yaw]), np.array([vx, vy, 0.0, yaw_rate]), elapsed, done

        return self.pose.copy(), np.zeros(4), 0.0, False

    def complete_mission(self, ref: np.ndarray) -> None:
        if self.mission.hold_at_end:
            self.hold_pose = ref.copy()
            self.mission = Mission(kind="hold", target_pose=self.hold_pose.copy())
            self.get_logger().info("Mission complete. Holding final pose.")
        else:
            self.mission = Mission(kind="idle")
            self.publish_thrusters(np.zeros(6))
            self.get_logger().info("Mission complete. Thrusters zeroed.")
        self.reset_integrators()

    # ---------------- Control and allocation ----------------
    def control_wrench(self, ref: np.ndarray, ref_vel_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        psi = self.pose[3]
        c, s = math.cos(psi), math.sin(psi)
        Rtw = np.array([[c, s], [-s, c]], dtype=float)  # world -> body for xy

        e_world_xy = self.pose[:2] - ref[:2]
        e_body_xy = Rtw @ e_world_xy
        e_z = self.pose[2] - ref[2]
        e_yaw = wrap_pi(self.pose[3] - ref[3])

        use_ref_vel = bool(self.get_parameter("use_ref_velocity").value)
        if use_ref_vel:
            v_ref_body_xy = Rtw @ ref_vel_world[:2]
            w_ref = ref_vel_world[2]
            yaw_rate_ref = ref_vel_world[3]
        else:
            v_ref_body_xy = np.zeros(2)
            w_ref = 0.0
            yaw_rate_ref = 0.0

        vel_frame = str(self.get_parameter("velocity_frame").value).strip().lower()
        vel_xy = self.vel_body[:2]
        if vel_frame == "world":
            vel_xy = Rtw @ self.vel_body[:2]
        e_vel_xy = vel_xy - v_ref_body_xy
        e_w = self.vel_body[2] - w_ref
        e_r = self.vel_body[3] - yaw_rate_ref

        self.i_err[0:2] += e_body_xy * self.dt
        self.i_err[2] += e_z * self.dt
        self.i_err[3] += e_yaw * self.dt
        self.i_err[0:2] = np.clip(self.i_err[0:2], -float(self.get_parameter("ixy_max").value), float(self.get_parameter("ixy_max").value))
        self.i_err[2] = float(np.clip(self.i_err[2], -float(self.get_parameter("iz_max").value), float(self.get_parameter("iz_max").value)))
        self.i_err[3] = float(np.clip(self.i_err[3], -float(self.get_parameter("iyaw_max").value), float(self.get_parameter("iyaw_max").value)))

        kp_xy = float(self.get_parameter("kp_xy").value)
        kd_xy = float(self.get_parameter("kd_xy").value)
        ki_xy = float(self.get_parameter("ki_xy").value)
        kp_z = float(self.get_parameter("kp_z").value)
        kd_z = float(self.get_parameter("kd_z").value)
        ki_z = float(self.get_parameter("ki_z").value)
        kp_yaw = float(self.get_parameter("kp_yaw").value)
        kd_yaw = float(self.get_parameter("kd_yaw").value)
        ki_yaw = float(self.get_parameter("ki_yaw").value)

        tau = np.zeros(4, dtype=float)
        tau[0:2] = -kp_xy * e_body_xy - kd_xy * e_vel_xy - ki_xy * self.i_err[0:2]
        tau[2] = -kp_z * e_z - kd_z * e_w - ki_z * self.i_err[2]
        tau[3] = -kp_yaw * e_yaw - kd_yaw * e_r - ki_yaw * self.i_err[3]

        if str(self.get_parameter("controller_mode").value).strip().lower() == "smc":
            slope = float(self.get_parameter("smc_slope").value)
            lam_xy = float(self.get_parameter("lambda_xy").value)
            lam_z = float(self.get_parameter("lambda_z").value)
            lam_yaw = float(self.get_parameter("lambda_yaw").value)
            sx = e_vel_xy[0] + lam_xy * e_body_xy[0]
            sy = e_vel_xy[1] + lam_xy * e_body_xy[1]
            sz = e_w + lam_z * e_z
            syaw = e_r + lam_yaw * e_yaw
            tau[0] -= float(self.get_parameter("ks_smc_xy").value) * math.tanh(slope * sx)
            tau[1] -= float(self.get_parameter("ks_smc_xy").value) * math.tanh(slope * sy)
            tau[2] -= float(self.get_parameter("ks_smc_z").value) * math.tanh(slope * sz)
            tau[3] -= float(self.get_parameter("ks_smc_yaw").value) * math.tanh(slope * syaw)

        tau[0:2] = np.clip(tau[0:2], -float(self.get_parameter("tau_xy_max").value), float(self.get_parameter("tau_xy_max").value))
        tau[2] = float(np.clip(tau[2], -float(self.get_parameter("tau_z_max").value), float(self.get_parameter("tau_z_max").value)))
        tau[3] = float(np.clip(tau[3], -float(self.get_parameter("tau_yaw_max").value), float(self.get_parameter("tau_yaw_max").value)))

        err = np.array([e_body_xy[0], e_body_xy[1], e_z, e_yaw], dtype=float)
        return tau, err

    def allocate(self, tau: np.ndarray) -> np.ndarray:
        umin = float(self.get_parameter("thruster_min").value)
        umax = float(self.get_parameter("thruster_max").value)
        rate = float(self.get_parameter("thruster_rate_max").value)
        rho = float(self.get_parameter("allocation_rho").value)
        iters = int(self.get_parameter("allocation_iterations").value)
        lo = np.maximum(np.full(6, umin), self.u_prev - rate * self.dt)
        hi = np.minimum(np.full(6, umax), self.u_prev + rate * self.dt)

        A = np.vstack((self.B, math.sqrt(rho) * np.eye(6)))
        b = np.concatenate((tau, math.sqrt(rho) * self.u_prev))
        try:
            u = np.linalg.lstsq(A, b, rcond=None)[0]
        except np.linalg.LinAlgError:
            u = self.u_prev.copy()
        u = np.clip(u, lo, hi)

        # Projected-gradient refinement of the bounded least-squares objective.
        lip = 2.0 * (np.linalg.norm(self.B, 2) ** 2 + rho) + 1e-6
        step = 0.85 / lip
        for _ in range(max(0, iters)):
            grad = 2.0 * self.B.T @ (self.B @ u - tau) + 2.0 * rho * (u - self.u_prev)
            u = np.clip(u - step * grad, lo, hi)
        return u

    def publish_thrusters(self, u: np.ndarray) -> None:
        for pub, val in zip(self.thruster_pubs, u):
            msg = Float64()
            msg.data = float(val)
            pub.publish(msg)

    @staticmethod
    def _array_msg(values: np.ndarray) -> Float64MultiArray:
        msg = Float64MultiArray()
        msg.data = [float(x) for x in np.asarray(values, dtype=float).reshape(-1)]
        return msg

    def update_reconciliation(self) -> None:
        sensor_conf = float(self.get_parameter("sensor_confidence").value)
        self.last_sensor_confidence = sensor_conf
        tau_est = self.wrench_observer.update(self.vel_body, self.dt)
        tau_nominal = self.B @ self.u_prev
        raw_residual = tau_est - tau_nominal

        if self.wrench_reconciliation_enabled:
            rhat = self.reconciler.update(raw_residual, self.dt, sensor_confidence=sensor_conf)
        else:
            rhat = np.zeros(4, dtype=float)

        comp_residual = raw_residual - rhat
        self.last_tau_est = tau_est.copy()
        self.last_raw_residual = raw_residual.copy()
        self.last_comp_residual = comp_residual.copy()

        self.pub_tau_est.publish(self._array_msg(tau_est))
        self.pub_tau_nom.publish(self._array_msg(tau_nominal))
        self.pub_raw_res.publish(self._array_msg(raw_residual))
        self.pub_comp_res.publish(self._array_msg(comp_residual))
        self.pub_rhat.publish(self._array_msg(rhat))

    def tick(self) -> None:
        if not self.have_odom:
            return
        if self.emergency:
            self.publish_thrusters(np.zeros(6))
            return

        t = self.now_sec()
        ref, ref_vel, elapsed, done = self.reference(t)

        # Update model-aided wrench residual estimate using the previous command.
        self.update_reconciliation()

        tau, err = self.control_wrench(ref, ref_vel)
        rhat = self.reconciler.rhat if self.wrench_reconciliation_enabled else np.zeros(4, dtype=float)
        tau_target = tau - rhat
        tau_target[0:2] = np.clip(tau_target[0:2], -float(self.get_parameter("tau_xy_max").value), float(self.get_parameter("tau_xy_max").value))
        tau_target[2] = float(np.clip(tau_target[2], -float(self.get_parameter("tau_z_max").value), float(self.get_parameter("tau_z_max").value)))
        tau_target[3] = float(np.clip(tau_target[3], -float(self.get_parameter("tau_yaw_max").value), float(self.get_parameter("tau_yaw_max").value)))
        self.last_tau_target = tau_target.copy()

        u = self.allocate(tau_target)
        self.publish_thrusters(u)
        self.u_prev = u.copy()

        if done:
            self.complete_mission(ref)

        debug_period = float(self.get_parameter("debug_period_sec").value)
        if t - self.last_debug_time >= debug_period:
            self.last_debug_time = t
            self.get_logger().info(
                f"mission={self.mission.kind} ref=[{ref[0]:+.2f},{ref[1]:+.2f},{ref[2]:+.2f},{ref[3]:+.2f}] "
                f"pose=[{self.pose[0]:+.2f},{self.pose[1]:+.2f},{self.pose[2]:+.2f},{self.pose[3]:+.2f}] "
                f"err=[{err[0]:+.2f},{err[1]:+.2f},{err[2]:+.2f},{err[3]:+.2f}] "
                f"tau=[{tau[0]:+.1f},{tau[1]:+.1f},{tau[2]:+.1f},{tau[3]:+.1f}] "
                f"target=[{self.last_tau_target[0]:+.1f},{self.last_tau_target[1]:+.1f},{self.last_tau_target[2]:+.1f},{self.last_tau_target[3]:+.1f}] "
                f"rhat=[{self.reconciler.rhat[0]:+.2f},{self.reconciler.rhat[1]:+.2f},{self.reconciler.rhat[2]:+.2f},{self.reconciler.rhat[3]:+.2f}] "
                f"u=[{','.join(f'{x:+.1f}' for x in u)}]"
            )

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                f"{t:.6f}", self.mission.kind,
                *[f"{x:.6f}" for x in ref],
                *[f"{x:.6f}" for x in self.pose],
                *[f"{x:.6f}" for x in err],
                *[f"{x:.6f}" for x in tau],
                *[f"{x:.6f}" for x in self.last_tau_target],
                *[f"{x:.6f}" for x in self.last_tau_est],
                *[f"{x:.6f}" for x in self.last_raw_residual],
                *[f"{x:.6f}" for x in self.reconciler.rhat],
                *[f"{x:.6f}" for x in self.last_comp_residual],
                f"{self.last_sensor_confidence:.6f}",
                int(self.wrench_reconciliation_enabled),
                int(self.reconciler.last_update_enabled),
                *[f"{x:.6f}" for x in u],
            ])
            if self.csv_file is not None:
                self.csv_file.flush()

    def destroy_node(self) -> None:
        try:
            self.publish_thrusters(np.zeros(6))
        except Exception:
            pass
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueROV2PathController()
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
