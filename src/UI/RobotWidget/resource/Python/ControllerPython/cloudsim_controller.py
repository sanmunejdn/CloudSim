# -*- coding: utf-8 -*-
"""CloudSim External Controller 最小客户端（protocolVer=1/2）。"""

from __future__ import print_function

import json
import os
import socket


PROTOCOL_VER = 1
PROTOCOL_VER_MAX = 2
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 19620


def parse_cloudsim_host(default_host=DEFAULT_HOST, default_port=DEFAULT_PORT):
    """读 CLOUDSIM_HOST=host:port；缺省回退 default。"""
    raw = os.environ.get("CLOUDSIM_HOST", "").strip()
    if not raw:
        return default_host, int(default_port)
    if ":" in raw:
        host, port_s = raw.rsplit(":", 1)
        try:
            return host.strip() or default_host, int(port_s)
        except ValueError:
            return default_host, int(default_port)
    return raw, int(default_port)


def cloudsim_robot_index(default=0):
    try:
        return int(os.environ.get("CLOUDSIM_ROBOT_INDEX", str(default)))
    except ValueError:
        return int(default)


class ControllerError(RuntimeError):
    def __init__(self, code="", message=""):
        self.code = code
        self.message = message
        super(ControllerError, self).__init__("%s: %s" % (code, message) if code else message)


class RobotController:
    def __init__(self):
        self._sock = None
        self._buf = b""
        self.protocol_ver = PROTOCOL_VER
        self.sim_dt_ms = 16
        self.joint_count = 0
        self.joint_names = []
        self.joint_lower_rad = []
        self.joint_upper_rad = []
        self.supports_step_pose = False

    def connect(self, host=None, port=None):
        if self._sock is not None:
            raise RuntimeError("already connected")
        if host is None or port is None:
            env_host, env_port = parse_cloudsim_host()
            if host is None:
                host = env_host
            if port is None:
                port = env_port
        self._sock = socket.create_connection((host, int(port)))
        self._buf = b""

    def hello(self, robot_instance_index=None, protocol_ver=None):
        if robot_instance_index is None:
            robot_instance_index = cloudsim_robot_index(0)
        if protocol_ver is None:
            protocol_ver = PROTOCOL_VER_MAX
        self._send({
            "type": "HELLO",
            "protocolVer": int(protocol_ver),
            "robotInstanceIndex": int(robot_instance_index),
        })
        ack = self._recv_expect("HELLO_ACK")
        self.protocol_ver = int(ack.get("protocolVer", protocol_ver))
        self.sim_dt_ms = int(ack.get("simDtMs", 16))
        self.joint_count = int(ack["jointCount"])
        names = ack.get("jointNames") or []
        self.joint_names = list(names) if isinstance(names, list) else []
        self.joint_lower_rad = [float(x) for x in (ack.get("jointLowerRad") or [])]
        self.joint_upper_rad = [float(x) for x in (ack.get("jointUpperRad") or [])]
        self.supports_step_pose = bool(ack.get("supportsStepPose", False))
        return ack

    def step(self, dt_ms, target_joint_rad):
        self._send({
            "type": "STEP",
            "protocolVer": int(self.protocol_ver),
            "dtMs": int(dt_ms),
            "targetJointRad": [float(x) for x in target_joint_rad],
        })
        return self._recv_expect("STEP_REPLY")

    def step_pose(self, dt_ms, tcp_mm, euler_deg, frame="robot_base"):
        if self.protocol_ver < 2:
            raise ControllerError("PROTOCOL_MISMATCH", "step_pose requires HELLO protocolVer=2")
        self._send({
            "type": "STEP_POSE",
            "protocolVer": 2,
            "dtMs": int(dt_ms),
            "targetTcpMm": [float(x) for x in tcp_mm],
            "targetEulerDeg": [float(x) for x in euler_deg],
            "frame": frame,
        })
        return self._recv_expect("STEP_REPLY")

    def goodbye(self):
        if self._sock is None:
            return
        try:
            self._send({"type": "GOODBYE", "protocolVer": int(self.protocol_ver)})
        except OSError:
            pass
        self.close()

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            self._buf = b""

    def _send(self, obj):
        if self._sock is None:
            raise RuntimeError("not connected")
        line = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
        self._sock.sendall((line + "\n").encode("utf-8"))

    def _readline(self):
        if self._sock is None:
            raise RuntimeError("not connected")
        while True:
            nl = self._buf.find(b"\n")
            if nl >= 0:
                line = self._buf[:nl]
                self._buf = self._buf[nl + 1:]
                return line.decode("utf-8")
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionError("connection closed by peer")
            self._buf += chunk

    def _recv(self):
        msg = json.loads(self._readline())
        if not isinstance(msg, dict):
            raise ControllerError(message="expected JSON object")
        return msg

    def _recv_expect(self, expected_type, expect_ver=None):
        msg = self._recv()
        t = msg.get("type")
        if t == "ERROR":
            raise ControllerError(str(msg.get("code", "")), str(msg.get("message", "")))
        if t == "QUIT":
            raise ControllerError("QUIT", str(msg.get("reason", "")))
        if t != expected_type:
            raise ControllerError(message="expected %s, got %s" % (expected_type, t))
        pv = int(msg.get("protocolVer", -1))
        if expected_type == "HELLO_ACK":
            if pv not in (1, 2):
                raise ControllerError("PROTOCOL_MISMATCH", "protocolVer mismatch")
        elif pv != int(self.protocol_ver):
            raise ControllerError("PROTOCOL_MISMATCH", "protocolVer mismatch")
        return msg

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.goodbye()
