# -*- coding: utf-8 -*-
"""CloudSim External Controller 最小客户端（protocolVer=1）。"""

import json
import socket


PROTOCOL_VER = 1
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 19620


class ControllerError(RuntimeError):
    def __init__(self, code="", message=""):
        self.code = code
        self.message = message
        super().__init__("%s: %s" % (code, message) if code else message)


class RobotController:
    def __init__(self):
        self._sock = None
        self._buf = b""
        self.sim_dt_ms = 16
        self.joint_count = 0
        self.joint_names = []

    def connect(self, host=DEFAULT_HOST, port=DEFAULT_PORT):
        if self._sock is not None:
            raise RuntimeError("already connected")
        self._sock = socket.create_connection((host, port))
        self._buf = b""

    def hello(self, robot_instance_index=0):
        self._send({
            "type": "HELLO",
            "protocolVer": PROTOCOL_VER,
            "robotInstanceIndex": int(robot_instance_index),
        })
        ack = self._recv_expect("HELLO_ACK")
        self.sim_dt_ms = int(ack.get("simDtMs", 16))
        self.joint_count = int(ack["jointCount"])
        names = ack.get("jointNames") or []
        self.joint_names = list(names) if isinstance(names, list) else []
        return ack

    def step(self, dt_ms, target_joint_rad):
        self._send({
            "type": "STEP",
            "protocolVer": PROTOCOL_VER,
            "dtMs": int(dt_ms),
            "targetJointRad": [float(x) for x in target_joint_rad],
        })
        return self._recv_expect("STEP_REPLY")

    def goodbye(self):
        if self._sock is None:
            return
        try:
            self._send({"type": "GOODBYE", "protocolVer": PROTOCOL_VER})
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
        """阻塞读到一行完整 JSON（以 \\n 结束）。"""
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

    def _recv_expect(self, expected_type):
        msg = self._recv()
        t = msg.get("type")
        if t == "ERROR":
            raise ControllerError(str(msg.get("code", "")), str(msg.get("message", "")))
        if t == "QUIT":
            raise ControllerError("QUIT", str(msg.get("reason", "")))
        if t != expected_type:
            raise ControllerError(message="expected %s, got %s" % (expected_type, t))
        if int(msg.get("protocolVer", -1)) != PROTOCOL_VER:
            raise ControllerError("PROTOCOL_MISMATCH", "protocolVer mismatch")
        return msg

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.goodbye()
