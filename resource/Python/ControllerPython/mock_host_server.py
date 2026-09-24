# -*- coding: utf-8 -*-
"""离线 mock Host：按冻结协议应答 HELLO / STEP，便于无 CloudSim 冒烟。"""

import json
import socket


HOST = "127.0.0.1"
PORT = 19620
PROTOCOL_VER = 1
JOINT_COUNT = 6
SIM_DT_MS = 16


def _send(conn, obj):
    line = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
    conn.sendall((line + "\n").encode("utf-8"))


def _readline(conn, buf):
    while True:
        nl = buf[0].find(b"\n")
        if nl >= 0:
            line = buf[0][:nl]
            buf[0] = buf[0][nl + 1:]
            return line.decode("utf-8")
        chunk = conn.recv(4096)
        if not chunk:
            return None
        buf[0] += chunk


def handle_client(conn):
    buf = [b""]
    sim_time_ms = 0
    try:
        while True:
            raw = _readline(conn, buf)
            if raw is None:
                break
            if not raw.strip():
                continue
            msg = json.loads(raw)
            t = msg.get("type")
            if t == "HELLO":
                _send(conn, {
                    "type": "HELLO_ACK",
                    "protocolVer": PROTOCOL_VER,
                    "simDtMs": SIM_DT_MS,
                    "jointCount": JOINT_COUNT,
                    "jointNames": ["j%d" % i for i in range(JOINT_COUNT)],
                })
            elif t == "STEP":
                targets = msg.get("targetJointRad") or []
                dt = int(msg.get("dtMs", SIM_DT_MS))
                sim_time_ms += dt
                # MVP：直接回显目标角
                _send(conn, {
                    "type": "STEP_REPLY",
                    "protocolVer": PROTOCOL_VER,
                    "simTimeMs": sim_time_ms,
                    "actualJointRad": [float(x) for x in targets],
                })
            elif t == "GOODBYE":
                break
            else:
                _send(conn, {
                    "type": "ERROR",
                    "protocolVer": PROTOCOL_VER,
                    "code": "INTERNAL",
                    "message": "unknown type: %s" % t,
                })
    finally:
        try:
            conn.close()
        except OSError:
            pass


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print("mock Host listening on %s:%d (Ctrl+C stop)" % (HOST, PORT), flush=True)
    try:
        while True:
            conn, addr = srv.accept()
            print("client from %s" % (addr,), flush=True)
            handle_client(conn)
            print("client closed", flush=True)
    except KeyboardInterrupt:
        print("\nstopped", flush=True)
    finally:
        srv.close()


if __name__ == "__main__":
    main()
