# -*- coding: utf-8 -*-
"""笛卡尔 STEP_POSE 示例：相对法兰小位移往复（protocolVer=2）。"""

from __future__ import print_function

import math
import time

from cloudsim_controller import RobotController, parse_cloudsim_host, cloudsim_robot_index


def main():
    host, port = parse_cloudsim_host()
    idx = cloudsim_robot_index(0)
    ctrl = RobotController()
    ctrl.connect(host, port)
    try:
        ack = ctrl.hello(idx, protocol_ver=2)
        if not ctrl.supports_step_pose:
            print("Host 未声明 supportsStepPose，仍尝试 STEP_POSE")
        dt = ctrl.sim_dt_ms
        # 基座系下一组保守目标（按常见工业臂工作空间微调）
        base = [600.0, 0.0, 800.0]
        euler = [180.0, 0.0, 0.0]
        print("jointCount=%d supportsStepPose=%s" % (ctrl.joint_count, ctrl.supports_step_pose))
        t0 = time.time()
        while True:
            t = time.time() - t0
            tcp = [base[0] + 40.0 * math.sin(t), base[1], base[2] + 20.0 * math.sin(0.5 * t)]
            reply = ctrl.step_pose(dt, tcp, euler, "robot_base")
            print(
                "simTimeMs=%s actual[0]=%.3f"
                % (reply.get("simTimeMs"), (reply.get("actualJointRad") or [0])[0])
            )
            time.sleep(dt / 1000.0)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt, goodbye")
    finally:
        ctrl.goodbye()


if __name__ == "__main__":
    main()
