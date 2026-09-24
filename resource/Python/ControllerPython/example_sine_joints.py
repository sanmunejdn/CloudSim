# -*- coding: utf-8 -*-
"""正弦关节目标示例：连 Host ExternalController 或 mock_host_server。"""

from __future__ import print_function

import math
import time

from cloudsim_controller import RobotController, parse_cloudsim_host, cloudsim_robot_index


def main():
    host, port = parse_cloudsim_host()
    ctrl = RobotController()
    ctrl.connect(host, port)
    try:
        ctrl.hello(cloudsim_robot_index(0))
        n = ctrl.joint_count
        dt = ctrl.sim_dt_ms
        print(
            "jointCount=%d simDtMs=%d limitsLo=%s limitsHi=%s  Ctrl+C 退出"
            % (n, dt, ["%.2f" % x for x in ctrl.joint_lower_rad],
               ["%.2f" % x for x in ctrl.joint_upper_rad])
        )
        t0 = time.time()
        while True:
            t = time.time() - t0
            # 各轴相位错开，幅度约 ±0.3 rad（须落在限幅内）
            targets = [0.3 * math.sin(t + i * 0.4) for i in range(n)]
            reply = ctrl.step(dt, targets)
            sensors = reply.get("sensors") or {}
            joint_pos = sensors.get("jointPosition", reply.get("actualJointRad", []))
            print(
                "simTimeMs=%s sensors.jointPosition=%s"
                % (reply.get("simTimeMs"), ["%.3f" % x for x in joint_pos])
            )
            time.sleep(dt / 1000.0)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt, goodbye")
    finally:
        ctrl.goodbye()


if __name__ == "__main__":
    main()
