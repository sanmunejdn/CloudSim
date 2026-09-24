# -*- coding: utf-8 -*-
"""正弦关节目标示例：连 Host ExternalController 或 mock_host_server。"""

import math
import time

from cloudsim_controller import RobotController


def main():
    ctrl = RobotController()
    ctrl.connect("127.0.0.1", 19620)
    try:
        ctrl.hello(0)
        n = ctrl.joint_count
        dt = ctrl.sim_dt_ms
        t0 = time.time()
        print("jointCount=%d simDtMs=%d  Ctrl+C 退出" % (n, dt))
        while True:
            t = time.time() - t0
            # 各轴相位错开，幅度约 ±0.3 rad
            targets = [0.3 * math.sin(t + i * 0.4) for i in range(n)]
            reply = ctrl.step(dt, targets)
            print("simTimeMs=%s actual=%s" % (
                reply.get("simTimeMs"),
                ["%.3f" % x for x in reply.get("actualJointRad", [])],
            ))
            time.sleep(dt / 1000.0)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt, goodbye")
    finally:
        ctrl.goodbye()


if __name__ == "__main__":
    main()
