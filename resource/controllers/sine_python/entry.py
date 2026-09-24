# -*- coding: utf-8 -*-
"""controllers/sine_python 入口：转发到 ControllerPython 正弦示例。"""

import os
import sys

def _controller_python_dirs():
    here = os.path.abspath(os.path.dirname(__file__))
    # resource/controllers/sine_python -> ../../Python/ControllerPython
    yield os.path.normpath(os.path.join(here, "..", "..", "Python", "ControllerPython"))
    # 从 bin/x64(d)/controllers/... 回落到 resource
    app = os.environ.get("CLOUDSIM_BIN_DIR")
    if app:
        yield os.path.join(app, "resource", "Python", "ControllerPython")


for d in _controller_python_dirs():
    if os.path.isdir(d) and d not in sys.path:
        sys.path.insert(0, d)

try:
    import example_sine_joints
except ImportError as exc:
    sys.stderr.write("cannot import example_sine_joints: %s\n" % exc)
    sys.exit(1)

if __name__ == "__main__":
    example_sine_joints.main()
