"""Windows 控制台 UTF-8 输出，供 CloudSim 训练子进程日志正确显示中文。"""

import sys


def configure_stdio_utf8() -> None:
    if sys.platform != 'win32':
        return
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, 'reconfigure', None)
        if callable(reconfigure):
            try:
                reconfigure(encoding='utf-8', errors='replace')
            except Exception:
                pass
