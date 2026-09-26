# CloudSimWeb headless fixture：动态端口启停

from __future__ import annotations

import os
import socket
import subprocess
import time
from pathlib import Path

import pytest
import requests

REPO_ROOT = Path(__file__).resolve().parents[2]


def _bin_dir(configuration: str) -> Path:
    return REPO_ROOT.parent / ("bin/x64d" if configuration.lower() == "debug" else "bin/x64")


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addoption(
        "--cloudsim-config",
        action="store",
        default=os.environ.get("CLOUDSIM_TEST_CONFIG", "Debug"),
        help="Debug or Release（对应 bin/x64d 或 bin/x64）",
    )


@pytest.fixture(scope="session")
def cloudsim_config(pytestconfig: pytest.Config) -> str:
    return str(pytestconfig.getoption("--cloudsim-config"))


@pytest.fixture(scope="session")
def bin_dir(cloudsim_config: str) -> Path:
    d = _bin_dir(cloudsim_config)
    if not d.is_dir():
        pytest.skip(f"bin dir missing: {d}")
    return d


@pytest.fixture(scope="session")
def cloudsim_web(bin_dir: Path, tmp_path_factory: pytest.TempPathFactory):
    exe = bin_dir / "CloudSimWeb.exe"
    if not exe.is_file():
        pytest.skip(f"CloudSimWeb.exe missing: {exe}")

    port = _free_port()
    art = tmp_path_factory.mktemp("cloudsim_web")
    log_path = art / "cloudsim_web.log"

    env = os.environ.copy()
    qt_bin = Path(os.environ.get("QTDIR", r"D:\Qt\Qt5.14.2\5.14.2\msvc2017_64")) / "bin"
    osg_bin = REPO_ROOT.parent / "bin" / "SDK" / "OSG3.6.5" / "bin"
    path_prefix = os.pathsep.join(str(p) for p in (bin_dir, qt_bin, osg_bin) if p.exists())
    env["PATH"] = path_prefix + os.pathsep + env.get("PATH", "")

    proc = subprocess.Popen(
        [str(exe), f"--port={port}"],
        cwd=str(bin_dir),
        stdout=open(log_path, "w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        env=env,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
    )
    base = f"http://127.0.0.1:{port}"
    deadline = time.time() + 60.0
    last_err = ""
    while time.time() < deadline:
        if proc.poll() is not None:
            pytest.fail(f"CloudSimWeb exited early code={proc.returncode}; log={log_path.read_text(encoding='utf-8', errors='replace')[:2000]}")
        try:
            r = requests.get(f"{base}/api/help", timeout=2)
            if r.status_code == 200:
                break
            last_err = f"status={r.status_code}"
        except requests.RequestException as e:
            last_err = str(e)
        time.sleep(0.5)
    else:
        proc.kill()
        pytest.fail(f"CloudSimWeb not ready: {last_err}; log={log_path}")

    yield {"base": base, "port": port, "bin_dir": bin_dir, "log": log_path, "proc": proc}

    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


@pytest.fixture
def api(cloudsim_web):
    return cloudsim_web["base"]


@pytest.fixture(scope="session")
def has_robot_models(bin_dir: Path) -> bool:
    return (bin_dir / "resource" / "models").is_dir()
