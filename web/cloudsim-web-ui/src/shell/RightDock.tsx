/// @file RightDock.tsx
/// @brief 右坞：工作区 / AI / 点云 / 几何；设备子 Tab 主路径 + 更多

import { useMemo, useState } from "react";
import UnitsTree from "../docks/workspace/UnitsTree";
import AnnotationsPanel from "../docks/workspace/AnnotationsPanel";
import InstructionPanel from "../docks/robot/InstructionPanel";
import JointAxesPanel from "../docks/robot/JointAxesPanel";
import TrajectoryGenPanel from "../docks/robot/TrajectoryGenPanel";
import TrajectoryEditPanel from "../docks/robot/TrajectoryEditPanel";
import FramesPanel from "../docks/robot/FramesPanel";
import CollisionPanel from "../docks/robot/CollisionPanel";
import ExternalAxesPanel from "../docks/robot/ExternalAxesPanel";
import CommPanel from "../docks/robot/CommPanel";
import DeviceCommandPanel from "../docks/devices/DeviceCommandPanel";
import AiPanel from "../docks/ai/AiPanel";
import PointCloudPanel from "../docks/cloud/PointCloudPanel";
import GeometryPanel from "../docks/geometry/GeometryPanel";
import { useDockNav, ROBOT_MORE_TABS, ROBOT_PRIMARY_TABS, type RobotTab } from "../state/dockNavStore";
import { useProject } from "../state/projectStore";

function RobotPanel({ robot }: { robot: RobotTab }) {
  switch (robot) {
    case "cmd":
      return <InstructionPanel />;
    case "joint":
      return <JointAxesPanel />;
    case "trajGen":
      return <TrajectoryGenPanel />;
    case "trajEdit":
      return <TrajectoryEditPanel />;
    case "frame":
      return <FramesPanel />;
    case "extAxis":
      return <ExternalAxesPanel />;
    case "collision":
      return <CollisionPanel />;
    case "comm":
      return <CommPanel />;
    default:
      return null;
  }
}

export default function RightDock() {
  const { mode } = useProject();
  const {
    primary,
    setPrimary,
    ws,
    setWs,
    deviceMode,
    setDeviceMode,
    robot,
    setRobot,
    deviceTab,
    setDeviceTab,
  } = useDockNav();
  const [moreOpen, setMoreOpen] = useState(false);

  const robotInMore = useMemo(() => ROBOT_MORE_TABS.some((t) => t.id === robot), [robot]);

  if (mode === "geomodeling") {
    return (
      <aside className="right dock">
        <div className="dock-tabs primary">
          <span className="tab active">AI 助手</span>
        </div>
        <AiPanel />
      </aside>
    );
  }

  return (
    <aside className="right dock">
      <div className="dock-tabs primary">
        <button type="button" className={`tab ${primary === "workspace" ? "active" : ""}`} onClick={() => setPrimary("workspace")}>
          工作区
        </button>
        <button type="button" className={`tab ${primary === "ai" ? "active" : ""}`} onClick={() => setPrimary("ai")}>
          AI 助手
        </button>
        <button type="button" className={`tab ${primary === "cloud" ? "active" : ""}`} onClick={() => setPrimary("cloud")}>
          点云
        </button>
        <button type="button" className={`tab ${primary === "geometry" ? "active" : ""}`} onClick={() => setPrimary("geometry")}>
          几何
        </button>
      </div>

      {primary === "ai" && <AiPanel />}
      {primary === "cloud" && <PointCloudPanel />}
      {primary === "geometry" && <GeometryPanel />}
      {primary === "workspace" && (
        <div className="dock-stack">
          <div className="dock-tabs secondary">
            <button type="button" className={`tab ${ws === "units" ? "active" : ""}`} onClick={() => setWs("units")}>
              单元部件
            </button>
            <button type="button" className={`tab ${ws === "devices" ? "active" : ""}`} onClick={() => setWs("devices")}>
              设备
            </button>
            <button type="button" className={`tab ${ws === "annotations" ? "active" : ""}`} onClick={() => setWs("annotations")}>
              装配标注
            </button>
          </div>
          {ws === "units" ? (
            <div className="dock-body">
              <UnitsTree />
            </div>
          ) : ws === "annotations" ? (
            <AnnotationsPanel />
          ) : (
            <div className="dock-body">
              <div className="dock-tabs tertiary mode-bar">
                <button
                  type="button"
                  className={`tab ${deviceMode === "robot" ? "active" : ""}`}
                  onClick={() => setDeviceMode("robot")}
                >
                  机器人
                </button>
                <button
                  type="button"
                  className={`tab ${deviceMode === "customDevice" ? "active" : ""}`}
                  onClick={() => setDeviceMode("customDevice")}
                >
                  自定义设备
                </button>
              </div>
              {deviceMode === "robot" ? (
                <>
                  <div className="dock-tabs tertiary">
                    {ROBOT_PRIMARY_TABS.map((t) => (
                      <button
                        key={t.id}
                        type="button"
                        className={`tab ${robot === t.id ? "active" : ""}`}
                        onClick={() => {
                          setMoreOpen(false);
                          setRobot(t.id);
                        }}
                      >
                        {t.label}
                      </button>
                    ))}
                    <button
                      type="button"
                      className={`tab ${robotInMore || moreOpen ? "active" : ""}`}
                      onClick={() => setMoreOpen((v) => !v)}
                      title="坐标系 / 外轴 / 碰撞 / 通讯"
                    >
                      更多{moreOpen ? "▾" : "▸"}
                    </button>
                  </div>
                  {moreOpen || robotInMore ? (
                    <div className="dock-tabs tertiary">
                      {ROBOT_MORE_TABS.map((t) => (
                        <button
                          key={t.id}
                          type="button"
                          className={`tab ${robot === t.id ? "active" : ""}`}
                          onClick={() => {
                            setRobot(t.id);
                            setMoreOpen(true);
                          }}
                        >
                          {t.label}
                        </button>
                      ))}
                    </div>
                  ) : null}
                  <RobotPanel robot={robot} />
                </>
              ) : (
                <>
                  <div className="dock-tabs tertiary">
                    <button
                      type="button"
                      className={`tab ${deviceTab === "cmd" ? "active" : ""}`}
                      onClick={() => setDeviceTab("cmd")}
                    >
                      设备指令
                    </button>
                    <button
                      type="button"
                      className={`tab ${deviceTab === "joint" ? "active" : ""}`}
                      onClick={() => setDeviceTab("joint")}
                    >
                      轴控制
                    </button>
                  </div>
                  {deviceTab === "cmd" && <DeviceCommandPanel />}
                  {deviceTab === "joint" && <JointAxesPanel preferCustomDevice />}
                </>
              )}
            </div>
          )}
        </div>
      )}
    </aside>
  );
}
