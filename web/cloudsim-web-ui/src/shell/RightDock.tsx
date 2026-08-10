import UnitsTree from "../docks/workspace/UnitsTree";
import InstructionPanel from "../docks/robot/InstructionPanel";
import JointAxesPanel from "../docks/robot/JointAxesPanel";
import TrajectoryGenPanel from "../docks/robot/TrajectoryGenPanel";
import TrajectoryEditPanel from "../docks/robot/TrajectoryEditPanel";
import FramesPanel from "../docks/robot/FramesPanel";
import AiPanel from "../docks/ai/AiPanel";
import PointCloudPanel from "../docks/cloud/PointCloudPanel";
import { useDockNav } from "../state/dockNavStore";

export default function RightDock() {
  const { primary, setPrimary, ws, setWs, robot, setRobot } = useDockNav();

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
      </div>

      {primary === "ai" && <AiPanel />}
      {primary === "cloud" && <PointCloudPanel />}
      {primary === "workspace" && (
        <div className="dock-stack">
          <div className="dock-tabs secondary">
            <button type="button" className={`tab ${ws === "units" ? "active" : ""}`} onClick={() => setWs("units")}>
              单元部件
            </button>
            <button type="button" className={`tab ${ws === "robot" ? "active" : ""}`} onClick={() => setWs("robot")}>
              机器人
            </button>
          </div>
          {ws === "units" ? (
            <div className="dock-body">
              <UnitsTree />
            </div>
          ) : (
            <div className="dock-body">
              <div className="dock-tabs tertiary">
                {(
                  [
                    ["cmd", "指令"],
                    ["joint", "轴控制"],
                    ["trajGen", "轨迹生成"],
                    ["trajEdit", "轨迹编辑"],
                    ["frame", "坐标系"],
                  ] as const
                ).map(([k, label]) => (
                  <button
                    key={k}
                    type="button"
                    className={`tab ${robot === k ? "active" : ""}`}
                    onClick={() => setRobot(k)}
                  >
                    {label}
                  </button>
                ))}
              </div>
              {robot === "cmd" && <InstructionPanel />}
              {robot === "joint" && <JointAxesPanel />}
              {robot === "trajGen" && <TrajectoryGenPanel />}
              {robot === "trajEdit" && <TrajectoryEditPanel />}
              {robot === "frame" && <FramesPanel />}
            </div>
          )}
        </div>
      )}
    </aside>
  );
}
