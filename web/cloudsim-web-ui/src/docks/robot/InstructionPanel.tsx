import { useEffect, useRef, useState } from "react";
import {
  createPathPlan,
  runProgram,
  stopProgram,
  tcpPose,
  importUrdf,
  postJoints,
  type Instruction,
} from "../../api";
import { dialogOpen } from "../../api/project";
import { useDockNav } from "../../state/dockNavStore";
import { useRobotProgram } from "../../state/robotProgramStore";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";
import { useTrajectory } from "../../state/trajectoryStore";
import {
  estimateStepDurationSec,
  planStepFrames,
  playJointFrames,
} from "../../robot/playback";
import InstructionTree, { CMD_LABEL } from "./InstructionTree";

function newId() {
  return `INS_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 7)}`;
}

const TEACH_BTNS: { type: string; label: string }[] = [
  { type: "ptp", label: "PTP" },
  { type: "line", label: "直线" },
  { type: "arc", label: "圆弧" },
  { type: "wait", label: "等待" },
  { type: "if", label: "条件" },
  { type: "while", label: "循环" },
  { type: "set_do", label: "DO" },
  { type: "set_ao", label: "AO" },
  { type: "path_plan", label: "路径" },
];

export default function InstructionPanel() {
  const {
    activeProgram,
    activeRootId,
    selectedInstrId,
    setSelectedInstrId,
    updateActiveProgram,
    playing,
    setPlaying,
    reloadPrograms,
  } = useRobotProgram();
  const { robotDragMode, setRobotDragMode, refreshObjects } = useScene();
  const { setStatus } = useStatus();
  const { goTrajGen } = useDockNav();
  const { bindPlan, reloadPathPlans, setFeatures } = useTrajectory();
  const [simRate, setSimRate] = useState(1);
  const abortRef = useRef(false);
  const [urdfPath, setUrdfPath] = useState("");
  const [jointsCsv, setJointsCsv] = useState("0,0,0,0,0,0");
  const [robotRoot, setRobotRoot] = useState(activeRootId);

  useEffect(() => {
    setRobotRoot(activeRootId);
  }, [activeRootId]);

  const steps = activeProgram?.instructions || [];
  const groups = activeProgram?.groups || [];

  const teach = async (type: string) => {
    if (!activeRootId) {
      setStatus("请先导入机器人", "warn");
      return;
    }
    if (type === "path_plan") {
      const r = await createPathPlan(activeRootId);
      if (!r.ok) {
        setStatus(r.error || "创建 PathPlan 失败", "err");
        return;
      }
      await reloadPrograms();
      if (r.pathPlanId) {
        await reloadPathPlans(activeRootId);
        setFeatures([]);
        await bindPlan(r.pathPlanId, activeRootId);
        setSelectedInstrId(r.pathPlanId);
      }
      goTrajGen();
      setStatus(`已创建 PathPlan ${r.pathPlanId || ""}，请点「开始修改」再拾取/离散`);
      return;
    }
    if (type === "wait") {
      const ins: Instruction = { id: newId(), type: "wait", name: "WAIT", durationSec: 1 };
      await updateActiveProgram((p) => ({ ...p, instructions: [...(p.instructions || []), ins] }));
      setSelectedInstrId(ins.id);
      setStatus("已添加等待");
      return;
    }
    if (type === "if" || type === "while" || type === "set_do" || type === "set_ao") {
      const ins: Instruction = {
        id: newId(),
        type,
        name: (CMD_LABEL[type] || type).toUpperCase(),
        then: type === "if" ? [] : undefined,
        else: type === "if" ? [] : undefined,
        body: type === "while" ? [] : undefined,
      };
      await updateActiveProgram((p) => ({ ...p, instructions: [...(p.instructions || []), ins] }));
      setSelectedInstrId(ins.id);
      setStatus(`已添加 ${TEACH_BTNS.find((b) => b.type === type)?.label || type}`);
      return;
    }
    const pose = await tcpPose(activeRootId);
    const ins: Instruction = {
      id: newId(),
      type,
      name: (CMD_LABEL[type] || type).toUpperCase(),
      pose: { x: pose.positionMm?.[0] || 0, y: pose.positionMm?.[1] || 0, z: pose.positionMm?.[2] || 0 },
      eulerDeg: { x: pose.eulerDeg?.[0] || 0, y: pose.eulerDeg?.[1] || 0, z: pose.eulerDeg?.[2] || 0 },
      speed: type === "ptp" ? 100 : 200,
      accel: type === "ptp" ? 100 : 200,
      blendRadius: type === "line" || type === "arc" ? 0 : undefined,
      extensions: pose.jointRadCsv ? { "context.currentJointRadCsv": pose.jointRadCsv } : undefined,
    };
    await updateActiveProgram((p) => ({ ...p, instructions: [...(p.instructions || []), ins] }));
    setSelectedInstrId(ins.id);
    setStatus(`已添加 ${TEACH_BTNS.find((b) => b.type === type)?.label || type}`);
  };

  const selectAndJump = async (id: string) => {
    setSelectedInstrId(id);
    window.dispatchEvent(new CustomEvent("cloudsim-focus-props"));
    const step = steps.find((s) => s.id === id);
    if (!step || !activeRootId) return;
    const type = String(step.type || "").toLowerCase();
    if (type === "path_plan") {
      setFeatures([]);
      await bindPlan(step.id, activeRootId);
      goTrajGen();
      setStatus(`已绑定 PathPlan ${step.id}`);
      return;
    }
    if (!["ptp", "line", "arc"].includes(type)) return;
    const planned = await planStepFrames(activeRootId, step);
    if (!planned.ok) {
      setStatus(planned.error || "跳转失败", "err");
      return;
    }
    const target = planned.frames[planned.frames.length - 1];
    const ok = await postJoints(activeRootId, target);
    if (!ok.ok) {
      setStatus("应用关节失败", "err");
      return;
    }
    setStatus(`已跳转到 ${TEACH_BTNS.find((b) => b.type === type)?.label || type}`);
  };

  const delSelected = async () => {
    if (!selectedInstrId) return;
    await updateActiveProgram((p) => ({
      ...p,
      instructions: (p.instructions || []).filter((i) => i.id !== selectedInstrId),
    }));
    setSelectedInstrId(null);
  };

  const run = async () => {
    if (playing) {
      abortRef.current = true;
      await stopProgram();
      setPlaying(false);
      setStatus("已停止运行");
      return;
    }
    if (!activeRootId) {
      setStatus("请先导入机器人", "warn");
      return;
    }
    if (!steps.length) {
      setStatus("程序无指令", "warn");
      return;
    }
    setPlaying(true);
    abortRef.current = false;
    try {
      await runProgram();
    } catch {
      /* Host 侧可选 */
    }
    setStatus("运行中…");
    for (const step of steps) {
      if (abortRef.current) break;
      setSelectedInstrId(step.id);
      const type = String(step.type || "").toLowerCase();
      if (type === "wait") {
        const sec = Number(step.durationSec) || 1;
        await new Promise((r) => setTimeout(r, (sec * 1000) / Math.max(0.1, simRate)));
        continue;
      }
      if (!["ptp", "line", "arc"].includes(type)) continue;
      const planned = await planStepFrames(activeRootId, step);
      if (!planned.ok) {
        setStatus(planned.error || `规划失败: ${step.id}`, "err");
        break;
      }
      const ok = await playJointFrames(
        activeRootId,
        planned.frames,
        estimateStepDurationSec(step, planned.frames.length),
        simRate,
        () => abortRef.current,
      );
      if (!ok && abortRef.current) break;
      if (!ok) {
        setStatus(`执行失败: ${step.id}`, "err");
        break;
      }
    }
    setPlaying(false);
    if (!abortRef.current) setStatus("运行完成");
    await reloadPrograms();
  };

  return (
    <div className="robot-pane" id="robotCmd">
      <div className="toolbar-row">
        <button type="button" className="btn-run" onClick={() => void run()}>
          {playing ? "停止" : "运行"}
        </button>
        <button
          type="button"
          className="btn-stop"
          onClick={async () => {
            abortRef.current = true;
            await stopProgram();
            setPlaying(false);
            setStatus("已停止运行");
          }}
        >
          停止
        </button>
        <label className="inline">
          仿真倍率
          <input
            type="number"
            min={0.1}
            max={10}
            step={0.1}
            value={simRate}
            onChange={(e) => setSimRate(Number(e.target.value) || 1)}
          />
          <span>x</span>
        </label>
        <button type="button" className="btn-ghost" onClick={() => setStatus("导出尚未接入", "warn")}>
          导出
        </button>
      </div>
      <div className="prog-head">
        <span>程序 {activeProgram?.name || activeProgram?.id || "Main"}</span>
        <button
          type="button"
          className="btn-ghost"
          title="规划选中指令"
          onClick={() => selectedInstrId && void selectAndJump(selectedInstrId)}
        >
          规划
        </button>
        <button type="button" className="btn-ghost danger" onClick={() => void delSelected()}>
          删除
        </button>
      </div>
      <div className="section-title">指令</div>
      <div className="cmd-grid">
        {TEACH_BTNS.map((b) => (
          <button key={b.type} type="button" data-cmd={b.type} onClick={() => void teach(b.type)}>
            {b.label}
          </button>
        ))}
      </div>
      <div className="section-title">功能</div>
      <div className="cmd-grid narrow">
        <button
          type="button"
          className={robotDragMode ? "active" : ""}
          title="开启后拖动末端罗盘做 IK；G 移动 / R 旋转"
          onClick={() => setRobotDragMode(!robotDragMode)}
        >
          拖拽
        </button>
        <button type="button" className="danger" onClick={() => void delSelected()}>
          删除
        </button>
        <button
          type="button"
          className="danger"
          onClick={async () => {
            await updateActiveProgram((p) => ({ ...p, instructions: [], groups: [] }));
            setSelectedInstrId(null);
            setStatus("程序已清空");
          }}
        >
          清空
        </button>
      </div>
      {steps.length ? (
        <InstructionTree
          steps={steps}
          groups={groups}
          selectedId={selectedInstrId}
          onSelect={(id) => void selectAndJump(id)}
        />
      ) : (
        <div className="prog-list muted">暂无指令</div>
      )}
      <details className="adv">
        <summary>URDF / 关节</summary>
        <label className="field">
          URDF
          <input
            value={urdfPath}
            placeholder="选择或粘贴路径"
            onChange={(e) => setUrdfPath(e.target.value)}
          />
        </label>
        <button
          type="button"
          onClick={async () => {
            let p = urdfPath;
            if (!p) {
              const d = await dialogOpen({ purpose: "urdf", title: "导入 URDF" });
              if (!d.ok || !d.path) return;
              p = d.path;
              setUrdfPath(p);
            }
            const r = await importUrdf(p);
            setStatus(r.ok ? "URDF 已导入" : r.error || "导入失败", r.ok ? "info" : "err");
            await refreshObjects();
            await reloadPrograms();
          }}
        >
          导入 URDF
        </button>
        <label className="field">
          sceneRootBackendId
          <input value={robotRoot} onChange={(e) => setRobotRoot(e.target.value)} />
        </label>
        <label className="field">
          关节角 rad (CSV)
          <input value={jointsCsv} onChange={(e) => setJointsCsv(e.target.value)} placeholder="0,0,0,0,0,0" />
        </label>
        <button
          type="button"
          onClick={async () => {
            const root = robotRoot || activeRootId;
            if (!root) {
              setStatus("缺少 sceneRootBackendId", "warn");
              return;
            }
            const q = jointsCsv
              .split(",")
              .map((s) => Number(s.trim()))
              .filter((n) => !Number.isNaN(n));
            const r = await postJoints(root, q);
            setStatus(r.ok ? "关节已应用" : r.error || "失败", r.ok ? "info" : "err");
          }}
        >
          应用关节
        </button>
      </details>
    </div>
  );
}
