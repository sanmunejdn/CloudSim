import { useCallback, useEffect, useMemo, useState } from "react";
import {
  Background,
  Controls,
  Handle,
  Position,
  ReactFlow,
  type Connection,
  type Edge,
  type Node,
  type NodeProps,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import {
  attachCustomDeviceChildren,
  defaultRotateMotion,
  defaultTranslateMotion,
  ensureCustomDevice,
  exportCustomDeviceUrdf,
  fetchAssemblyCandidates,
  fetchCustomDevice,
  fetchMountRobotCandidates,
  mountCustomDeviceToRobot,
  postCustomDeviceAssembly,
  unmountCustomDeviceFromRobot,
  type CustomDeviceMotion,
  type CustomDeviceJointDto,
  type CustomDeviceLinkDto,
  type MountRobotCandidate,
} from "../../api/customDevices";
import { importObject, listCoordinateFrames } from "../../api";
import { dialogOpen, dialogPaths } from "../../api/project";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";

type LinkData = {
  linkId: string;
  displayName: string;
  geometryBackendId: string;
  fixed: boolean;
};

type JointEdgeData = { motion: CustomDeviceMotion };

function shortId(id: string) {
  if (!id) return "无几何";
  return id.length > 18 ? `${id.slice(0, 8)}…${id.slice(-6)}` : id;
}

function LinkNode({ data }: NodeProps) {
  const d = data as LinkData;
  return (
    <div className={`assembly-link-node ${d.fixed ? "is-fixed" : ""}`}>
      <Handle type="target" position={Position.Left} id="in" className="assembly-handle" />
      <div className="assembly-link-head">
        <span className="assembly-link-title">{d.displayName || d.linkId}</span>
        {d.fixed ? <span className="assembly-fixed-badge">固定</span> : null}
      </div>
      <div className="assembly-link-meta">{shortId(d.geometryBackendId)}</div>
      <Handle type="source" position={Position.Right} id="out" className="assembly-handle" />
    </div>
  );
}

const nodeTypes = { link: LinkNode };

function newLinkId(n: number) {
  return `L${n}`;
}
function newJointId(n: number) {
  return `J${n}`;
}

type Props = {
  open: boolean;
  /** 空 = 新建 */
  deviceId?: string;
  onClose: () => void;
  onDone: (id?: string) => void;
};

export default function CustomDeviceAssemblyDialog({ open, deviceId, onClose, onDone }: Props) {
  const { setStatus } = useStatus();
  const { refreshObjects } = useScene();
  const [name, setName] = useState("CustomDevice");
  const [workingId, setWorkingId] = useState(deviceId || "");
  const [nodes, setNodes] = useState<Node[]>([]);
  const [edges, setEdges] = useState<Edge[]>([]);
  const [selectedEdgeId, setSelectedEdgeId] = useState<string | null>(null);
  const [selectedNodeId, setSelectedNodeId] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [pickOpen, setPickOpen] = useState(false);
  const [candidates, setCandidates] = useState<{ id: string; name: string }[]>([]);
  const [picked, setPicked] = useState<string[]>([]);
  const [frameOptions, setFrameOptions] = useState<{ id: string; name: string }[]>([]);
  const [linkSeq, setLinkSeq] = useState(1);
  const [jointSeq, setJointSeq] = useState(1);
  const [connectMode, setConnectMode] = useState(false);
  const [addJointOpen, setAddJointOpen] = useState(false);
  const [jointParent, setJointParent] = useState("");
  const [jointChild, setJointChild] = useState("");
  const [jointType, setJointType] = useState<"Rotate" | "Translate">("Rotate");
  const [mountRobots, setMountRobots] = useState<MountRobotCandidate[]>([]);
  const [mountRobotId, setMountRobotId] = useState("");
  const [mountFrameId, setMountFrameId] = useState("");
  const [deviceMounted, setDeviceMounted] = useState(false);

  const selectedEdge = edges.find((e) => e.id === selectedEdgeId) || null;
  const motion: CustomDeviceMotion =
    (selectedEdge?.data as JointEdgeData | undefined)?.motion || defaultRotateMotion(selectedEdgeId || "J");

  const styledEdges = useMemo(
    () =>
      edges.map((e) => ({
        ...e,
        className: e.id === selectedEdgeId ? "assembly-edge is-selected" : "assembly-edge",
        label: String((e.data as JointEdgeData | undefined)?.motion?.motionType || e.label || "Rotate"),
        labelStyle: { fill: "#1f5f9a", fontWeight: 600, fontSize: 11 },
        labelBgStyle: { fill: "#eef5fb", fillOpacity: 0.95 },
        labelBgPadding: [4, 6] as [number, number],
        style: {
          stroke: e.id === selectedEdgeId ? "#2b79c2" : "#8aa4bf",
          strokeWidth: e.id === selectedEdgeId ? 2.5 : 1.75,
        },
      })),
    [edges, selectedEdgeId],
  );
  const centerOptions = useMemo(() => {
    const opts: { id: string; label: string }[] = [{ id: "", label: "（无）" }];
    const seen = new Set<string>();
    for (const n of nodes) {
      const d = n.data as LinkData;
      if (!d.geometryBackendId || seen.has(d.geometryBackendId)) continue;
      seen.add(d.geometryBackendId);
      opts.push({ id: d.geometryBackendId, label: `模型：${d.displayName} [${d.geometryBackendId}]` });
    }
    for (const f of frameOptions) {
      opts.push({ id: f.id, label: `坐标系：${f.name} [${f.id}]` });
    }
    return opts;
  }, [nodes, frameOptions]);

  const loadFrames = useCallback(async () => {
    try {
      const r = await listCoordinateFrames();
      const frames = (r.frames || []) as Array<{ id?: string; name?: string }>;
      setFrameOptions(
        frames
          .filter((f) => f.id)
          .map((f) => ({ id: String(f.id), name: String(f.name || f.id) })),
      );
      if (frames.length && !mountFrameId) {
        setMountFrameId(String(frames[0]?.id || ""));
      }
    } catch {
      setFrameOptions([]);
    }
  }, [mountFrameId]);

  const loadMountRobots = useCallback(async () => {
    try {
      const r = await fetchMountRobotCandidates();
      const robots = r.robots || [];
      setMountRobots(robots);
      if (robots.length && !mountRobotId) {
        setMountRobotId(robots[0].sceneBackendId);
      }
    } catch {
      setMountRobots([]);
    }
  }, [mountRobotId]);

  const loadExisting = useCallback(async () => {
    if (!deviceId) return;
    const r = await fetchCustomDevice(deviceId);
    if (!r.ok) return;
    setName(r.name || "CustomDevice");
    setWorkingId(deviceId);
    const links = (r.links || []) as CustomDeviceLinkDto[];
    const joints = (r.joints || []) as CustomDeviceJointDto[];
    let maxL = 0;
    let maxJ = 0;
    setNodes(
      links.map((l, i) => {
        const m = /^L(\d+)$/i.exec(l.id || "");
        if (m) maxL = Math.max(maxL, Number(m[1]));
        return {
          id: l.id,
          type: "link",
          position: {
            x: Number(l.canvasX) || 80 + (i % 3) * 200,
            y: Number(l.canvasY) || 80 + Math.floor(i / 3) * 140,
          },
          data: {
            linkId: l.id,
            displayName: l.displayName || l.id,
            geometryBackendId: l.geometryBackendId || "",
            fixed: !!l.fixed,
          } satisfies LinkData,
        };
      }),
    );
    setEdges(
      joints.map((j) => {
        const m = /^J(\d+)$/i.exec(j.id || "");
        if (m) maxJ = Math.max(maxJ, Number(m[1]));
        const motionIn = j.motion || defaultRotateMotion(j.id);
        return {
          id: j.id,
          source: j.parentLinkId,
          target: j.childLinkId,
          label: String(motionIn.motionType || "Rotate"),
          data: { motion: { ...defaultRotateMotion(j.id), ...motionIn } } satisfies JointEdgeData,
        };
      }),
    );
    setLinkSeq(Math.max(1, maxL + 1));
    setJointSeq(Math.max(1, maxJ + 1));
    setDeviceMounted(!!r.robotMount?.enabled);
    if (r.robotMount?.robotSceneBackendId) {
      setMountRobotId(r.robotMount.robotSceneBackendId);
    }
    if (r.robotMount?.mountFrameBackendId) {
      setMountFrameId(r.robotMount.mountFrameBackendId);
    }
  }, [deviceId]);

  useEffect(() => {
    if (!open) return;
    setSelectedEdgeId(null);
    setSelectedNodeId(null);
    setConnectMode(false);
    setAddJointOpen(false);
    void loadFrames();
    void loadMountRobots();
    if (deviceId) void loadExisting();
    else {
      setNodes([]);
      setEdges([]);
      setName("CustomDevice");
      setWorkingId("");
      setLinkSeq(1);
      setJointSeq(1);
      setDeviceMounted(false);
      setMountFrameId("");
    }
  }, [open, deviceId, loadExisting, loadFrames, loadMountRobots]);

  const createJoint = useCallback(
    (parentId: string, childId: string, type: "Rotate" | "Translate" = "Rotate") => {
      if (!parentId || !childId || parentId === childId) {
        setStatus("请选择不同的父/子 Link", "warn");
        return null;
      }
      if (edges.some((e) => e.target === childId)) {
        setStatus("每个子 Link 只能有一条入边", "warn");
        return null;
      }
      const id = newJointId(jointSeq);
      setJointSeq((s) => s + 1);
      const motion0 = type === "Translate" ? defaultTranslateMotion(id) : defaultRotateMotion(id);
      setEdges((prev) => [
        ...prev,
        {
          id,
          source: parentId,
          target: childId,
          label: type,
          data: { motion: motion0 } satisfies JointEdgeData,
        },
      ]);
      setSelectedEdgeId(id);
      setSelectedNodeId(null);
      setConnectMode(false);
      setAddJointOpen(false);
      setStatus(`已添加运动副 ${id}`);
      return id;
    },
    [edges, jointSeq, setStatus],
  );
  const ensureDevice = async (): Promise<string | null> => {
    const r = await ensureCustomDevice({ id: workingId || undefined, name });
    if (!r.ok || !r.id) {
      setStatus(r.error || "创建设备失败", "err");
      return null;
    }
    setWorkingId(r.id);
    return r.id;
  };

  const addGeometryLinks = async (geomIds: { id: string; name: string }[]) => {
    if (!geomIds.length) return;
    setBusy(true);
    const id = await ensureDevice();
    if (!id) {
      setBusy(false);
      return;
    }
    const r = await attachCustomDeviceChildren(
      id,
      geomIds.map((g) => g.id),
    );
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "挂接几何失败", "err");
      return;
    }
    await refreshObjects();
    setNodes((prev) => {
      let seq = linkSeq;
      const next = [...prev];
      const makeFirstFixed = next.length === 0;
      for (let i = 0; i < geomIds.length; ++i) {
        const g = geomIds[i];
        if (next.some((n) => (n.data as LinkData).geometryBackendId === g.id)) continue;
        const lid = newLinkId(seq++);
        next.push({
          id: lid,
          type: "link",
          position: { x: 80 + (next.length % 3) * 200, y: 80 + Math.floor(next.length / 3) * 140 },
          data: {
            linkId: lid,
            displayName: g.name || g.id,
            geometryBackendId: g.id,
            fixed: makeFirstFixed && i === 0,
          } satisfies LinkData,
        });
      }
      queueMicrotask(() => setLinkSeq(seq));
      return next;
    });
    setStatus(`已添加 ${geomIds.length} 个部件`);
  };

  const openFromScene = async () => {
    const r = await fetchAssemblyCandidates();
    if (!r.ok) {
      setStatus(r.error || "候选列表失败", "err");
      return;
    }
    setCandidates(r.objects || []);
    setPicked([]);
    setPickOpen(true);
  };

  const importModels = async () => {
    const d = await dialogOpen({ purpose: "model", title: "导入模型到组装" });
    if (!d.ok) return;
    const paths = dialogPaths(d);
    if (!paths.length) return;
    setBusy(true);
    const geoms: { id: string; name: string }[] = [];
    let fail = 0;
    for (const p of paths) {
      const r = await importObject(p, false);
      if (!r.ok || !r.id) {
        ++fail;
        continue;
      }
      const base = p.replace(/\\/g, "/").split("/").pop() || r.id;
      geoms.push({ id: r.id, name: base });
    }
    setBusy(false);
    if (!geoms.length) {
      setStatus("导入失败", "err");
      return;
    }
    await refreshObjects();
    await addGeometryLinks(geoms);
    if (fail > 0) setStatus(`已导入 ${geoms.length}，失败 ${fail}`, "warn");
  };

  const setSelectedFixed = () => {
    if (!selectedNodeId) {
      setStatus("请先选中一个 Link", "warn");
      return;
    }
    setNodes((prev) =>
      prev.map((n) => {
        const d = n.data as LinkData;
        return {
          ...n,
          data: { ...d, fixed: n.id === selectedNodeId } satisfies LinkData,
        };
      }),
    );
  };

  const removeSelected = () => {
    if (selectedEdgeId) {
      setEdges((eds) => eds.filter((e) => e.id !== selectedEdgeId));
      setSelectedEdgeId(null);
      return;
    }
    if (selectedNodeId) {
      setNodes((nds) => nds.filter((n) => n.id !== selectedNodeId));
      setEdges((eds) => eds.filter((e) => e.source !== selectedNodeId && e.target !== selectedNodeId));
      setSelectedNodeId(null);
    }
  };

  const onConnect = (c: Connection) => {
    if (!c.source || !c.target) return;
    createJoint(c.source, c.target, "Rotate");
  };

  const patchMotion = (patch: Partial<CustomDeviceMotion>) => {
    if (!selectedEdgeId) return;
    setEdges((eds) =>
      eds.map((e) => {
        if (e.id !== selectedEdgeId) return e;
        const cur = (e.data as JointEdgeData)?.motion || defaultRotateMotion(e.id);
        let next = { ...cur, ...patch };
        if (patch.motionType === "Translate" || patch.motionType === "translate") {
          next = { ...defaultTranslateMotion(e.id), ...patch, motionType: "Translate" };
        } else if (patch.motionType === "Rotate" || patch.motionType === "rotate") {
          next = { ...defaultRotateMotion(e.id), ...patch, motionType: "Rotate" };
        }
        return {
          ...e,
          label: String(next.motionType || "Rotate"),
          data: { motion: next } satisfies JointEdgeData,
        };
      }),
    );
  };

  const apply = async () => {
    if (!nodes.length || !edges.length) {
      setStatus("至少需要 1 个 Link 和 1 个 Joint", "warn");
      return;
    }
    if (nodes.some((n) => !(n.data as LinkData).geometryBackendId)) {
      setStatus("每个 Link 须绑定几何（从场景选择或导入模型）", "warn");
      return;
    }
    let fixedCount = nodes.filter((n) => (n.data as LinkData).fixed).length;
    if (fixedCount > 1) {
      setStatus("只能有一个固定 Link", "warn");
      return;
    }
    setBusy(true);
    const id = workingId || (await ensureDevice());
    if (!id) {
      setBusy(false);
      return;
    }
    const links: CustomDeviceLinkDto[] = nodes.map((n, i) => {
      const d = n.data as LinkData;
      return {
        id: n.id,
        displayName: d.displayName || n.id,
        geometryBackendId: d.geometryBackendId,
        fixed: fixedCount === 0 ? i === 0 : !!d.fixed,
        canvasX: n.position.x,
        canvasY: n.position.y,
      };
    });
    const joints: CustomDeviceJointDto[] = edges.map((e) => {
      const m = (e.data as JointEdgeData)?.motion || defaultRotateMotion(e.id);
      return {
        id: e.id,
        parentLinkId: e.source,
        childLinkId: e.target,
        motion: {
          ...m,
          motionType: String(m.motionType || "Rotate").toLowerCase().startsWith("t") ? "Translate" : "Rotate",
          jointName: m.jointName || e.id,
        },
      };
    });
    const r = await postCustomDeviceAssembly({ id, name, links, joints });
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "组装提交失败", "err");
      return;
    }
    await refreshObjects();
    setStatus("自定义设备已应用");
    onDone(r.id || id);
  };

  const doExportUrdf = async () => {
    const id = workingId || deviceId;
    if (!id) {
      setStatus("请先应用保存设备", "warn");
      return;
    }
    const d = await dialogOpen({ purpose: "directory", title: "选择 URDF 包输出目录", directory: true });
    if (!d.ok || !d.path) return;
    setBusy(true);
    const r = await exportCustomDeviceUrdf(id, d.path);
    setBusy(false);
    if (!r.ok) setStatus(r.error || "导出失败", "err");
    else setStatus(`已导出：${r.packageDir || d.path}`);
  };

  const selectedMountRobot = mountRobots.find((r) => r.sceneBackendId === mountRobotId);

  const doMount = async () => {
    const id = workingId || deviceId;
    if (!id) {
      setStatus("请先应用组装", "warn");
      return;
    }
    if (!mountRobotId) {
      setStatus("请选择机器人", "warn");
      return;
    }
    setBusy(true);
    const r = await mountCustomDeviceToRobot(id, {
      robotSceneBackendId: mountRobotId,
      flangeLinkName: selectedMountRobot?.flangeLinkName,
      flangeBackendId: selectedMountRobot?.flangeBackendId,
      mountFrameBackendId: mountFrameId || undefined,
    });
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "挂载失败", "err");
      return;
    }
    setDeviceMounted(true);
    await refreshObjects();
    setStatus("已挂载到机器人法兰");
  };

  const doUnmount = async () => {
    const id = workingId || deviceId;
    if (!id) return;
    setBusy(true);
    const r = await unmountCustomDeviceFromRobot(id);
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "解除挂载失败", "err");
      return;
    }
    setDeviceMounted(false);
    await refreshObjects();
    setStatus("已解除机器人挂载");
  };

  if (!open) return null;

  const isRotate = !String(motion.motionType || "Rotate").toLowerCase().startsWith("t");
  const childCandidates = nodes.filter(
    (n) => n.id !== jointParent && !edges.some((e) => e.target === n.id),
  );

  return (
    <div className="modal-overlay" role="dialog">
      <div className="modal-card assembly-modal">
        <div className="modal-header assembly-header">
          <div>
            <strong>{deviceId ? "编辑自定义设备" : "自定义设备组装"}</strong>
            <p className="assembly-subtitle">部件拓扑 · 运动副 · 提交烘焙</p>
          </div>
          <button type="button" className="btn-ghost" onClick={onClose}>
            关闭
          </button>
        </div>

        <div className="assembly-toolbar">
          <label className="field compact assembly-name-field">
            名称
            <input className="prop-input" value={name} onChange={(e) => setName(e.target.value)} />
          </label>
          <div className="assembly-tool-group" aria-label="几何">
            <button type="button" className="btn-ghost" disabled={busy} onClick={() => void openFromScene()}>
              从场景选择…
            </button>
            <button type="button" className="btn-ghost" disabled={busy} onClick={() => void importModels()}>
              导入模型…
            </button>
          </div>
          <div className="assembly-tool-group" aria-label="运动副">
            <button
              type="button"
              className={`btn-ghost ${connectMode ? "active" : ""}`}
              disabled={busy || nodes.length < 2}
              title="开启后：从父块右侧拖到子块左侧"
              onClick={() => {
                setConnectMode((v) => !v);
                setAddJointOpen(false);
              }}
            >
              {connectMode ? "连接中…" : "连接"}
            </button>
            <button
              type="button"
              className={`btn-ghost ${addJointOpen ? "active" : ""}`}
              disabled={busy || nodes.length < 2}
              onClick={() => {
                setAddJointOpen((v) => !v);
                setConnectMode(false);
                if (!jointParent && nodes[0]) setJointParent(nodes[0].id);
                if (!jointChild && nodes[1]) setJointChild(nodes[1].id);
              }}
            >
              添加运动副…
            </button>
          </div>
          <div className="assembly-tool-group" aria-label="编辑">
            <button type="button" className="btn-ghost" disabled={busy} onClick={removeSelected}>
              移除
            </button>
            <button type="button" className="btn-ghost" disabled={busy} onClick={setSelectedFixed}>
              设为固定
            </button>
            <button type="button" className="btn-ghost" disabled={busy} onClick={() => void doExportUrdf()}>
              导出 URDF…
            </button>
          </div>
          <button type="button" className="btn-primary" disabled={busy} onClick={() => void apply()}>
            应用
          </button>
        </div>

        {addJointOpen ? (
          <div className="assembly-joint-form">
            <label className="field compact">
              父 Link
              <select value={jointParent} onChange={(e) => setJointParent(e.target.value)}>
                <option value="">选择…</option>
                {nodes.map((n) => (
                  <option key={n.id} value={n.id}>
                    {(n.data as LinkData).displayName || n.id}
                  </option>
                ))}
              </select>
            </label>
            <label className="field compact">
              子 Link
              <select value={jointChild} onChange={(e) => setJointChild(e.target.value)}>
                <option value="">选择…</option>
                {childCandidates.map((n) => (
                  <option key={n.id} value={n.id}>
                    {(n.data as LinkData).displayName || n.id}
                  </option>
                ))}
              </select>
            </label>
            <label className="field compact">
              类型
              <select
                value={jointType}
                onChange={(e) => setJointType(e.target.value as "Rotate" | "Translate")}
              >
                <option value="Rotate">旋转副</option>
                <option value="Translate">移动副</option>
              </select>
            </label>
            <button
              type="button"
              className="btn-primary"
              disabled={!jointParent || !jointChild}
              onClick={() => createJoint(jointParent, jointChild, jointType)}
            >
              确认添加
            </button>
          </div>
        ) : null}

        <p className={`assembly-hint ${connectMode ? "is-connect" : ""}`}>
          {connectMode
            ? "连接模式：从父块右侧端口拖到子块左侧端口。再点「连接」可退出。"
            : "每个子块仅一条入边；提交前须绑定几何。可用「连接」拖拽，或「添加运动副」表单创建。"}
        </p>

        <div className="assembly-mount-panel">
          <strong>安装到机器人法兰</strong>
          <p className="hint">安装坐标系须位于设备根或 fixed 连杆下；确认后设备根通过 Follow 跟随法兰（与 TCP 对齐）。</p>
          <div className="assembly-mount-fields">
            <label className="field compact">
              机器人
              <select value={mountRobotId} onChange={(e) => setMountRobotId(e.target.value)} disabled={busy}>
                <option value="">选择…</option>
                {mountRobots.map((r) => (
                  <option key={r.sceneBackendId} value={r.sceneBackendId}>
                    {r.label || r.sceneBackendId}
                  </option>
                ))}
              </select>
            </label>
            <label className="field compact">
              法兰
              <input
                className="prop-input"
                readOnly
                value={selectedMountRobot?.flangeLinkName || "—"}
              />
            </label>
            <label className="field compact">
              安装坐标系
              <select value={mountFrameId} onChange={(e) => setMountFrameId(e.target.value)} disabled={busy}>
                <option value="">（无）</option>
                {frameOptions.map((f) => (
                  <option key={f.id} value={f.id}>
                    {f.name}
                  </option>
                ))}
              </select>
            </label>
          </div>
          <div className="assembly-tool-group">
            <button
              type="button"
              className="btn-primary"
              disabled={busy || deviceMounted || !mountRobotId || !(workingId || deviceId) || !edges.length}
              onClick={() => void doMount()}
            >
              确认挂载
            </button>
            <button
              type="button"
              className="btn-ghost"
              disabled={busy || !deviceMounted}
              onClick={() => void doUnmount()}
            >
              解除挂载
            </button>
            {deviceMounted ? <span className="hint inline">已挂载</span> : null}
          </div>
        </div>

        <div className="assembly-body">
          <div className={`assembly-canvas ${connectMode ? "is-connect" : ""}`}>
            <ReactFlow
              nodes={nodes}
              edges={styledEdges}
              nodeTypes={nodeTypes}
              nodesConnectable={connectMode}
              onConnect={onConnect}
              connectionLineStyle={{ stroke: "#2b79c2", strokeWidth: 2 }}
              onNodeClick={(_, n) => {
                setSelectedNodeId(n.id);
                setSelectedEdgeId(null);
              }}
              onEdgeClick={(_, e) => {
                setSelectedEdgeId(e.id);
                setSelectedNodeId(null);
                setAddJointOpen(false);
              }}
              onPaneClick={() => {
                setSelectedEdgeId(null);
                setSelectedNodeId(null);
              }}
              onNodesChange={(chs) => {
                setNodes((nds) => {
                  let next = [...nds];
                  for (const ch of chs) {
                    if (ch.type === "position" && "id" in ch && ch.position) {
                      next = next.map((n) => (n.id === ch.id ? { ...n, position: ch.position! } : n));
                    } else if (ch.type === "remove" && "id" in ch) {
                      next = next.filter((n) => n.id !== ch.id);
                    }
                  }
                  return next;
                });
              }}
              onEdgesChange={(chs) => {
                setEdges((eds) => {
                  let next = [...eds];
                  for (const ch of chs) {
                    if (ch.type === "remove" && "id" in ch) next = next.filter((e) => e.id !== ch.id);
                  }
                  return next;
                });
              }}
              fitView
              proOptions={{ hideAttribution: true }}
            >
              <Background gap={18} color="#d5dde8" />
              <Controls showInteractive={false} />
            </ReactFlow>
          </div>
          <aside className={`assembly-props ${selectedEdge ? "enabled" : ""}`}>
            <div className="assembly-props-title">关节属性</div>
            {!selectedEdge ? (
              <div className="assembly-props-empty">
                <p>选中一条运动副边，或用上方「添加运动副…」创建。</p>
                <ul>
                  <li>旋转副：角限位 / 轴 / 可选旋转中心</li>
                  <li>移动副：线限位 / 轴方向</li>
                </ul>
              </div>
            ) : (
              <>
                <div className="assembly-props-id">{selectedEdge.id}</div>
                <label className="field compact">
                  类型
                  <select
                    value={isRotate ? "Rotate" : "Translate"}
                    onChange={(e) => patchMotion({ motionType: e.target.value })}
                  >
                    <option value="Translate">移动副</option>
                    <option value="Rotate">旋转副</option>
                  </select>
                </label>
                <label className="field compact">
                  下限
                  <input
                    type="number"
                    step={0.01}
                    value={motion.lower ?? 0}
                    onChange={(e) => patchMotion({ lower: Number(e.target.value) })}
                  />
                </label>
                <label className="field compact">
                  上限
                  <input
                    type="number"
                    step={0.01}
                    value={motion.upper ?? 0}
                    onChange={(e) => patchMotion({ upper: Number(e.target.value) })}
                  />
                </label>
                <label className="field compact">
                  Home
                  <input
                    type="number"
                    step={0.01}
                    value={motion.home ?? 0}
                    onChange={(e) => patchMotion({ home: Number(e.target.value) })}
                  />
                </label>
                <div className="assembly-axis-row">
                  {(["X", "Y", "Z"] as const).map((lab, i) => (
                    <label key={lab} className="field compact">
                      轴{lab}
                      <input
                        type="number"
                        step={0.01}
                        value={(motion.axis || [0, 0, 1])[i] ?? 0}
                        onChange={(e) => {
                          const axis = [...(motion.axis || [0, 0, 1])];
                          axis[i] = Number(e.target.value);
                          patchMotion({ axis });
                        }}
                      />
                    </label>
                  ))}
                </div>
                {isRotate ? (
                  <label className="field compact">
                    旋转中心
                    <select
                      value={motion.motionCenterFrameBackendId || ""}
                      onChange={(e) => patchMotion({ motionCenterFrameBackendId: e.target.value })}
                    >
                      {centerOptions.map((o) => (
                        <option key={o.id || "none"} value={o.id}>
                          {o.label}
                        </option>
                      ))}
                    </select>
                  </label>
                ) : null}
              </>
            )}
          </aside>
        </div>
      </div>

      {pickOpen ? (
        <div className="modal-overlay nested" role="dialog">
          <div className="modal-card assembly-pick-card">
            <div className="modal-header">
              <strong>从场景选择</strong>
              <button type="button" className="btn-ghost" onClick={() => setPickOpen(false)}>
                取消
              </button>
            </div>
            <div className="assembly-pick-list">
              {candidates.map((c) => (
                <label key={c.id} className="assembly-pick-row">
                  <input
                    type="checkbox"
                    checked={picked.includes(c.id)}
                    onChange={(e) => {
                      setPicked((p) => (e.target.checked ? [...p, c.id] : p.filter((x) => x !== c.id)));
                    }}
                  />
                  <span>
                    {c.name} <span className="hint inline">{c.id}</span>
                  </span>
                </label>
              ))}
              {!candidates.length ? <p className="hint">场景中无可用 Mesh/STEP</p> : null}
            </div>
            <div className="dlg-actions">
              <button
                type="button"
                className="btn-primary"
                disabled={!picked.length || busy}
                onClick={async () => {
                  setPickOpen(false);
                  await addGeometryLinks(
                    picked.map((id) => {
                      const c = candidates.find((x) => x.id === id);
                      return { id, name: c?.name || id };
                    }),
                  );
                }}
              >
                添加
              </button>
            </div>
          </div>
        </div>
      ) : null}
    </div>
  );
}
