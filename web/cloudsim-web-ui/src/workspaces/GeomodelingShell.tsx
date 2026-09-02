import { useCallback, useEffect, useState, type ReactNode } from "react";
import { dialogOpen } from "../api/project";
import {
  fetchGeomodelSummary,
  geomodelOp,
  type GeomodelBody,
} from "../api/geomodeling";
import { eventHub } from "../sse/EventHub";
import { useScene } from "../state/sceneStore";
import { useStatus } from "../state/statusStore";

type Props = { children?: ReactNode; sideVisible?: boolean };

const END_OPTIONS = ["Blind", "MidPlane", "TwoDirections", "ThroughAll"] as const;
const PLANES = ["XY", "XZ", "YZ"] as const;

type ToolId =
  | "box"
  | "cylinder"
  | "polygon"
  | "slot"
  | "ellipse"
  | "sketch"
  | "pad"
  | "pocket"
  | "sweep"
  | "sweepCut"
  | "fillet"
  | "chamfer"
  | "revolve"
  | "revolveCut"
  | "linearPattern"
  | "circularPattern"
  | "mirror3d"
  | "loft"
  | "loftCut"
  | "shell"
  | "draft";

const RIBBON: { group: string; tools: { id: ToolId; label: string }[] }[] = [
  {
    group: "草图",
    tools: [
      { id: "sketch", label: "轮廓" },
      { id: "pad", label: "拉伸" },
    ],
  },
  {
    group: "实体",
    tools: [
      { id: "box", label: "长方体" },
      { id: "cylinder", label: "圆柱" },
      { id: "polygon", label: "多边形" },
      { id: "slot", label: "槽口" },
      { id: "ellipse", label: "椭圆" },
    ],
  },
  {
    group: "特征",
    tools: [
      { id: "pocket", label: "切除" },
      { id: "sweep", label: "扫描" },
      { id: "sweepCut", label: "扫描切除" },
      { id: "fillet", label: "圆角" },
      { id: "chamfer", label: "倒角" },
      { id: "revolve", label: "旋转" },
      { id: "revolveCut", label: "旋转切除" },
    ],
  },
  {
    group: "阵列",
    tools: [
      { id: "linearPattern", label: "线性" },
      { id: "circularPattern", label: "圆周" },
      { id: "mirror3d", label: "镜像" },
      { id: "loft", label: "放样" },
      { id: "loftCut", label: "放样切除" },
      { id: "shell", label: "抽壳" },
      { id: "draft", label: "拔模" },
    ],
  },
];

const TOOL_TITLE: Record<ToolId, string> = Object.fromEntries(
  RIBBON.flatMap((g) => g.tools.map((t) => [t.id, t.label])),
) as Record<ToolId, string>;

const NEEDS_BODY = new Set<ToolId>([
  "pocket",
  "sweepCut",
  "fillet",
  "chamfer",
  "revolveCut",
  "linearPattern",
  "circularPattern",
  "mirror3d",
  "loftCut",
  "shell",
  "draft",
]);

type Params = {
  plane: (typeof PLANES)[number];
  profile: "rectangle" | "circle" | "polygon" | "slot" | "ellipse";
  L: number;
  W: number;
  H: number;
  R: number;
  Rb: number;
  sides: number;
  edges: string;
  faces: string;
  filletR: number;
  chamfer: number;
  revolveAngle: number;
  patternCount: number;
  dx: number;
  dy: number;
  dz: number;
  circAngle: number;
  shellT: number;
  draftA: number;
  loftGap: number;
  sweepLen: number;
  twist: number;
  startOffset: number;
  padDraft: number;
  endCondition: string;
  keepOriginal: boolean;
  useLastSketch: boolean;
};

const DEFAULT_PARAMS: Params = {
  plane: "XY",
  profile: "rectangle",
  L: 100,
  W: 100,
  H: 100,
  R: 50,
  Rb: 30,
  sides: 6,
  edges: "0",
  faces: "0",
  filletR: 2,
  chamfer: 1,
  revolveAngle: 360,
  patternCount: 2,
  dx: 20,
  dy: 0,
  dz: 0,
  circAngle: 360,
  shellT: 1,
  draftA: 5,
  loftGap: 80,
  sweepLen: 80,
  twist: 0,
  startOffset: 0,
  padDraft: 0,
  endCondition: "Blind",
  keepOriginal: true,
  useLastSketch: false,
};

function parseIndexList(s: string): number[] {
  return s
    .split(/[,，\s]+/)
    .map((x) => Number(x))
    .filter((n) => Number.isFinite(n));
}

export default function GeomodelingShell({ children, sideVisible = true }: Props) {
  const { setStatus } = useStatus();
  const { refreshObjects, selectedId } = useScene();
  const [bodies, setBodies] = useState<GeomodelBody[]>([]);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const [activeBodyId, setActiveBodyId] = useState("");
  const [activeFeatureId, setActiveFeatureId] = useState("");
  const [tool, setTool] = useState<ToolId>("box");
  const [p, setP] = useState<Params>(DEFAULT_PARAMS);
  const [undoCount, setUndoCount] = useState(0);
  const [redoCount, setRedoCount] = useState(0);
  const [editLength, setEditLength] = useState<number | "">("");
  const [editRadius, setEditRadius] = useState<number | "">("");
  const [editEnd, setEditEnd] = useState("Blind");
  const [editSuppressed, setEditSuppressed] = useState(false);
  const [editVisible, setEditVisible] = useState(true);

  const setNum = (key: keyof Params, raw: string, fallback: number) => {
    const n = Number(raw);
    setP((prev) => ({ ...prev, [key]: Number.isFinite(n) ? n : fallback }));
  };

  const reload = useCallback(async () => {
    try {
      const r = await fetchGeomodelSummary();
      if (r.ok) {
        const list = Array.isArray(r.bodies) ? r.bodies : [];
        setBodies(list);
        setError("");
        setUndoCount(r.undoCount ?? 0);
        setRedoCount(r.redoCount ?? 0);
        setActiveBodyId((prev) => {
          if (prev && list.some((b) => b.backendId === prev)) return prev;
          return list[0]?.backendId || "";
        });
      } else setError(r.error || "无几何建模数据");
    } catch {
      setError("无法连接几何建模 API");
      setBodies([]);
    }
  }, []);

  useEffect(() => {
    void reload();
  }, [reload]);

  useEffect(() => {
    const off = eventHub.onAny((_d, type) => {
      if (type === "SceneChanged" || type === "BackendObjectCreated" || type === "ProjectLoaded") void reload();
    });
    return () => off();
  }, [reload]);

  const activeBody = bodies.find((b) => b.backendId === activeBodyId) || null;
  const activeFeature = activeBody?.features?.find((f) => f.id === activeFeatureId) || null;

  useEffect(() => {
    if (!activeFeature) return;
    setEditLength(activeFeature.lengthMm ?? "");
    setEditRadius(activeFeature.radiusMm ?? activeFeature.chamferDistMm ?? "");
    setEditEnd(activeFeature.endCondition || "Blind");
    setEditSuppressed(!!activeFeature.suppressed);
    setEditVisible(activeFeature.visible !== false);
  }, [activeFeature]);

  const runOp = async (label: string, body: Record<string, unknown>) => {
    setBusy(true);
    setStatus(`${label}…`);
    try {
      const r = await geomodelOp(body);
      setStatus(r.ok ? `${label} 完成` : r.error || `${label} 失败`, r.ok ? "info" : "err");
      if (r.ok) {
        if (typeof r.undoCount === "number") setUndoCount(r.undoCount);
        if (typeof r.redoCount === "number") setRedoCount(r.redoCount);
        if (r.backendId) setActiveBodyId(r.backendId);
        await refreshObjects();
        await reload();
      }
      return r.ok;
    } catch {
      setStatus(`${label} 失败`, "err");
      return false;
    } finally {
      setBusy(false);
    }
  };

  const inTree = (id: string | null | undefined) => !!id && bodies.some((b) => b.backendId === id);
  const targetBodyId = inTree(activeBodyId) ? activeBodyId : inTree(selectedId) ? selectedId || "" : "";

  const profileFields = (): Record<string, unknown> => {
    const base: Record<string, unknown> = { plane: p.plane, profile: p.profile };
    if (p.profile === "circle") base.radiusMm = p.R;
    else if (p.profile === "polygon") {
      base.radiusMm = p.R;
      base.sides = p.sides;
    } else if (p.profile === "ellipse") {
      base.radiusMm = p.R;
      base.radiusBMm = p.Rb;
    } else if (p.profile === "slot") {
      base.lengthMm = p.L;
      base.widthMm = p.W;
    } else {
      base.lengthMm = p.L;
      base.widthMm = p.W;
    }
    return base;
  };

  const runActiveTool = () => {
    if (tool === "box") {
      void runOp("长方体", {
        op: "primitive",
        kind: "box",
        plane: p.plane,
        lengthMm: p.L,
        widthMm: p.W,
        heightMm: p.H,
      });
      return;
    }
    if (tool === "cylinder") {
      void runOp("圆柱", { op: "primitive", kind: "cylinder", plane: p.plane, radiusMm: p.R, heightMm: p.H });
      return;
    }
    if (tool === "polygon") {
      void runOp("多边形", {
        op: "primitive",
        kind: "polygon",
        plane: p.plane,
        sides: p.sides,
        radiusMm: p.R,
        heightMm: p.H,
        name: "PolygonBody",
      });
      return;
    }
    if (tool === "slot") {
      void runOp("槽口", {
        op: "primitive",
        kind: "slot",
        plane: p.plane,
        lengthMm: p.L,
        widthMm: p.W,
        heightMm: p.H,
      });
      return;
    }
    if (tool === "ellipse") {
      void runOp("椭圆", {
        op: "primitive",
        kind: "ellipse",
        plane: p.plane,
        radiusMm: p.R,
        radiusBMm: p.Rb,
        heightMm: p.H,
      });
      return;
    }
    if (tool === "sketch") {
      const payload: Record<string, unknown> = { op: "append", kind: "Sketch", ...profileFields() };
      if (targetBodyId) payload.backendId = targetBodyId;
      void runOp("草图轮廓", payload);
      return;
    }
    if (tool === "pad") {
      const payload: Record<string, unknown> = {
        op: "extrude",
        mode: "pad",
        plane: p.plane,
        heightMm: p.H,
        startOffsetMm: p.startOffset,
        draftAngleDeg: p.padDraft,
        endCondition: p.endCondition,
        useLastSketch: p.useLastSketch,
        ...profileFields(),
      };
      if (targetBodyId) payload.backendId = targetBodyId;
      void runOp("拉伸", payload);
      return;
    }
    if (tool === "sweep" || tool === "revolve" || tool === "loft") {
      const kind = tool === "sweep" ? "Sweep" : tool === "revolve" ? "Revolve" : "Loft";
      const payload: Record<string, unknown> = { op: "append", kind, ...profileFields() };
      if (targetBodyId) payload.backendId = targetBodyId;
      if (tool === "sweep") {
        payload.sweepLengthMm = p.sweepLen;
        payload.twistDeg = p.twist;
      }
      if (tool === "revolve") payload.revolveAngleDeg = p.revolveAngle;
      if (tool === "loft") payload.loftGapMm = p.loftGap;
      void runOp(TOOL_TITLE[tool], payload);
      return;
    }
    if (!targetBodyId) {
      setStatus("先选一个参数化体", "warn");
      return;
    }
    if (tool === "pocket") {
      void runOp("切除", {
        op: "extrude",
        mode: "pocket",
        backendId: targetBodyId,
        plane: p.plane,
        profile: "circle",
        radiusMm: p.R,
        heightMm: p.H,
        startOffsetMm: p.startOffset,
        draftAngleDeg: p.padDraft,
        endCondition: p.endCondition,
      });
      return;
    }
    const edges = parseIndexList(p.edges);
    const faces = parseIndexList(p.faces);
    if (tool === "fillet") {
      void runOp("圆角", { op: "append", kind: "Fillet", backendId: targetBodyId, edgeIndices: edges, radiusMm: p.filletR });
      return;
    }
    if (tool === "chamfer") {
      void runOp("倒角", {
        op: "append",
        kind: "Chamfer",
        backendId: targetBodyId,
        edgeIndices: edges,
        chamferDistMm: p.chamfer,
      });
      return;
    }
    if (tool === "sweepCut") {
      void runOp("扫描切除", {
        op: "append",
        kind: "SweepCut",
        backendId: targetBodyId,
        ...profileFields(),
        sweepLengthMm: p.sweepLen,
        twistDeg: p.twist,
      });
      return;
    }
    if (tool === "revolveCut") {
      void runOp("旋转切除", {
        op: "append",
        kind: "RevolveCut",
        backendId: targetBodyId,
        ...profileFields(),
        revolveAngleDeg: p.revolveAngle,
      });
      return;
    }
    if (tool === "linearPattern") {
      void runOp("线性阵列", {
        op: "append",
        kind: "LinearPattern",
        backendId: targetBodyId,
        patternCount: p.patternCount,
        patternD: [p.dx, p.dy, p.dz],
        sourceFeatureId: activeFeatureId,
      });
      return;
    }
    if (tool === "circularPattern") {
      void runOp("圆周阵列", {
        op: "append",
        kind: "CircularPattern",
        backendId: targetBodyId,
        patternCount: p.patternCount,
        patternAngleDeg: p.circAngle,
        sourceFeatureId: activeFeatureId,
        plane: p.plane,
      });
      return;
    }
    if (tool === "mirror3d") {
      void runOp("镜像", {
        op: "append",
        kind: "Mirror3D",
        backendId: targetBodyId,
        plane: p.plane,
        keepOriginal: p.keepOriginal,
      });
      return;
    }
    if (tool === "loftCut") {
      void runOp("放样切除", {
        op: "append",
        kind: "LoftCut",
        backendId: targetBodyId,
        ...profileFields(),
        loftGapMm: p.loftGap,
      });
      return;
    }
    if (tool === "shell") {
      void runOp("抽壳", {
        op: "append",
        kind: "Shell",
        backendId: targetBodyId,
        faceIndices: faces,
        shellThicknessMm: p.shellT,
      });
      return;
    }
    void runOp("拔模", {
      op: "append",
      kind: "Draft",
      backendId: targetBodyId,
      faceIndices: faces,
      draftAngleDeg: p.draftA,
      plane: p.plane,
    });
  };

  const exportHistory = async () => {
    if (!targetBodyId) {
      setStatus("先选一个参数化体", "warn");
      return;
    }
    const d = await dialogOpen({ purpose: "saveFile", title: "导出特征史", filter: "JSON (*.json)" });
    if (!d.ok || !d.path) return;
    void runOp("导出特征史", { op: "exportHistory", backendId: targetBodyId, path: d.path });
  };

  const importHistory = async (createNew: boolean) => {
    if (!createNew && !targetBodyId) {
      setStatus("替换需要先选参数化体", "warn");
      return;
    }
    const d = await dialogOpen({ purpose: "file", title: createNew ? "导入特征史（新建）" : "导入特征史（替换）", filter: "JSON (*.json)" });
    if (!d.ok || !d.path) return;
    const payload: Record<string, unknown> = { op: "importHistory", path: d.path, createNew };
    if (!createNew) payload.backendId = targetBodyId;
    void runOp(createNew ? "导入新建" : "导入替换", payload);
  };

  const showProfile = ["sketch", "pad", "sweep", "sweepCut", "revolve", "revolveCut", "loft", "loftCut"].includes(tool);
  const showSizeLW =
    tool === "box" || tool === "slot" || (showProfile && (p.profile === "rectangle" || p.profile === "slot"));
  const showRadius = tool === "cylinder" || tool === "polygon" || tool === "ellipse" || tool === "pocket"
    || (showProfile && (p.profile === "circle" || p.profile === "polygon" || p.profile === "ellipse"));

  return (
    <div className="workspace-shell geomodeling-shell">
      <section className="gm-ribbon" aria-label="几何建模">
        {RIBBON.map((g) => (
          <div key={g.group} className="gm-ribbon-group">
            <span className="gm-ribbon-label">{g.group}</span>
            <div className="gm-ribbon-btns">
              {g.tools.map((t) => (
                <button
                  key={t.id}
                  type="button"
                  className={tool === t.id ? "active" : ""}
                  disabled={busy}
                  onClick={() => setTool(t.id)}
                >
                  {t.label}
                </button>
              ))}
            </div>
          </div>
        ))}
        <div className="gm-ribbon-group">
          <span className="gm-ribbon-label">编辑</span>
          <div className="gm-ribbon-btns">
            <button type="button" className="primary" disabled={busy} onClick={runActiveTool}>
              确定
            </button>
            <button
              type="button"
              disabled={busy || !targetBodyId}
              onClick={() => void runOp("重建", { op: "rebuild", backendId: targetBodyId })}
            >
              重建
            </button>
            <button type="button" disabled={busy || undoCount <= 0} onClick={() => void runOp("撤销", { op: "undo" })}>
              撤销
            </button>
            <button type="button" disabled={busy || redoCount <= 0} onClick={() => void runOp("重做", { op: "redo" })}>
              重做
            </button>
            <button type="button" disabled={busy || !targetBodyId} onClick={() => void exportHistory()}>
              导出
            </button>
            <button type="button" disabled={busy} onClick={() => void importHistory(false)}>
              导入替换
            </button>
            <button type="button" disabled={busy} onClick={() => void importHistory(true)}>
              导入新建
            </button>
          </div>
        </div>
        <span className="gm-ribbon-meta">
          {bodies.length} 个体{activeBody ? ` · ${activeBody.name || activeBody.backendId}` : ""}
        </span>
      </section>
      <div className="geomodeling-layout">
        {sideVisible ? (
          <aside className="geomodeling-side">
            <div className="geomodeling-tree">
              <div className="gm-side-caption">特征树</div>
              {error ? <p className="hint">{error}</p> : null}
              <ul className="feature-tree workspace-list">
                {bodies.length ? (
                  bodies.map((b) => (
                    <li key={b.backendId || b.name} className={b.backendId === activeBodyId ? "gm-active-body" : ""}>
                      <button
                        type="button"
                        className="gm-tree-body"
                        onClick={() => {
                          setActiveBodyId(b.backendId || "");
                          setActiveFeatureId("");
                        }}
                      >
                        <span className="feature-tree-name">{b.name || b.backendId}</span>
                        <span className="feature-tree-meta">
                          {b.featureCount != null ? `${b.featureCount} 特征` : ""}
                          {b.hasGeometry ? " · 有几何" : ""}
                        </span>
                      </button>
                      <ul className="gm-feat-list">
                        {(b.features || []).map((f) => (
                          <li key={f.id}>
                            <button
                              type="button"
                              className={
                                f.id === activeFeatureId && b.backendId === activeBodyId ? "gm-feat active" : "gm-feat"
                              }
                              onClick={() => {
                                setActiveBodyId(b.backendId || "");
                                setActiveFeatureId(f.id);
                              }}
                            >
                              {f.kind || "Feature"} {f.suppressed ? "（抑制）" : ""}
                              {f.visible === false ? "（隐）" : ""}
                              <span className="feature-tree-meta">{f.id}</span>
                            </button>
                          </li>
                        ))}
                      </ul>
                    </li>
                  ))
                ) : (
                  <li className="hint">暂无参数化体</li>
                )}
              </ul>
            </div>
            <div className="geomodeling-props">
              {activeFeature ? (
                <>
                  <div className="gm-side-caption">{activeFeature.kind}</div>
                  <p className="hint">{activeFeature.id}</p>
                  {activeFeature.lengthMm != null ? (
                    <label className="gm-field-block">
                      深度 mm
                      <input
                        type="number"
                        value={editLength}
                        disabled={busy}
                        onChange={(e) => setEditLength(e.target.value === "" ? "" : Number(e.target.value))}
                      />
                    </label>
                  ) : null}
                  {activeFeature.endCondition ? (
                    <label className="gm-field-block">
                      终止
                      <select value={editEnd} disabled={busy} onChange={(e) => setEditEnd(e.target.value)}>
                        {END_OPTIONS.map((x) => (
                          <option key={x} value={x}>
                            {x}
                          </option>
                        ))}
                      </select>
                    </label>
                  ) : null}
                  {activeFeature.radiusMm != null || activeFeature.chamferDistMm != null ? (
                    <label className="gm-field-block">
                      {activeFeature.kind === "Chamfer" ? "倒角距 mm" : "半径 mm"}
                      <input
                        type="number"
                        min={0.01}
                        step={0.1}
                        value={editRadius}
                        disabled={busy}
                        onChange={(e) => setEditRadius(e.target.value === "" ? "" : Number(e.target.value))}
                      />
                    </label>
                  ) : null}
                  <label className="inline">
                    <input
                      type="checkbox"
                      checked={editSuppressed}
                      disabled={busy}
                      onChange={(e) => setEditSuppressed(e.target.checked)}
                    />
                    抑制
                  </label>
                  <label className="inline">
                    <input
                      type="checkbox"
                      checked={editVisible}
                      disabled={busy}
                      onChange={(e) => setEditVisible(e.target.checked)}
                    />
                    可见
                  </label>
                  <button
                    type="button"
                    className="primary"
                    disabled={busy || !activeBodyId}
                    onClick={() => {
                      const patch: Record<string, unknown> = {
                        op: "patch",
                        backendId: activeBodyId,
                        featureId: activeFeature.id,
                        suppressed: editSuppressed,
                        visible: editVisible,
                      };
                      if (editLength !== "") patch.lengthMm = editLength;
                      if (editRadius !== "") {
                        if (activeFeature.kind === "Chamfer") patch.chamferDistMm = editRadius;
                        else patch.radiusMm = editRadius;
                      }
                      if (activeFeature.endCondition) patch.endCondition = editEnd;
                      void runOp("改参数", patch);
                    }}
                  >
                    应用
                  </button>
                  <button
                    type="button"
                    disabled={busy || !activeBodyId}
                    onClick={() =>
                      void runOp("删除特征", { op: "delete", backendId: activeBodyId, featureId: activeFeature.id })
                    }
                  >
                    删除
                  </button>
                </>
              ) : (
                <>
                  <div className="gm-side-caption">{TOOL_TITLE[tool]}</div>
                  <label className="gm-field-block">
                    草图平面
                    <select
                      value={p.plane}
                      disabled={busy}
                      onChange={(e) => setP((prev) => ({ ...prev, plane: e.target.value as Params["plane"] }))}
                    >
                      {PLANES.map((x) => (
                        <option key={x} value={x}>
                          {x}
                        </option>
                      ))}
                    </select>
                  </label>
                  {showProfile ? (
                    <label className="gm-field-block">
                      轮廓
                      <select
                        value={p.profile}
                        disabled={busy}
                        onChange={(e) =>
                          setP((prev) => ({ ...prev, profile: e.target.value as Params["profile"] }))
                        }
                      >
                        <option value="rectangle">矩形</option>
                        <option value="circle">圆</option>
                        <option value="polygon">多边形</option>
                        <option value="slot">槽口</option>
                        <option value="ellipse">椭圆</option>
                      </select>
                    </label>
                  ) : null}
                  {tool === "pad" ? (
                    <label className="inline">
                      <input
                        type="checkbox"
                        checked={p.useLastSketch}
                        disabled={busy}
                        onChange={(e) => setP((prev) => ({ ...prev, useLastSketch: e.target.checked }))}
                      />
                      使用上一草图
                    </label>
                  ) : null}
                  {showSizeLW ? (
                    <>
                      <label className="gm-field-block">
                        L
                        <input type="number" min={0.1} value={p.L} disabled={busy} onChange={(e) => setNum("L", e.target.value, 100)} />
                      </label>
                      <label className="gm-field-block">
                        W
                        <input type="number" min={0.1} value={p.W} disabled={busy} onChange={(e) => setNum("W", e.target.value, 100)} />
                      </label>
                    </>
                  ) : null}
                  {showRadius ? (
                    <label className="gm-field-block">
                      R
                      <input type="number" min={0.1} value={p.R} disabled={busy} onChange={(e) => setNum("R", e.target.value, 50)} />
                    </label>
                  ) : null}
                  {tool === "ellipse" || (showProfile && p.profile === "ellipse") ? (
                    <label className="gm-field-block">
                      R2
                      <input type="number" min={0.1} value={p.Rb} disabled={busy} onChange={(e) => setNum("Rb", e.target.value, 30)} />
                    </label>
                  ) : null}
                  {tool === "polygon" || (showProfile && p.profile === "polygon") ? (
                    <label className="gm-field-block">
                      边数
                      <input type="number" min={3} max={24} value={p.sides} disabled={busy} onChange={(e) => setNum("sides", e.target.value, 6)} />
                    </label>
                  ) : null}
                  {tool === "box" || tool === "cylinder" || tool === "polygon" || tool === "slot" || tool === "ellipse"
                  || tool === "pad" || tool === "pocket" ? (
                    <label className="gm-field-block">
                      H / 深度
                      <input type="number" min={0.1} value={p.H} disabled={busy} onChange={(e) => setNum("H", e.target.value, 100)} />
                    </label>
                  ) : null}
                  {tool === "pad" || tool === "pocket" ? (
                    <>
                      <label className="gm-field-block">
                        起始偏移
                        <input type="number" value={p.startOffset} disabled={busy} onChange={(e) => setNum("startOffset", e.target.value, 0)} />
                      </label>
                      <label className="gm-field-block">
                        拔模角
                        <input type="number" value={p.padDraft} disabled={busy} onChange={(e) => setNum("padDraft", e.target.value, 0)} />
                      </label>
                      <label className="gm-field-block">
                        终止
                        <select
                          value={p.endCondition}
                          disabled={busy}
                          onChange={(e) => setP((prev) => ({ ...prev, endCondition: e.target.value }))}
                        >
                          {END_OPTIONS.map((x) => (
                            <option key={x} value={x}>
                              {x}
                            </option>
                          ))}
                        </select>
                      </label>
                    </>
                  ) : null}
                  {tool === "fillet" || tool === "chamfer" ? (
                    <>
                      <label className="gm-field-block">
                        {tool === "chamfer" ? "倒角距" : "半径"}
                        <input
                          type="number"
                          min={0.01}
                          step={0.1}
                          value={tool === "chamfer" ? p.chamfer : p.filletR}
                          disabled={busy}
                          onChange={(e) => setNum(tool === "chamfer" ? "chamfer" : "filletR", e.target.value, 1)}
                        />
                      </label>
                      <label className="gm-field-block">
                        边索引
                        <input value={p.edges} disabled={busy} onChange={(e) => setP((prev) => ({ ...prev, edges: e.target.value }))} />
                      </label>
                    </>
                  ) : null}
                  {tool === "revolve" || tool === "revolveCut" ? (
                    <label className="gm-field-block">
                      角度
                      <input type="number" value={p.revolveAngle} disabled={busy} onChange={(e) => setNum("revolveAngle", e.target.value, 360)} />
                    </label>
                  ) : null}
                  {tool === "sweep" || tool === "sweepCut" ? (
                    <>
                      <label className="gm-field-block">
                        路径长
                        <input type="number" min={0.1} value={p.sweepLen} disabled={busy} onChange={(e) => setNum("sweepLen", e.target.value, 80)} />
                      </label>
                      <label className="gm-field-block">
                        扭转°
                        <input type="number" value={p.twist} disabled={busy} onChange={(e) => setNum("twist", e.target.value, 0)} />
                      </label>
                    </>
                  ) : null}
                  {tool === "loft" || tool === "loftCut" ? (
                    <label className="gm-field-block">
                      截面间距
                      <input type="number" min={0.1} value={p.loftGap} disabled={busy} onChange={(e) => setNum("loftGap", e.target.value, 80)} />
                    </label>
                  ) : null}
                  {tool === "linearPattern" ? (
                    <>
                      <label className="gm-field-block">
                        个数
                        <input type="number" min={2} value={p.patternCount} disabled={busy} onChange={(e) => setNum("patternCount", e.target.value, 2)} />
                      </label>
                      <label className="gm-field-block">
                        dX
                        <input type="number" value={p.dx} disabled={busy} onChange={(e) => setNum("dx", e.target.value, 20)} />
                      </label>
                      <label className="gm-field-block">
                        dY
                        <input type="number" value={p.dy} disabled={busy} onChange={(e) => setNum("dy", e.target.value, 0)} />
                      </label>
                      <label className="gm-field-block">
                        dZ
                        <input type="number" value={p.dz} disabled={busy} onChange={(e) => setNum("dz", e.target.value, 0)} />
                      </label>
                    </>
                  ) : null}
                  {tool === "circularPattern" ? (
                    <>
                      <label className="gm-field-block">
                        个数
                        <input type="number" min={2} value={p.patternCount} disabled={busy} onChange={(e) => setNum("patternCount", e.target.value, 4)} />
                      </label>
                      <label className="gm-field-block">
                        总角°
                        <input type="number" value={p.circAngle} disabled={busy} onChange={(e) => setNum("circAngle", e.target.value, 360)} />
                      </label>
                    </>
                  ) : null}
                  {tool === "mirror3d" ? (
                    <label className="inline">
                      <input
                        type="checkbox"
                        checked={p.keepOriginal}
                        disabled={busy}
                        onChange={(e) => setP((prev) => ({ ...prev, keepOriginal: e.target.checked }))}
                      />
                      保留原件
                    </label>
                  ) : null}
                  {tool === "shell" || tool === "draft" ? (
                    <label className="gm-field-block">
                      面索引
                      <input value={p.faces} disabled={busy} onChange={(e) => setP((prev) => ({ ...prev, faces: e.target.value }))} />
                    </label>
                  ) : null}
                  {tool === "shell" ? (
                    <label className="gm-field-block">
                      壁厚
                      <input type="number" min={0.01} value={p.shellT} disabled={busy} onChange={(e) => setNum("shellT", e.target.value, 1)} />
                    </label>
                  ) : null}
                  {tool === "draft" ? (
                    <label className="gm-field-block">
                      拔模角
                      <input type="number" value={p.draftA} disabled={busy} onChange={(e) => setNum("draftA", e.target.value, 5)} />
                    </label>
                  ) : null}
                  {NEEDS_BODY.has(tool) && !targetBodyId ? <p className="hint">需要先有参数化体</p> : null}
                  <p className="hint">视口点选/PlaneGCS 草图仅桌面</p>
                  <button type="button" className="primary" disabled={busy} onClick={runActiveTool}>
                    确定
                  </button>
                </>
              )}
            </div>
          </aside>
        ) : null}
        <div className="geomodeling-viewport">{children}</div>
      </div>
    </div>
  );
}
