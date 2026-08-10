import { useCallback, useEffect, useMemo, useState } from "react";
import {
  cropPointCloud,
  downsamplePointCloud,
  fetchPointCloudInfo,
  fetchPointCloudMeasure,
  meshExportPly,
  meshPostPointCloud,
  preprocessPointCloud,
  reconstructPointCloud,
  registerPointCloud,
  surfaceReset,
  surfaceRun,
  type PointCloudInfo,
  type PointCloudMeasure,
} from "../../api/pointcloud";
import { importObject } from "../../api/objects";
import { dialogOpen } from "../../api/project";
import { usePointCloud } from "../../state/pointCloudStore";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";

type RegMethod = "icp" | "spare" | "sdf";

function Collapse({ title, children, defaultOpen = false }: { title: string; children: React.ReactNode; defaultOpen?: boolean }) {
  return (
    <details className="adv" open={defaultOpen}>
      <summary>{title}</summary>
      <div style={{ display: "flex", flexDirection: "column", gap: 6, marginTop: 6 }}>{children}</div>
    </details>
  );
}

export default function PointCloudPanel() {
  const { objects, selectedId, refreshObjects, selectObject, requestFocus } = useScene();
  const { setStatus } = useStatus();
  const { busy, setBusy, setPolylinePickActive, polylinePickActive, polylineScreenXy, clearPolyline, bumpRenderRevision } =
    usePointCloud();

  const pointClouds = useMemo(() => objects.filter((o) => o.geometryKind === 1), [objects]);
  const meshes = useMemo(() => objects.filter((o) => o.hasGeometry && o.geometryKind !== 1), [objects]);
  const regTargets = useMemo(() => [...pointClouds, ...meshes], [pointClouds, meshes]);
  const selected = objects.find((o) => o.id === selectedId) || null;
  const selectedPc = selected?.geometryKind === 1 ? selected : null;

  const [info, setInfo] = useState<string>("请选择点云");
  const [pcInfo, setPcInfo] = useState<PointCloudInfo | null>(null);
  const [pcMeasure, setPcMeasure] = useState<PointCloudMeasure | null>(null);

  // 下采样 — 对齐桌面默认
  const [voxelMm, setVoxelMm] = useState(2);
  const [randomFrac, setRandomFrac] = useState(0.5);

  // 裁剪
  const [keepInside, setKeepInside] = useState(true);

  // 配准
  const [regMethod, setRegMethod] = useState<RegMethod>("icp");
  const [icpTargetId, setIcpTargetId] = useState("");
  const [spareSourceId, setSpareSourceId] = useState("");
  const [spareTargetId, setSpareTargetId] = useState("");
  const [spareVoxelMm, setSpareVoxelMm] = useState(0);
  const [spareRigidPreAlign, setSpareRigidPreAlign] = useState(false);
  const [spareCreateNew, setSpareCreateNew] = useState(false);
  const [sdfFieldMode, setSdfFieldMode] = useState(1);
  const [sdfFieldVoxelMm, setSdfFieldVoxelMm] = useState(0);
  const [sdfFineDataTerm, setSdfFineDataTerm] = useState(0);
  const [sdfRigidPreAlign, setSdfRigidPreAlign] = useState(true);
  const [sdfCreateNew, setSdfCreateNew] = useState(false);

  // 重建
  const [prefilterMm, setPrefilterMm] = useState(1);
  const [meshTargetId, setMeshTargetId] = useState("");

  // 网格后处理 — 对齐桌面默认
  const [simplifyFaces, setSimplifyFaces] = useState(10000);
  const [simplifyQuality, setSimplifyQuality] = useState(0.3);
  const [smoothIters, setSmoothIters] = useState(3);
  const [smoothLambda, setSmoothLambda] = useState(0.2);
  const [repairBeforeSmooth, setRepairBeforeSmooth] = useState(false);
  const [fillHoles, setFillHoles] = useState(false);
  const [holeMaxEdges, setHoleMaxEdges] = useState(30);
  const [remeshEdgeMm, setRemeshEdgeMm] = useState(2);

  const [surfaceStageDone, setSurfaceStageDone] = useState(0);

  const importPointCloud = useCallback(async () => {
    const d = await dialogOpen({
      purpose: "pointcloud",
      title: "导入点云",
      filter: "点云 (*.ply *.xyz *.pcd *.las *.laz);;所有文件 (*.*)",
    });
    if (!d.ok || !d.path) return;
    setBusy(true);
    try {
      const r = await importObject(d.path, true);
      setStatus(r.ok ? "点云导入成功" : r.error || "导入失败", r.ok ? "info" : "err");
      if (r.ok) {
        await refreshObjects();
        bumpRenderRevision();
        if (r.id) await selectObject(r.id);
        window.setTimeout(() => requestFocus(), 400);
      }
    } finally {
      setBusy(false);
    }
  }, [bumpRenderRevision, refreshObjects, requestFocus, selectObject, setBusy, setStatus]);

  const runJob = useCallback(
    async (label: string, fn: () => Promise<{ ok: boolean; error?: string }>) => {
      setBusy(true);
      setStatus(`${label}…`);
      try {
        const r = await fn();
        setStatus(r.ok ? `${label} 完成` : r.error || `${label} 失败`, r.ok ? "info" : "err");
        if (r.ok) await refreshObjects();
        if (r.ok) bumpRenderRevision();
        return r.ok;
      } finally {
        setBusy(false);
      }
    },
    [bumpRenderRevision, refreshObjects, setBusy, setStatus],
  );

  const refreshInfo = useCallback(async () => {
    if (!selectedPc) {
      setInfo("请选择点云");
      setPcInfo(null);
      setPcMeasure(null);
      return;
    }
    const [i, m] = await Promise.all([fetchPointCloudInfo(selectedPc.id), fetchPointCloudMeasure(selectedPc.id)]);
    if (!i.ok || !i.info) {
      setInfo(i.error || "查询失败");
      setPcInfo(null);
      return;
    }
    setPcInfo(i.info);
    setPcMeasure(m.ok && m.measure ? m.measure : null);
    const lines: string[] = [`点数: ${i.info.pointCount}`];
    const b = i.info.bounds?.valid ? i.info.bounds : m.measure?.bounds;
    if (b?.valid && b.minMm && b.maxMm && b.minMm.length >= 3 && b.maxMm.length >= 3) {
      lines.push(
        `包围盒 min: (${b.minMm[0].toFixed(2)}, ${b.minMm[1].toFixed(2)}, ${b.minMm[2].toFixed(2)})`,
        `包围盒 max: (${b.maxMm[0].toFixed(2)}, ${b.maxMm[1].toFixed(2)}, ${b.maxMm[2].toFixed(2)})`,
      );
    }
    if (m.ok && m.measure?.centroidMm) {
      const c = m.measure.centroidMm;
      lines.push(`质心(mm): (${c[0].toFixed(2)}, ${c[1].toFixed(2)}, ${c[2].toFixed(2)})`);
    }
    if (m.ok && m.measure?.averageSpacingMm != null) {
      lines.push(`平均间距: ${m.measure.averageSpacingMm.toFixed(4)} mm`);
    }
    lines.push(`法线: ${i.info.hasPointNormals ? "有" : "无"}`);
    setInfo(lines.join("\n"));
  }, [selectedPc]);

  useEffect(() => {
    void refreshInfo();
  }, [refreshInfo, selectedPc?.id]);

  useEffect(() => {
    if (selectedPc && (!spareSourceId || !pointClouds.some((p) => p.id === spareSourceId))) {
      setSpareSourceId(selectedPc.id);
    }
    if (!icpTargetId || !pointClouds.some((p) => p.id === icpTargetId)) {
      const other = pointClouds.find((p) => p.id !== selectedPc?.id);
      setIcpTargetId(other?.id || "");
    }
    if (!spareTargetId || !regTargets.some((o) => o.id === spareTargetId)) {
      const other = regTargets.find((o) => o.id !== (spareSourceId || selectedPc?.id));
      setSpareTargetId(other?.id || "");
    }
    if (!meshTargetId && meshes.length > 0) setMeshTargetId(meshes[0].id);
  }, [pointClouds, meshes, regTargets, selectedPc, icpTargetId, spareSourceId, spareTargetId, meshTargetId]);

  const pickContext = () =>
    (window as unknown as { cloudsimViewportPick?: () => { mvpMatrix: number[]; viewportWidth: number; viewportHeight: number } | null })
      .cloudsimViewportPick?.();

  const modelToWorld = () => {
    const wm = selectedPc?.worldMatrix;
    if (wm && wm.length >= 16) return wm;
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  };

  const applyPolylineCrop = useCallback(() => {
    if (!selectedPc || polylineScreenXy.length < 6) return;
    const ctx = pickContext();
    if (!ctx) {
      setStatus("视口矩阵不可用", "err");
      return;
    }
    void runJob("多边形裁剪", async () => {
      const r = await cropPointCloud({
        backendId: selectedPc.id,
        mode: "polyline",
        polylineScreenXy,
        mvpMatrix: ctx.mvpMatrix,
        modelToWorld: modelToWorld(),
        viewportWidth: ctx.viewportWidth,
        viewportHeight: ctx.viewportHeight,
        keepInside,
      });
      setPolylinePickActive(false);
      clearPolyline();
      return r;
    });
  }, [
    clearPolyline,
    keepInside,
    polylineScreenXy,
    runJob,
    selectedPc,
    setPolylinePickActive,
    setStatus,
  ]);

  const runRegister = () => {
    if (regMethod === "icp") {
      if (!selectedPc || !icpTargetId || icpTargetId === selectedPc.id) {
        setStatus("请为 ICP 选择不同的目标点云", "warn");
        return;
      }
      void runJob("ICP 配准", () =>
        registerPointCloud({
          method: "icp",
          sourceId: selectedPc.id,
          targetId: icpTargetId,
          applyTransformToSource: true,
        }),
      );
      return;
    }
    const sourceId = spareSourceId || selectedPc?.id;
    if (!sourceId || !spareTargetId || spareTargetId === sourceId) {
      setStatus(regMethod === "spare" ? "请选择 SPARE 源与目标" : "请选择 SDF/DDF 源与目标", "warn");
      return;
    }
    if (regMethod === "spare") {
      if (spareCreateNew) {
        setStatus("网页端暂不支持「输出为新对象」，将就地变形源点云", "warn");
      }
      void runJob("SPARE 配准", () =>
        registerPointCloud({
          method: "spare",
          sourceId,
          targetId: spareTargetId,
          voxelPrefilterMm: spareVoxelMm,
          rigidPreAlign: spareRigidPreAlign,
          createNewObject: spareCreateNew,
        }),
      );
      return;
    }
    if (sdfCreateNew) {
      setStatus("网页端暂不支持「输出为新对象」，将就地变形源点云", "warn");
    }
    void runJob("SDF/DDF 配准", () =>
      registerPointCloud({
        method: "sdf",
        sourceId,
        targetId: spareTargetId,
        fieldMode: sdfFieldMode,
        fieldVoxelMm: sdfFieldVoxelMm,
        fineDataTerm: sdfFineDataTerm,
        rigidPreAlign: sdfRigidPreAlign,
        createNewObject: sdfCreateNew,
      }),
    );
  };

  const regBtnLabel =
    regMethod === "spare" ? "SPARE 配准" : regMethod === "sdf" ? "SDF/DDF 配准" : "ICP 配准";

  const meshPostTargetOk = Boolean(meshTargetId);

  return (
    <div className="dock-body" id="rightCloud" style={{ padding: 8, display: "flex", flexDirection: "column", gap: 8 }}>
      <Collapse title="文档与导入" defaultOpen>
        <div className="pc-row-2">
          <button type="button" disabled={busy} onClick={() => void importPointCloud()}>
            导入点云
          </button>
          <button type="button" disabled={busy} onClick={() => void refreshObjects()}>
            刷新列表
          </button>
        </div>
      </Collapse>

      <Collapse title="点云对象" defaultOpen>
        <label className="field">
          请选择点云
          <select
            value={selectedPc?.id || ""}
            onChange={(e) => {
              const id = e.target.value;
              if (id) void selectObject(id);
            }}
          >
            <option value="">请选择点云</option>
            {pointClouds.map((p) => (
              <option key={p.id} value={p.id}>
                {p.name || p.id}
              </option>
            ))}
          </select>
        </label>
        <pre style={{ fontSize: 11, whiteSpace: "pre-wrap", margin: 0 }}>{info}</pre>
        <button type="button" disabled={!selectedPc || busy} onClick={() => void refreshInfo()}>
          刷新信息
        </button>
      </Collapse>

      <Collapse title="下采样">
        <div className="pc-row">
          <label className="field">
            体素(mm):
            <input type="number" step="0.1" min={0.01} value={voxelMm} onChange={(e) => setVoxelMm(Number(e.target.value))} />
          </label>
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() =>
              void runJob("体素下采样", () =>
                downsamplePointCloud({ backendId: selectedPc!.id, mode: "voxel", voxelSizeMm: voxelMm }),
              )
            }
          >
            体素下采样
          </button>
        </div>
        <div className="pc-row">
          <label className="field">
            保留比:
            <input
              type="number"
              step="0.05"
              min={0.01}
              max={1}
              value={randomFrac}
              onChange={(e) => setRandomFrac(Number(e.target.value))}
            />
          </label>
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() =>
              void runJob("随机下采样", () =>
                downsamplePointCloud({ backendId: selectedPc!.id, mode: "random", retainedFraction: randomFrac }),
              )
            }
          >
            随机下采样
          </button>
        </div>
      </Collapse>

      <Collapse title="裁剪">
        <div className="pc-row-2">
          <button
            type="button"
            disabled={!selectedPc || busy || !pcInfo?.bounds?.valid}
            onClick={() => {
              const b = pcInfo?.bounds;
              if (!b?.minMm || !b.maxMm) {
                setStatus("无有效包围盒，无法裁剪", "warn");
                return;
              }
              void runJob("按包围盒裁剪", () =>
                cropPointCloud({
                  backendId: selectedPc!.id,
                  mode: "box",
                  box: { valid: true, minMm: b.minMm, maxMm: b.maxMm },
                }),
              );
            }}
          >
            按包围盒裁剪
          </button>
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() => {
              const c = pcMeasure?.centroidMm;
              if (!c || c.length < 3) {
                setStatus("无法获取质心，球裁剪取消", "warn");
                return;
              }
              void runJob("球裁剪", () =>
                cropPointCloud({
                  backendId: selectedPc!.id,
                  mode: "sphere",
                  centerMm: c,
                  radiusMm: 50,
                }),
              );
            }}
          >
            球裁剪 (r=50)
          </button>
        </div>
        <div className="pc-row">
          <label className="field">
            <select value={keepInside ? "1" : "0"} onChange={(e) => setKeepInside(e.target.value === "1")}>
              <option value="1">保留内部</option>
              <option value="0">删除内部</option>
            </select>
          </label>
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() => {
              if (polylinePickActive && polylineScreenXy.length >= 6) {
                applyPolylineCrop();
                return;
              }
              clearPolyline();
              setPolylinePickActive(true);
              setStatus("多边形裁剪：左键加点，右键结束（≥3 点后可再点本按钮应用）");
            }}
          >
            {polylinePickActive
              ? polylineScreenXy.length >= 6
                ? `应用多边形 (${polylineScreenXy.length / 2})`
                : "拾取中…"
              : "多边形裁剪…"}
          </button>
        </div>
      </Collapse>

      <Collapse title="预处理">
        <div className="pc-row-2">
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() =>
              void runJob("离群移除", () => preprocessPointCloud({ backendId: selectedPc!.id, op: "outliers" }))
            }
          >
            离群移除
          </button>
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() =>
              void runJob("双边平滑", () => preprocessPointCloud({ backendId: selectedPc!.id, op: "bilateral" }))
            }
          >
            双边平滑
          </button>
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() =>
              void runJob("法线 PCA", () => preprocessPointCloud({ backendId: selectedPc!.id, op: "normalsPca" }))
            }
          >
            法线 PCA
          </button>
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() =>
              void runJob("MST 定向", () => preprocessPointCloud({ backendId: selectedPc!.id, op: "normalsMst" }))
            }
          >
            MST 定向
          </button>
        </div>
      </Collapse>

      <Collapse title="配准">
        <div className="pc-row">
          <span style={{ fontSize: 12 }}>方法:</span>
          <label className="field">
            <select value={regMethod} onChange={(e) => setRegMethod(e.target.value as RegMethod)}>
              <option value="icp">刚性 ICP</option>
              <option value="spare">SPARE 非刚性</option>
              <option value="sdf">SDF/DDF 非刚性</option>
            </select>
          </label>
        </div>

        {regMethod === "icp" ? (
          <div className="pc-row">
            <span style={{ fontSize: 12, whiteSpace: "nowrap" }}>ICP 目标:</span>
            <label className="field">
              <select value={icpTargetId} onChange={(e) => setIcpTargetId(e.target.value)}>
                <option value="">—</option>
                {pointClouds
                  .filter((p) => p.id !== selectedPc?.id)
                  .map((p) => (
                    <option key={p.id} value={p.id}>
                      {p.name || p.id}
                    </option>
                  ))}
              </select>
            </label>
            <button type="button" disabled={!selectedPc || !icpTargetId || busy} onClick={runRegister}>
              {regBtnLabel}
            </button>
          </div>
        ) : (
          <>
            <div className="pc-row">
              <span style={{ fontSize: 12, whiteSpace: "nowrap" }}>非刚性源:</span>
              <label className="field">
                <select value={spareSourceId} onChange={(e) => setSpareSourceId(e.target.value)}>
                  <option value="">—</option>
                  {pointClouds.map((p) => (
                    <option key={p.id} value={p.id}>
                      [点云] {p.name || p.id}
                    </option>
                  ))}
                </select>
              </label>
            </div>
            <div className="pc-row">
              <span style={{ fontSize: 12, whiteSpace: "nowrap" }}>非刚性目标:</span>
              <label className="field">
                <select value={spareTargetId} onChange={(e) => setSpareTargetId(e.target.value)}>
                  <option value="">—</option>
                  {regTargets
                    .filter((o) => o.id !== spareSourceId)
                    .map((o) => (
                      <option key={o.id} value={o.id}>
                        {o.geometryKind === 1 ? "[点云] " : "[网格] "}
                        {o.name || o.id}
                      </option>
                    ))}
                </select>
              </label>
              <button type="button" disabled={!spareSourceId || !spareTargetId || busy} onClick={runRegister}>
                {regBtnLabel}
              </button>
            </div>
            {regMethod === "spare" && (
              <>
                <div className="pc-row">
                  <label className="field">
                    体素预滤波 (mm):
                    <input
                      type="number"
                      step="0.1"
                      min={0}
                      value={spareVoxelMm}
                      onChange={(e) => setSpareVoxelMm(Number(e.target.value))}
                    />
                  </label>
                </div>
                <label className="pc-check">
                  <input
                    type="checkbox"
                    checked={spareRigidPreAlign}
                    onChange={(e) => setSpareRigidPreAlign(e.target.checked)}
                  />
                  刚性预对齐 (ICP)
                </label>
                <label className="pc-check">
                  <input type="checkbox" checked={spareCreateNew} onChange={(e) => setSpareCreateNew(e.target.checked)} />
                  输出为新对象
                </label>
              </>
            )}
            {regMethod === "sdf" && (
              <>
                <div className="pc-row">
                  <label className="field">
                    场模式:
                    <select value={sdfFieldMode} onChange={(e) => setSdfFieldMode(Number(e.target.value))}>
                      <option value={0}>DDF 有向距离</option>
                      <option value={1}>有符号 SDF</option>
                    </select>
                  </label>
                  <label className="field">
                    场体素 (mm):
                    <input
                      type="number"
                      step="0.1"
                      min={0}
                      value={sdfFieldVoxelMm}
                      onChange={(e) => setSdfFieldVoxelMm(Number(e.target.value))}
                    />
                  </label>
                </div>
                <div className="pc-row">
                  <label className="field">
                    细阶段数据项:
                    <select value={sdfFineDataTerm} onChange={(e) => setSdfFineDataTerm(Number(e.target.value))}>
                      <option value={0}>点-面</option>
                      <option value={1}>DDF</option>
                      <option value={2}>SDF</option>
                    </select>
                  </label>
                </div>
                <label className="pc-check">
                  <input type="checkbox" checked={sdfRigidPreAlign} onChange={(e) => setSdfRigidPreAlign(e.target.checked)} />
                  刚性预对齐 (ICP)
                </label>
                <label className="pc-check">
                  <input type="checkbox" checked={sdfCreateNew} onChange={(e) => setSdfCreateNew(e.target.checked)} />
                  输出为新对象
                </label>
              </>
            )}
          </>
        )}
      </Collapse>

      <Collapse title="重建网格">
        <div className="pc-row">
          <label className="field">
            预滤波体素(mm):
            <input
              type="number"
              step="0.1"
              min={0}
              value={prefilterMm}
              onChange={(e) => setPrefilterMm(Number(e.target.value))}
            />
          </label>
        </div>
        <div className="pc-row-2">
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() =>
              void runJob("Poisson Auto", () =>
                reconstructPointCloud({
                  backendId: selectedPc!.id,
                  method: "poissonAuto",
                  voxelPrefilterMm: prefilterMm,
                  meshOptions: { displayName: "Poisson网格" },
                }),
              )
            }
          >
            Poisson Auto
          </button>
          <button
            type="button"
            disabled={!selectedPc || busy}
            onClick={() =>
              void runJob("Scale-space", () =>
                reconstructPointCloud({
                  backendId: selectedPc!.id,
                  method: "scaleSpace",
                  smoothIterations: 4,
                  meshOptions: { displayName: "ScaleSpace网格" },
                }),
              )
            }
          >
            Scale-space
          </button>
        </div>
        <div className="pc-row">
          <span style={{ fontSize: 12 }}>网格:</span>
          <label className="field">
            <select value={meshTargetId} onChange={(e) => setMeshTargetId(e.target.value)}>
              <option value="">—</option>
              {meshes.map((m) => (
                <option key={m.id} value={m.id}>
                  {m.name || m.id}
                </option>
              ))}
            </select>
          </label>
          <button
            type="button"
            disabled={!meshTargetId || busy}
            onClick={async () => {
              const d = await dialogOpen({
                purpose: "saveFile",
                title: "导出网格 PLY",
                filter: "网格 PLY (*.ply);;所有文件 (*.*)",
              });
              if (!d.ok || !d.path) return;
              void runJob("导出 PLY", () => meshExportPly({ backendId: meshTargetId, path: d.path }));
            }}
          >
            导出 PLY…
          </button>
        </div>
      </Collapse>

      <Collapse title="网格后处理">
        <div className="pc-row">
          <span style={{ fontSize: 12, whiteSpace: "nowrap" }}>网格对象:</span>
          <label className="field">
            <select value={meshTargetId} onChange={(e) => setMeshTargetId(e.target.value)}>
              <option value="">请选择网格</option>
              {meshes.map((m) => (
                <option key={m.id} value={m.id}>
                  {m.name || m.id}
                </option>
              ))}
            </select>
          </label>
        </div>

        <div className="pc-row">
          <label className="field">
            目标面数:
            <input
              type="number"
              step={1000}
              min={100}
              value={simplifyFaces}
              onChange={(e) => setSimplifyFaces(Number(e.target.value))}
            />
          </label>
          <label className="field">
            质量阈值:
            <input
              type="number"
              step={0.01}
              min={0.01}
              max={1}
              value={simplifyQuality}
              onChange={(e) => setSimplifyQuality(Number(e.target.value))}
            />
          </label>
          <button
            type="button"
            disabled={!meshPostTargetOk || busy}
            onClick={() =>
              void runJob("网格简化", () =>
                meshPostPointCloud({
                  backendId: meshTargetId,
                  op: "simplify",
                  targetFaceCount: simplifyFaces,
                  qualityThreshold: simplifyQuality,
                }),
              )
            }
          >
            网格简化
          </button>
        </div>

        <div className="pc-row">
          <label className="field">
            迭代次数:
            <input
              type="number"
              step={1}
              min={1}
              max={100}
              value={smoothIters}
              onChange={(e) => setSmoothIters(Number(e.target.value))}
            />
          </label>
          <label className="field">
            λ (Taubin):
            <input
              type="number"
              step={0.05}
              min={0.05}
              max={1}
              value={smoothLambda}
              onChange={(e) => setSmoothLambda(Number(e.target.value))}
            />
          </label>
          <button
            type="button"
            disabled={!meshPostTargetOk || busy}
            onClick={() =>
              void runJob("Laplacian 平滑", () =>
                meshPostPointCloud({
                  backendId: meshTargetId,
                  op: "smooth",
                  iterations: smoothIters,
                  lambda: smoothLambda,
                  useTaubinSmooth: false,
                  repairBeforeSmooth,
                }),
              )
            }
          >
            Laplacian 平滑
          </button>
          <button
            type="button"
            disabled={!meshPostTargetOk || busy}
            onClick={() =>
              void runJob("Taubin 平滑", () =>
                meshPostPointCloud({
                  backendId: meshTargetId,
                  op: "smooth",
                  iterations: smoothIters,
                  lambda: smoothLambda,
                  useTaubinSmooth: true,
                  repairBeforeSmooth,
                }),
              )
            }
          >
            Taubin 平滑
          </button>
        </div>
        <label className="pc-check">
          <input
            type="checkbox"
            checked={repairBeforeSmooth}
            onChange={(e) => setRepairBeforeSmooth(e.target.checked)}
          />
          平滑前修复
        </label>

        <div className="pc-row">
          <button
            type="button"
            disabled={!meshPostTargetOk || busy}
            onClick={() =>
              void runJob("网格修复", () =>
                meshPostPointCloud({
                  backendId: meshTargetId,
                  op: "repair",
                  fillHoles,
                  holeMaxEdgeCount: holeMaxEdges,
                }),
              )
            }
          >
            网格修复
          </button>
          <label className="pc-check">
            <input type="checkbox" checked={fillHoles} onChange={(e) => setFillHoles(e.target.checked)} />
            填孔
          </label>
          <label className="field">
            孔洞最大边数:
            <input
              type="number"
              step={1}
              min={3}
              max={500}
              disabled={!fillHoles}
              value={holeMaxEdges}
              onChange={(e) => setHoleMaxEdges(Number(e.target.value))}
            />
          </label>
        </div>

        <div className="pc-row">
          <label className="field">
            目标边长(mm):
            <input
              type="number"
              step={0.1}
              min={0.01}
              value={remeshEdgeMm}
              onChange={(e) => setRemeshEdgeMm(Number(e.target.value))}
            />
          </label>
          <button
            type="button"
            disabled={!meshPostTargetOk || busy}
            onClick={() =>
              void runJob("各向同性重网格", () =>
                meshPostPointCloud({
                  backendId: meshTargetId,
                  op: "remesh",
                  targetEdgeLengthMm: remeshEdgeMm,
                  iterations: 3,
                }),
              )
            }
          >
            各向同性重网格
          </button>
        </div>
      </Collapse>

      <Collapse title="曲面重构">
        <button
          type="button"
          disabled={!meshTargetId || busy}
          onClick={() =>
            void runJob("曲面重构全流程", async () => {
              const r = await surfaceRun({ backendId: meshTargetId, mode: "full", params: {} });
              if (r.ok) setSurfaceStageDone(0);
              return r;
            })
          }
        >
          全流程
        </button>
        <p className="hint">分阶段须按序：已完成 {surfaceStageDone}/8</p>
        {(
          [
            [1, "预处理"],
            [2, "分块"],
            [3, "栅格采样"],
            [4, "NURBS 拟合"],
            [5, "边界混合"],
            [6, "交汇混合"],
            [7, "光顺"],
            [8, "装配输出"],
          ] as const
        ).map(([stage, label]) => (
          <button
            key={stage}
            type="button"
            disabled={!meshTargetId || busy || surfaceStageDone !== stage - 1}
            onClick={() =>
              void runJob(label, async () => {
                const r = await surfaceRun({
                  backendId: meshTargetId,
                  mode: "stage",
                  stage,
                  params: {},
                });
                if (r.ok) setSurfaceStageDone(stage === 8 ? 0 : stage);
                return r;
              })
            }
          >
            {stage}. {label}
          </button>
        ))}
        <button
          type="button"
          disabled={busy}
          onClick={() =>
            void runJob("重置会话", async () => {
              const r = await surfaceReset({});
              if (r.ok) setSurfaceStageDone(0);
              return r;
            })
          }
        >
          重置会话
        </button>
      </Collapse>
    </div>
  );
}
