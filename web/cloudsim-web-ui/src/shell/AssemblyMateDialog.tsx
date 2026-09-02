import { useCallback, useEffect, useRef, useState } from "react";
import {
  applyAssemblyMate,
  restoreAssemblyMate,
  type MateFaceRef,
  type MateKind,
} from "../api/assembly";
import { useScene } from "../state/sceneStore";
import { useStatus } from "../state/statusStore";

const KIND_OPTS: { id: MateKind; label: string }[] = [
  { id: "coincident", label: "重合" },
  { id: "parallel", label: "平行" },
  { id: "perpendicular", label: "垂直" },
  { id: "tangent", label: "相切" },
  { id: "concentric", label: "同轴心" },
  { id: "lock", label: "锁定" },
  { id: "distance", label: "距离" },
  { id: "angle", label: "角度" },
];

type Props = { open: boolean; onClose: () => void };

function faceLabel(f: MateFaceRef | null) {
  if (!f) return "（未拾取）";
  return `${f.backendId} #${f.faceIndex}`;
}

export default function AssemblyMateDialog({ open, onClose }: Props) {
  const { setStatus } = useStatus();
  const { objects, refreshObjects, mateFacePickSlot, setMateFacePickSlot } = useScene();
  const [kind, setKind] = useState<MateKind>("coincident");
  const [alignment, setAlignment] = useState<"aligned" | "antiAligned">("antiAligned");
  const [distanceMm, setDistanceMm] = useState(0);
  const [angleDeg, setAngleDeg] = useState(90);
  const [face1, setFace1] = useState<MateFaceRef | null>(null);
  const [face2, setFace2] = useState<MateFaceRef | null>(null);
  const snapshotRef = useRef<number[] | null>(null);
  const previewSeqRef = useRef(0);
  const [busy, setBusy] = useState(false);

  const nameOf = useCallback(
    (id: string) => objects.find((o) => o.id === id)?.name || id,
    [objects],
  );

  const reset = useCallback(() => {
    setFace1(null);
    setFace2(null);
    snapshotRef.current = null;
    setMateFacePickSlot(null);
    setKind("coincident");
    setAlignment("antiAligned");
    setDistanceMm(0);
    setAngleDeg(90);
  }, [setMateFacePickSlot]);

  useEffect(() => {
    if (!open) {
      setMateFacePickSlot(null);
      return;
    }
    const onFace = (ev: Event) => {
      const d = (ev as CustomEvent).detail as {
        slot: 0 | 1;
        backendId: string;
        faceIndex: number;
        pickWorldMm: number[];
      };
      if (!d?.backendId || d.faceIndex < 0) return;
      const ref: MateFaceRef = {
        backendId: d.backendId,
        faceIndex: d.faceIndex,
        pickWorldMm: d.pickWorldMm,
      };
      if (d.slot === 0) {
        setFace1(ref);
        snapshotRef.current = null;
      } else {
        if (face1 && ref.backendId === face1.backendId) {
          setStatus("两面必须来自不同对象", "err");
          return;
        }
        setFace2(ref);
        snapshotRef.current = null;
      }
    };
    window.addEventListener("cloudsim-mate-face", onFace);
    return () => window.removeEventListener("cloudsim-mate-face", onFace);
  }, [open, face1, setMateFacePickSlot, setStatus]);

  const runMate = useCallback(
    async (commit: boolean) => {
      if (!face1 || !face2) return;
      const seq = ++previewSeqRef.current;
      setBusy(true);
      try {
        const r = await applyAssemblyMate({
          grounded: face1,
          moving: face2,
          kind,
          alignment,
          distanceMm,
          angleDeg,
          commit,
          movingWorldSnapshot: snapshotRef.current || undefined,
        });
        if (seq !== previewSeqRef.current && !commit) return;
        if (!r.ok) {
          setStatus(r.error || "配合失败", "err");
          return;
        }
        if (r.movingWorldSnapshot?.length === 16) {
          snapshotRef.current = r.movingWorldSnapshot;
        }
        await refreshObjects();
        if (commit) {
          setStatus("配合已应用");
          reset();
          onClose();
        }
      } finally {
        if (seq === previewSeqRef.current) setBusy(false);
      }
    },
    [face1, face2, kind, alignment, distanceMm, angleDeg, refreshObjects, setStatus, reset, onClose],
  );

  const previewKey = face1 && face2 ? `${face1.backendId}:${face1.faceIndex}|${face2.backendId}:${face2.faceIndex}|${kind}|${alignment}|${distanceMm}|${angleDeg}` : "";
  useEffect(() => {
    if (!open || !previewKey || busy) return;
    void runMate(false);
    // 参数键变化才预览；busy/snapshot 不入依赖防连环
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [open, previewKey]);

  const handleClose = useCallback(async () => {
    const snap = snapshotRef.current;
    if (snap && face2) {
      await restoreAssemblyMate(face2.backendId, snap);
      await refreshObjects();
    }
    reset();
    onClose();
  }, [face2, refreshObjects, reset, onClose]);

  if (!open) return null;

  const canApply = !!face1 && !!face2 && !busy;

  return (
    <div className="dlg-mask">
      <div className="dlg dlg-wide" onClick={(e) => e.stopPropagation()}>
        <h3>插入配合</h3>
        <p className="dlg-hint">先拾取固定面，再拾取动件面；应用后动件刚体对齐（与桌面 Mate 同 Host）</p>

        <div className="mate-kinds">
          {KIND_OPTS.map((k) => (
            <button
              key={k.id}
              type="button"
              className={kind === k.id ? "active" : ""}
              onClick={() => setKind(k.id)}
            >
              {k.label}
            </button>
          ))}
        </div>

        <div className="dlg-spins">
          <label>
            距离 mm
            <input
              type="number"
              step="0.1"
              disabled={kind !== "distance"}
              value={distanceMm}
              onChange={(e) => setDistanceMm(Number(e.target.value))}
            />
          </label>
          <label>
            角度 °
            <input
              type="number"
              step="0.1"
              disabled={kind !== "angle"}
              value={angleDeg}
              onChange={(e) => setAngleDeg(Number(e.target.value))}
            />
          </label>
        </div>

        <div className="mate-align">
          <label>
            <input
              type="radio"
              checked={alignment === "antiAligned"}
              onChange={() => setAlignment("antiAligned")}
            />
            反向对齐
          </label>
          <label>
            <input type="radio" checked={alignment === "aligned"} onChange={() => setAlignment("aligned")} />
            同向对齐
          </label>
        </div>

        <div className="mate-faces">
          <button
            type="button"
            className={mateFacePickSlot === 0 ? "active" : ""}
            onClick={() => setMateFacePickSlot(mateFacePickSlot === 0 ? null : 0)}
          >
            拾取固定面
          </button>
          <span title={face1 ? face1.backendId : ""}>
            {face1 ? `${nameOf(face1.backendId)} #${face1.faceIndex}` : faceLabel(null)}
          </span>
          <button
            type="button"
            className={mateFacePickSlot === 1 ? "active" : ""}
            onClick={() => setMateFacePickSlot(mateFacePickSlot === 1 ? null : 1)}
          >
            拾取动件面
          </button>
          <span title={face2 ? face2.backendId : ""}>
            {face2 ? `${nameOf(face2.backendId)} #${face2.faceIndex}` : faceLabel(null)}
          </span>
        </div>

        <div className="dlg-actions">
          <button type="button" onClick={() => void handleClose()}>
            取消
          </button>
          <button
            type="button"
            className="primary"
            disabled={!canApply}
            onClick={() => void runMate(true)}
          >
            应用
          </button>
        </div>
      </div>
    </div>
  );
}
