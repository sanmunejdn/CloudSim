import {
  createContext,
  useCallback,
  useContext,
  useMemo,
  useState,
  type ReactNode,
} from "react";
import { trajSession, trajBeginEdit, trajCancelEdit, trajPathPlans, trajBind } from "../api";
import { publishRawPreview } from "../scene/rawPreview";
import { useStatus } from "./statusStore";

export type TrajFeature = {
  featureId: string;
  strategyId: string;
  geometry?: { faceIndices?: number[]; edgeIndices?: number[]; polylineXyz?: number[] };
  params?: Record<string, unknown>;
};

function featuresFromSession(s: Record<string, unknown> | null | undefined): {
  features: TrajFeature[];
  workpieceId?: string;
  defaultStrategyId?: string;
} {
  const raw = s?.sourceFeatureJson;
  if (!raw) return { features: [] };
  try {
    const doc = (typeof raw === "string" ? JSON.parse(raw) : raw) as {
      features?: TrajFeature[];
      workpiece?: { backendIdUtf8?: string };
      defaultStrategyId?: string;
    };
    if (!Array.isArray(doc.features)) return { features: [] };
    return {
      features: doc.features.map((f) => ({
        ...f,
        params: f.params || {},
      })),
      workpieceId: doc.workpiece?.backendIdUtf8,
      defaultStrategyId: doc.defaultStrategyId,
    };
  } catch {
    return { features: [] };
  }
}

type TrajCtx = {
  pickMode: "edge" | "face" | null;
  setPickMode: (m: "edge" | "face" | null) => void;
  featureEditActive: boolean;
  pathPlanId: string;
  pathPlans: { id: string; name?: string; phase?: string }[];
  features: TrajFeature[];
  setFeatures: (f: TrajFeature[]) => void;
  workpieceId: string;
  setWorkpieceId: (id: string) => void;
  /** 生成/应用/取消后递增，供面板重置本地 UI */
  editUiEpoch: number;
  syncSession: () => Promise<Record<string, unknown> | null>;
  beginEdit: () => Promise<Record<string, unknown> | null>;
  cancelEdit: () => Promise<void>;
  /** 对齐 exitTrajEditUiAfterCommit */
  exitEditAfterCommit: () => Promise<Record<string, unknown> | null>;
  reloadPathPlans: (rootId: string) => Promise<void>;
  bindPlan: (pathPlanId: string, rootId: string) => Promise<void>;
};

const Ctx = createContext<TrajCtx | null>(null);

const EMPTY_AXIS = { x: true, y: true, z: true, interval: 0 };

export function TrajectoryProvider({ children }: { children: ReactNode }) {
  const { setStatus } = useStatus();
  const [pickMode, setPickMode] = useState<"edge" | "face" | null>(null);
  const [featureEditActive, setEdit] = useState(false);
  const [pathPlanId, setPathPlanId] = useState("");
  const [pathPlans, setPathPlans] = useState<{ id: string; name?: string; phase?: string }[]>([]);
  const [features, setFeatures] = useState<TrajFeature[]>([]);
  const [workpieceId, setWorkpieceId] = useState("");
  const [editUiEpoch, setEditUiEpoch] = useState(0);

  const clearEditUiLocal = useCallback(() => {
    setFeatures([]);
    setPickMode(null);
    setEdit(false);
    setEditUiEpoch((n) => n + 1);
    publishRawPreview(null, EMPTY_AXIS);
    window.dispatchEvent(new CustomEvent("cloudsim-pick-highlight", { detail: { clear: true } }));
  }, []);

  const syncSession = useCallback(async () => {
    try {
      const s = await trajSession();
      setEdit(!!s.featureEditActive);
      if (typeof s.pathPlanId === "string") setPathPlanId(s.pathPlanId);
      return s;
    } catch {
      return null;
    }
  }, []);

  const exitEditAfterCommit = useCallback(async () => {
    clearEditUiLocal();
    return syncSession();
  }, [clearEditUiLocal, syncSession]);

  const beginEdit = useCallback(async () => {
    const r = await trajBeginEdit();
    if (!r.ok) {
      setStatus(r.error || "开始修改失败", "err");
      return null;
    }
    const s = await syncSession();
    const loaded = featuresFromSession(s);
    setFeatures(loaded.features);
    if (loaded.workpieceId) setWorkpieceId(loaded.workpieceId);
    setStatus(s?.hasRaw ? "已开始修改（已加载特征）" : "已开始修改，请拾取或导入特征");
    return s;
  }, [setStatus, syncSession]);

  const cancelEdit = useCallback(async () => {
    await trajCancelEdit();
    clearEditUiLocal();
    await syncSession();
    setStatus("已取消修改；特征表已清空，预览已关闭");
  }, [setStatus, syncSession, clearEditUiLocal]);

  const reloadPathPlans = useCallback(async (rootId: string) => {
    if (!rootId) return;
    const r = await trajPathPlans(rootId);
    setPathPlans(r.pathPlans || []);
  }, []);

  const bindPlan = useCallback(
    async (id: string, rootId: string) => {
      const r = await trajBind(id, rootId);
      if (!r.ok) {
        setStatus(r.error || "绑定失败", "err");
        return;
      }
      // 对齐桌面：切换 PathPlan 只 bind，清特征表，等「开始修改」
      clearEditUiLocal();
      setPathPlanId(id);
      await syncSession();
      setStatus("已绑定 PathPlan，点击「开始修改」加载特征与预览");
    },
    [setStatus, syncSession, clearEditUiLocal],
  );

  const value = useMemo(
    () => ({
      pickMode,
      setPickMode,
      featureEditActive,
      pathPlanId,
      pathPlans,
      features,
      setFeatures,
      workpieceId,
      setWorkpieceId,
      editUiEpoch,
      syncSession,
      beginEdit,
      cancelEdit,
      exitEditAfterCommit,
      reloadPathPlans,
      bindPlan,
    }),
    [
      pickMode,
      featureEditActive,
      pathPlanId,
      pathPlans,
      features,
      workpieceId,
      editUiEpoch,
      syncSession,
      beginEdit,
      cancelEdit,
      exitEditAfterCommit,
      reloadPathPlans,
      bindPlan,
    ],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useTrajectory() {
  const v = useContext(Ctx);
  if (!v) throw new Error("TrajectoryProvider missing");
  return v;
}
