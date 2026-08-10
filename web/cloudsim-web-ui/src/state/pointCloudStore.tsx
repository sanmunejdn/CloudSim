import { createContext, useCallback, useContext, useMemo, useState, type ReactNode } from "react";

type PointCloudCtx = {
  polylinePickActive: boolean;
  polylineScreenXy: number[];
  setPolylinePickActive: (v: boolean) => void;
  addPolylinePoint: (x: number, y: number) => void;
  clearPolyline: () => void;
  busy: boolean;
  setBusy: (v: boolean) => void;
  renderRevision: number;
  bumpRenderRevision: () => void;
};

const Ctx = createContext<PointCloudCtx | null>(null);

export function PointCloudProvider({ children }: { children: ReactNode }) {
  const [polylinePickActive, setPolylinePickActive] = useState(false);
  const [polylineScreenXy, setPolylineScreenXy] = useState<number[]>([]);
  const [busy, setBusy] = useState(false);
  const [renderRevision, setRenderRevision] = useState(0);

  const bumpRenderRevision = useCallback(() => setRenderRevision((n) => n + 1), []);

  const addPolylinePoint = useCallback((x: number, y: number) => {
    setPolylineScreenXy((prev) => [...prev, x, y]);
  }, []);

  const clearPolyline = useCallback(() => setPolylineScreenXy([]), []);

  const value = useMemo(
    () => ({
      polylinePickActive,
      polylineScreenXy,
      setPolylinePickActive,
      addPolylinePoint,
      clearPolyline,
      busy,
      setBusy,
      renderRevision,
      bumpRenderRevision,
    }),
    [polylinePickActive, polylineScreenXy, addPolylinePoint, clearPolyline, busy, renderRevision, bumpRenderRevision],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function usePointCloud() {
  const v = useContext(Ctx);
  if (!v) throw new Error("PointCloudProvider missing");
  return v;
}
