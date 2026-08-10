import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from "react";
import { fetchFrames, type FrameSet } from "../api";
import { eventHub } from "../sse/EventHub";
import { useRobotProgram } from "./robotProgramStore";

type FrameCtx = {
  frames: FrameSet | null;
  linkNames: string[];
  reloadFrames: () => Promise<void>;
};

const Ctx = createContext<FrameCtx | null>(null);

export function FrameProvider({ children }: { children: ReactNode }) {
  const { activeRootId } = useRobotProgram();
  const [frames, setFrames] = useState<FrameSet | null>(null);
  const [linkNames, setLinkNames] = useState<string[]>([]);

  const reloadFrames = useCallback(async () => {
    if (!activeRootId) {
      setFrames(null);
      return;
    }
    const r = await fetchFrames(activeRootId);
    if (r.ok && r.frames) setFrames(r.frames);
    setLinkNames(r.linkNames || []);
  }, [activeRootId]);

  useEffect(() => {
    void reloadFrames();
  }, [reloadFrames]);

  useEffect(() => {
    const off = eventHub.on("RobotCoordinateFramesChanged", () => void reloadFrames());
    return off;
  }, [reloadFrames]);

  const value = useMemo(() => ({ frames, linkNames, reloadFrames }), [frames, linkNames, reloadFrames]);
  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useFrames() {
  const v = useContext(Ctx);
  if (!v) throw new Error("FrameProvider missing");
  return v;
}
