import { createContext, useCallback, useContext, useMemo, useState, type ReactNode } from "react";

export type PrimaryTab = "workspace" | "ai" | "cloud";
export type WsTab = "units" | "robot";
export type RobotTab = "cmd" | "joint" | "trajGen" | "trajEdit" | "frame";

type DockNav = {
  primary: PrimaryTab;
  ws: WsTab;
  robot: RobotTab;
  setPrimary: (v: PrimaryTab) => void;
  setWs: (v: WsTab) => void;
  setRobot: (v: RobotTab) => void;
  /** 对齐旧版：选中/新建 PathPlan 时切到轨迹生成 */
  goTrajGen: () => void;
  /** 生成/应用后回到指令树 */
  goCmd: () => void;
};

const Ctx = createContext<DockNav | null>(null);

export function DockNavProvider({ children }: { children: ReactNode }) {
  const [primary, setPrimary] = useState<PrimaryTab>("workspace");
  const [ws, setWs] = useState<WsTab>("robot");
  const [robot, setRobot] = useState<RobotTab>("cmd");

  const goTrajGen = useCallback(() => {
    setPrimary("workspace");
    setWs("robot");
    setRobot("trajGen");
  }, []);

  const goCmd = useCallback(() => {
    setPrimary("workspace");
    setWs("robot");
    setRobot("cmd");
  }, []);

  const value = useMemo(
    () => ({ primary, ws, robot, setPrimary, setWs, setRobot, goTrajGen, goCmd }),
    [primary, ws, robot, goTrajGen, goCmd],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useDockNav() {
  const v = useContext(Ctx);
  if (!v) throw new Error("DockNavProvider missing");
  return v;
}
