import { createContext, useCallback, useContext, useMemo, useState, type ReactNode } from "react";

export type PrimaryTab = "workspace" | "ai" | "cloud" | "geometry";
export type WsTab = "units" | "devices" | "annotations";
export type DeviceMode = "robot" | "customDevice";
export type RobotTab = "cmd" | "joint" | "trajGen" | "trajEdit" | "frame" | "collision";
export type DeviceTab = "cmd" | "joint";

type DockNav = {
  primary: PrimaryTab;
  ws: WsTab;
  deviceMode: DeviceMode;
  robot: RobotTab;
  deviceTab: DeviceTab;
  setPrimary: (v: PrimaryTab) => void;
  setWs: (v: WsTab) => void;
  setDeviceMode: (v: DeviceMode) => void;
  setRobot: (v: RobotTab) => void;
  setDeviceTab: (v: DeviceTab) => void;
  goTrajGen: () => void;
  goCmd: () => void;
  goDeviceCmd: () => void;
};

const Ctx = createContext<DockNav | null>(null);

export function DockNavProvider({ children }: { children: ReactNode }) {
  const [primary, setPrimary] = useState<PrimaryTab>("workspace");
  const [ws, setWs] = useState<WsTab>("devices");
  const [deviceMode, setDeviceMode] = useState<DeviceMode>("robot");
  const [robot, setRobot] = useState<RobotTab>("cmd");
  const [deviceTab, setDeviceTab] = useState<DeviceTab>("cmd");

  const goTrajGen = useCallback(() => {
    setPrimary("workspace");
    setWs("devices");
    setDeviceMode("robot");
    setRobot("trajGen");
  }, []);

  const goCmd = useCallback(() => {
    setPrimary("workspace");
    setWs("devices");
    setDeviceMode("robot");
    setRobot("cmd");
  }, []);

  const goDeviceCmd = useCallback(() => {
    setPrimary("workspace");
    setWs("devices");
    setDeviceMode("customDevice");
    setDeviceTab("cmd");
  }, []);

  const value = useMemo(
    () => ({
      primary,
      ws,
      deviceMode,
      robot,
      deviceTab,
      setPrimary,
      setWs,
      setDeviceMode,
      setRobot,
      setDeviceTab,
      goTrajGen,
      goCmd,
      goDeviceCmd,
    }),
    [primary, ws, deviceMode, robot, deviceTab, goTrajGen, goCmd, goDeviceCmd],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useDockNav() {
  const v = useContext(Ctx);
  if (!v) throw new Error("DockNavProvider missing");
  return v;
}
