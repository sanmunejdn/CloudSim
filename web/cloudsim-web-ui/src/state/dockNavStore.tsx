/// @file dockNavStore.tsx
/// @brief 左右坞导航真源（含左坞 Tab）

import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from "react";
import { uiEvents, UI_EVT } from "../ui/uiEvents";

export type PrimaryTab = "workspace" | "ai" | "cloud" | "geometry";
export type WsTab = "units" | "devices" | "annotations";
export type DeviceMode = "robot" | "customDevice";
export type RobotTab = "cmd" | "joint" | "trajGen" | "trajEdit" | "frame" | "collision" | "extAxis" | "comm";
export type DeviceTab = "cmd" | "joint";
export type LeftTab = "props" | "devices" | "signals" | "plc" | "camera";

/** 右坞机器人主路径 Tab；其余收入「更多」 */
export const ROBOT_PRIMARY_TABS: { id: RobotTab; label: string }[] = [
  { id: "cmd", label: "指令" },
  { id: "joint", label: "轴控制" },
  { id: "trajGen", label: "轨迹生成" },
  { id: "trajEdit", label: "轨迹编辑" },
];

export const ROBOT_MORE_TABS: { id: RobotTab; label: string }[] = [
  { id: "frame", label: "坐标系" },
  { id: "extAxis", label: "外轴" },
  { id: "collision", label: "碰撞" },
  { id: "comm", label: "通讯" },
];

type DockNav = {
  primary: PrimaryTab;
  ws: WsTab;
  deviceMode: DeviceMode;
  robot: RobotTab;
  deviceTab: DeviceTab;
  left: LeftTab;
  setPrimary: (v: PrimaryTab) => void;
  setWs: (v: WsTab) => void;
  setDeviceMode: (v: DeviceMode) => void;
  setRobot: (v: RobotTab) => void;
  setDeviceTab: (v: DeviceTab) => void;
  setLeft: (v: LeftTab) => void;
  focusProps: () => void;
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
  const [left, setLeft] = useState<LeftTab>("props");

  const focusProps = useCallback(() => {
    setLeft("props");
  }, []);

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

  useEffect(() => uiEvents.on(UI_EVT.focusProps, () => focusProps()), [focusProps]);

  const value = useMemo(
    () => ({
      primary,
      ws,
      deviceMode,
      robot,
      deviceTab,
      left,
      setPrimary,
      setWs,
      setDeviceMode,
      setRobot,
      setDeviceTab,
      setLeft,
      focusProps,
      goTrajGen,
      goCmd,
      goDeviceCmd,
    }),
    [primary, ws, deviceMode, robot, deviceTab, left, focusProps, goTrajGen, goCmd, goDeviceCmd],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useDockNav() {
  const v = useContext(Ctx);
  if (!v) throw new Error("DockNavProvider missing");
  return v;
}
