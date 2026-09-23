/// @file LeftDock.tsx
/// @brief 左坞：属性 / 设备 / 信号 / PLC / 相机

import { useEffect } from "react";
import PropsPanel from "../docks/props/PropsPanel";
import DevicesPanel from "../docks/devices/DevicesPanel";
import SignalsPanel from "../docks/signals/SignalsPanel";
import PlcPanel from "../docks/devices/PlcPanel";
import CameraPanel from "../docks/devices/CameraPanel";
import { useRobotProgram } from "../state/robotProgramStore";
import { useDockNav, type LeftTab } from "../state/dockNavStore";

const LEFT_TABS: { id: LeftTab; label: string }[] = [
  { id: "props", label: "属性" },
  { id: "devices", label: "设备" },
  { id: "signals", label: "信号" },
  { id: "plc", label: "PLC" },
  { id: "camera", label: "相机" },
];

export default function LeftDock() {
  const { left, setLeft, focusProps } = useDockNav();
  const { selectedInstrId } = useRobotProgram();

  useEffect(() => {
    if (selectedInstrId) focusProps();
  }, [selectedInstrId, focusProps]);

  return (
    <aside className="left dock">
      <div className="dock-tabs primary">
        {LEFT_TABS.map((t) => (
          <button
            key={t.id}
            type="button"
            className={`tab ${left === t.id ? "active" : ""}`}
            onClick={() => setLeft(t.id)}
          >
            {t.label}
          </button>
        ))}
      </div>
      {left === "props" ? <PropsPanel /> : null}
      {left === "devices" ? <DevicesPanel /> : null}
      {left === "signals" ? <SignalsPanel /> : null}
      {left === "plc" ? <PlcPanel /> : null}
      {left === "camera" ? <CameraPanel /> : null}
    </aside>
  );
}
