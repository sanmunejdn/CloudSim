import { useEffect, useState } from "react";
import PropsPanel from "../docks/props/PropsPanel";
import DevicesPanel from "../docks/devices/DevicesPanel";
import SignalsPanel from "../docks/signals/SignalsPanel";
import PlcPanel from "../docks/devices/PlcPanel";
import CameraPanel from "../docks/devices/CameraPanel";
import { useRobotProgram } from "../state/robotProgramStore";

export default function LeftDock() {
  const [tab, setTab] = useState<"props" | "devices" | "signals" | "plc" | "camera">("props");
  const { selectedInstrId } = useRobotProgram();

  // 对齐旧版 focusLeftPropsTab：选中指令时切到属性页
  useEffect(() => {
    if (selectedInstrId) setTab("props");
  }, [selectedInstrId]);

  useEffect(() => {
    const onFocus = () => setTab("props");
    window.addEventListener("cloudsim-focus-props", onFocus);
    return () => window.removeEventListener("cloudsim-focus-props", onFocus);
  }, []);

  return (
    <aside className="left dock">
      <div className="dock-tabs primary">
        <button type="button" className={`tab ${tab === "props" ? "active" : ""}`} onClick={() => setTab("props")}>
          属性
        </button>
        <button type="button" className={`tab ${tab === "devices" ? "active" : ""}`} onClick={() => setTab("devices")}>
          设备
        </button>
        <button type="button" className={`tab ${tab === "signals" ? "active" : ""}`} onClick={() => setTab("signals")}>
          信号
        </button>
        <button type="button" className={`tab ${tab === "plc" ? "active" : ""}`} onClick={() => setTab("plc")}>
          PLC
        </button>
        <button type="button" className={`tab ${tab === "camera" ? "active" : ""}`} onClick={() => setTab("camera")}>
          相机
        </button>
      </div>
      {tab === "props" ? <PropsPanel /> : null}
      {tab === "devices" ? <DevicesPanel /> : null}
      {tab === "signals" ? <SignalsPanel /> : null}
      {tab === "plc" ? <PlcPanel /> : null}
      {tab === "camera" ? <CameraPanel /> : null}
    </aside>
  );
}
