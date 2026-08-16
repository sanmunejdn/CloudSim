import { createContext, useContext, useMemo, useState, type ReactNode } from "react";

type DeviceRuntime = {
  selectedCustomDeviceId: string;
  setSelectedCustomDeviceId: (id: string) => void;
};

const Ctx = createContext<DeviceRuntime | null>(null);

export function DeviceRuntimeProvider({ children }: { children: ReactNode }) {
  const [selectedCustomDeviceId, setSelectedCustomDeviceId] = useState("");
  const value = useMemo(
    () => ({ selectedCustomDeviceId, setSelectedCustomDeviceId }),
    [selectedCustomDeviceId],
  );
  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useDeviceRuntime() {
  const v = useContext(Ctx);
  if (!v) throw new Error("DeviceRuntimeProvider missing");
  return v;
}
