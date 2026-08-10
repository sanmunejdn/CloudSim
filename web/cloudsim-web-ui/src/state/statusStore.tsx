import {
  createContext,
  useCallback,
  useContext,
  useMemo,
  useState,
  type ReactNode,
} from "react";

type StatusKind = "info" | "warn" | "err";

type StatusCtx = {
  status: string;
  kind: StatusKind;
  logLines: string[];
  setStatus: (msg: string, kind?: StatusKind) => void;
  appendLog: (msg: string) => void;
};

const Ctx = createContext<StatusCtx | null>(null);

export function StatusProvider({ children }: { children: ReactNode }) {
  const [status, setStatusState] = useState("");
  const [kind, setKind] = useState<StatusKind>("info");
  const [logLines, setLog] = useState<string[]>([]);

  const setStatus = useCallback((msg: string, k: StatusKind = "info") => {
    setStatusState(msg);
    setKind(k);
    const tag = k === "err" ? "ERR" : k === "warn" ? "WARN" : "INFO";
    const line = `[${new Date().toLocaleTimeString()}] [${tag}] ${msg}`;
    setLog((prev) => [...prev.slice(-200), line]);
  }, []);

  const appendLog = useCallback((msg: string) => {
    setLog((prev) => [...prev.slice(-200), msg]);
  }, []);

  const value = useMemo(
    () => ({ status, kind, logLines, setStatus, appendLog }),
    [status, kind, logLines, setStatus, appendLog],
  );
  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useStatus() {
  const v = useContext(Ctx);
  if (!v) throw new Error("StatusProvider missing");
  return v;
}
