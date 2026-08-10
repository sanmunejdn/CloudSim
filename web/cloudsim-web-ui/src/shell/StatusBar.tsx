import { useProject } from "../state/projectStore";
import { useStatus } from "../state/statusStore";

export default function StatusBar() {
  const { status, kind, logLines } = useStatus();
  const { path, setPath, health } = useProject();
  const lineClass = kind === "err" ? "err" : kind === "warn" ? "warn" : "info";
  return (
    <footer className="bottom">
      <div id="log" className="log">
        {logLines.map((l, i) => (
          <div key={i} className={`line ${lineClass}`}>
            {l}
          </div>
        ))}
      </div>
      <div className="statusbar">
        <span id="status">{status || "就绪"}</span>
        <span id="sseHint">{health ? "SSE 已连接" : "SSE…"}</span>
        <input
          id="path"
          className="path"
          value={path}
          onChange={(e) => setPath(e.target.value)}
          placeholder="工程路径"
        />
      </div>
    </footer>
  );
}
