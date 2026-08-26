import { useState } from "react";
import { postJson } from "../api/client";
import { useStatus } from "../state/statusStore";

export default function DrawingShell() {
  const { setStatus } = useStatus();
  const [svg, setSvg] = useState("");

  return (
    <div className="workspace-shell drawing-shell">
      <header className="workspace-head">
        <h2>工程图</h2>
        <p className="hint">HLR 多视图 · SVG 导出</p>
      </header>
      <section className="workspace-body">
        <button
          type="button"
          className="primary"
          onClick={async () => {
            const r = await postJson<{ ok: boolean; svg?: string; error?: string }>("/api/drawing/export", {
              format: "svg",
            });
            if (r.ok && r.svg) {
              setSvg(r.svg);
              setStatus("图纸已生成");
            } else setStatus(r.error || "生成失败", "err");
          }}
        >
          生成 SVG
        </button>
        {svg ? (
          <div className="drawing-preview" dangerouslySetInnerHTML={{ __html: svg }} />
        ) : (
          <p className="hint">选择模型后生成图纸</p>
        )}
      </section>
    </div>
  );
}
