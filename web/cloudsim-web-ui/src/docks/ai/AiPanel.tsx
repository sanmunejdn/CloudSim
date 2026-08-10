import { useState } from "react";
import { aiChat } from "../../api";
import { useStatus } from "../../state/statusStore";

export default function AiPanel() {
  const { setStatus } = useStatus();
  const [prompt, setPrompt] = useState("");
  const [out, setOut] = useState("");

  return (
    <div className="dock-body" id="rightAi" style={{ padding: 8, display: "flex", flexDirection: "column", gap: 8 }}>
      <textarea
        id="aiPrompt"
        rows={5}
        value={prompt}
        onChange={(e) => setPrompt(e.target.value)}
        placeholder="向 AI 助手提问…"
      />
      <button
        type="button"
        id="btnAi"
        onClick={async () => {
          const r = await aiChat(prompt);
          if (!r.ok) {
            setStatus(r.error || "AI 失败", "err");
            setOut(r.error || "");
          } else {
            setOut(r.reply || "");
            setStatus("AI 已回复");
          }
        }}
      >
        发送
      </button>
      <pre id="aiOut" className="ai-out">
        {out}
      </pre>
    </div>
  );
}
