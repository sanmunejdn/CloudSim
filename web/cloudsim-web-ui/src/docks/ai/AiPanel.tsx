import { useState } from "react";
import { aiChat, type AiDomainId } from "../../api/ai";
import { useStatus } from "../../state/statusStore";

const DOMAINS: { id: AiDomainId; label: string }[] = [
  { id: "scene.ops", label: "场景" },
  { id: "robot.command", label: "机器人" },
  { id: "process.flow", label: "工艺流程" },
];

export default function AiPanel() {
  const { setStatus } = useStatus();
  const [prompt, setPrompt] = useState("");
  const [domain, setDomain] = useState<AiDomainId>("scene.ops");
  const [out, setOut] = useState("");

  return (
    <div className="dock-body" id="rightAi" style={{ padding: 8, display: "flex", flexDirection: "column", gap: 8 }}>
      <label className="field">
        领域
        <select value={domain} onChange={(e) => setDomain(e.target.value as AiDomainId)}>
          {DOMAINS.map((d) => (
            <option key={d.id} value={d.id}>
              {d.label}
            </option>
          ))}
        </select>
      </label>
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
          const text = prompt.trim();
          if (!text) {
            setStatus("请输入问题", "warn");
            return;
          }
          const r = await aiChat(text, domain);
          if (!r.ok) {
            setStatus(r.error || "AI 失败", "err");
            setOut(r.error || "");
          } else {
            setOut(r.assistantText || r.reply || "");
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
