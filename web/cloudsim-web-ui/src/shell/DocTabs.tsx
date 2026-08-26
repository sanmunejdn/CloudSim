import { useEffect, useState } from "react";
import { useProject } from "../state/projectStore";

type DocTab = { id: string; title: string };

export default function DocTabs() {
  const { docTitle } = useProject();
  const [tabs, setTabs] = useState<DocTab[]>([
    { id: "doc-1", title: "主文档" },
    { id: "doc-2", title: "装配视图" },
  ]);
  const [activeId, setActiveId] = useState("doc-1");

  useEffect(() => {
    setTabs((prev) =>
      prev.map((t) => (t.id === "doc-1" ? { ...t, title: docTitle || "未命名1" } : t)),
    );
  }, [docTitle]);

  const closeTab = (id: string) => {
    if (tabs.length <= 1) return;
    setTabs((prev) => prev.filter((t) => t.id !== id));
    if (activeId === id) {
      const rest = tabs.filter((t) => t.id !== id);
      setActiveId(rest[0]?.id ?? "");
    }
  };

  return (
    <div className="doc-tabs">
      {tabs.map((tab) => (
        <div
          key={tab.id}
          className={`doc-tab ${tab.id === activeId ? "active" : ""}`}
          onClick={() => setActiveId(tab.id)}
        >
          <span>{tab.title}</span>
          <button
            type="button"
            className="doc-close"
            title="关闭"
            disabled={tabs.length <= 1}
            onClick={(e) => {
              e.stopPropagation();
              closeTab(tab.id);
            }}
          >
            ×
          </button>
        </div>
      ))}
    </div>
  );
}
