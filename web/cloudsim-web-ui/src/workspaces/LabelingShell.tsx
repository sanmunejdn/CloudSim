import { useEffect, useState } from "react";
import { apiJson } from "../api/client";

export default function LabelingShell() {
  const [tasks, setTasks] = useState<{ id: string; title: string }[]>([]);

  useEffect(() => {
    void apiJson<{ ok?: boolean; tasks?: { id: string; title: string }[] }>("/api/labeling/tasks").then((r) => {
      if (r.ok && r.tasks) setTasks(r.tasks);
    });
  }, []);

  return (
    <div className="workspace-shell labeling-shell">
      <header className="workspace-head">
        <h2>标注</h2>
        <p className="hint">3D 标注任务</p>
      </header>
      <ul className="workspace-list">
        {tasks.length ? (
          tasks.map((t) => (
            <li key={t.id}>
              {t.title || t.id}
            </li>
          ))
        ) : (
          <li className="hint">暂无标注任务</li>
        )}
      </ul>
    </div>
  );
}
