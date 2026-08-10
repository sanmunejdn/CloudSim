import { useEffect, useMemo, useState, type ReactNode } from "react";
import { deleteObject, patchObject } from "../../api";
import { fetchRobotInstances, robotModelNameFromInstance } from "../../api/robot";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";

export default function UnitsTree() {
  const { objects, selectedId, selectObject, refreshObjects, requestFocus } = useScene();
  const { setStatus } = useStatus();
  const [expanded, setExpanded] = useState<Set<string>>(() => new Set());
  const [ctx, setCtx] = useState<{ x: number; y: number; id: string } | null>(null);
  const [robotRootTitles, setRobotRootTitles] = useState<Map<string, string>>(() => new Map());

  const roots = useMemo(() => {
    const ids = new Set(objects.map((o) => o.id));
    return objects.filter((o) => !(o.parentIds || []).some((p) => ids.has(p)));
  }, [objects]);

  useEffect(() => {
    let cancelled = false;
    void (async () => {
      const r = await fetchRobotInstances();
      if (cancelled || !r.ok) return;
      const idSet = new Set(objects.map((o) => o.id));
      const m = new Map<string, string>();
      for (const inst of r.instances || []) {
        const rootId = inst.sceneRootBackendId;
        if (!rootId) continue;
        const model = robotModelNameFromInstance(inst);
        if (!model) continue;
        m.set(rootId, model);
        // 空壳根未进树时，顶层连杆（如 base_link）仍显示型号，对齐桌面 RobotURDF 根名
        const prefix = `${rootId}_`;
        for (const o of objects) {
          if (!o.id.startsWith(prefix)) continue;
          const orphanRoot = !(o.parentIds || []).some((p) => idSet.has(p));
          if (orphanRoot) m.set(o.id, model);
        }
      }
      setRobotRootTitles(m);
    })();
    return () => {
      cancelled = true;
    };
  }, [objects]);

  const byParent = useMemo(() => {
    const m = new Map<string, typeof objects>();
    for (const o of objects) {
      for (const p of o.parentIds || []) {
        if (!m.has(p)) m.set(p, []);
        m.get(p)!.push(o);
      }
    }
    return m;
  }, [objects]);

  const displayName = (id: string, fallback: string) => robotRootTitles.get(id) || fallback;

  const ctxObj = ctx ? objects.find((o) => o.id === ctx.id) : null;

  const renderNode = (id: string, depth: number): ReactNode => {
    const o = objects.find((x) => x.id === id);
    if (!o) return null;
    const kids = byParent.get(id) || [];
    const open = expanded.has(id);
    const hidden = !o.visible;
    const title = displayName(o.id, o.name || o.id);
    return (
      <li key={id} className={`${o.id === selectedId ? "sel" : ""} ${hidden ? "tree-hidden" : ""}`}>
        <div className="tree-row">
          {kids.length ? (
            <button
              type="button"
              className="tree-twist"
              onClick={(e) => {
                e.stopPropagation();
                setExpanded((prev) => {
                  const n = new Set(prev);
                  if (n.has(id)) n.delete(id);
                  else n.add(id);
                  return n;
                });
              }}
            >
              {open ? "▾" : "▸"}
            </button>
          ) : (
            <span className="tree-twist-spacer" />
          )}
          <div
            className="tree-body"
            onClick={() => void selectObject(o.id)}
            onContextMenu={(e) => {
              e.preventDefault();
              setCtx({ x: e.clientX, y: e.clientY, id: o.id });
            }}
          >
            <span className="tree-vis" title={hidden ? "已隐藏" : "显示中"}>
              {hidden ? "○" : "●"}
            </span>
            <span className="tree-name">{title}</span>
            <span className="cls">{o.className}</span>
          </div>
        </div>
        {open && kids.length > 0 && (
          <ul className="tree-kids">{kids.map((c) => renderNode(c.id, depth + 1))}</ul>
        )}
      </li>
    );
  };

  return (
    <>
      <ul className="tree">{roots.map((r) => renderNode(r.id, 0))}</ul>
      {ctx && (
        <div className="ctx-menu" style={{ left: ctx.x, top: ctx.y }} onMouseLeave={() => setCtx(null)}>
          <button
            type="button"
            onClick={() => {
              void selectObject(ctx.id);
              requestFocus();
              setCtx(null);
            }}
          >
            聚焦
          </button>
          <button
            type="button"
            onClick={async () => {
              const next = !(ctxObj?.visible ?? true);
              const r = await patchObject(ctx.id, { visible: next });
              setStatus(r.ok ? (next ? "已显示" : "已隐藏") : r.error || "可见性设置失败", r.ok ? "info" : "err");
              setCtx(null);
              if (r.ok) await refreshObjects();
            }}
          >
            {ctxObj?.visible === false ? "显示" : "隐藏"}
          </button>
          <button
            type="button"
            className="danger"
            onClick={async () => {
              const r = await deleteObject(ctx.id);
              setStatus(r.ok ? "已删除" : r.error || "删除失败", r.ok ? "info" : "err");
              setCtx(null);
              await refreshObjects();
            }}
          >
            删除
          </button>
        </div>
      )}
    </>
  );
}
