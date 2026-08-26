import { useCallback, useEffect, useMemo, useState } from "react";
import {
  Background,
  Controls,
  ReactFlow,
  type Edge,
  type Node,
  addEdge,
  useEdgesState,
  useNodesState,
  type Connection,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import { apiJson, putJson } from "../api/client";
import { useStatus } from "../state/statusStore";

type PfGraph = { nodes?: unknown[]; edges?: unknown[] };

function asRecord(v: unknown): Record<string, unknown> {
  return v && typeof v === "object" ? (v as Record<string, unknown>) : {};
}

function graphToFlow(graph: PfGraph | null): { nodes: Node[]; edges: Edge[] } {
  const rawNodes = Array.isArray(graph?.nodes) ? graph!.nodes! : [];
  const rawEdges = Array.isArray(graph?.edges) ? graph!.edges! : [];
  const nodes: Node[] = rawNodes.map((n, i) => {
    const o = asRecord(n);
    const id = String(o.id ?? `node-${i}`);
    const pos = asRecord(o.position);
    return {
      id,
      position: {
        x: Number(pos.x ?? o.x ?? i * 140),
        y: Number(pos.y ?? o.y ?? i * 72),
      },
      data: { label: String(o.label ?? o.title ?? id) },
    };
  });
  const edges: Edge[] = rawEdges.map((e, i) => {
    const o = asRecord(e);
    return {
      id: String(o.id ?? `edge-${i}`),
      source: String(o.source ?? ""),
      target: String(o.target ?? ""),
    };
  });
  return { nodes, edges };
}

function flowToGraph(nodes: Node[], edges: Edge[]): PfGraph {
  return {
    nodes: nodes.map((n) => ({
      id: n.id,
      label: String(n.data?.label ?? n.id),
      position: n.position,
    })),
    edges: edges.map((e) => ({ id: e.id, source: e.source, target: e.target })),
  };
}

export default function ProcessFlowShell() {
  const { setStatus } = useStatus();
  const [nodes, setNodes, onNodesChange] = useNodesState<Node>([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState<Edge>([]);

  const reload = useCallback(async () => {
    const r = await apiJson<{ ok?: boolean; graph?: PfGraph }>("/api/processflow/graph");
    if (!r.ok) return;
    const flow = graphToFlow(r.graph || { nodes: [], edges: [] });
    setNodes(flow.nodes);
    setEdges(flow.edges);
  }, [setNodes, setEdges]);

  useEffect(() => {
    void reload();
  }, [reload]);

  const onConnect = useCallback(
    (conn: Connection) => setEdges((eds) => addEdge(conn, eds)),
    [setEdges],
  );

  const saveGraph = async () => {
    const r = await putJson<{ ok: boolean; error?: string }>("/api/processflow/graph", {
      graph: flowToGraph(nodes, edges),
    });
    setStatus(r.ok ? "工艺流程图已保存" : r.error || "保存失败", r.ok ? "info" : "err");
  };

  const summary = useMemo(
    () => `${nodes.length} 节点 · ${edges.length} 边`,
    [nodes.length, edges.length],
  );

  return (
    <div className="workspace-shell processflow-shell">
      <header className="workspace-head">
        <h2>工艺流程</h2>
        <p className="hint">DES 流程图 · {summary}</p>
      </header>
      <div className="workspace-toolbar">
        <button
          type="button"
          className="primary"
          onClick={async () => {
            const r = await putJson<{ ok: boolean; error?: string }>("/api/processflow/sim/run", {
              durationSec: 3600,
              policy: "FIFO",
            });
            setStatus(r.ok ? "DES 仿真已启动" : r.error || "仿真失败", r.ok ? "info" : "err");
          }}
        >
          运行仿真
        </button>
        <button type="button" onClick={() => void reload()}>
          刷新
        </button>
        <button type="button" onClick={() => void saveGraph()}>
          保存图
        </button>
        <button
          type="button"
          onClick={() =>
            setNodes((nds) => [
              ...nds,
              {
                id: `node-${Date.now()}`,
                position: { x: 40 + nds.length * 24, y: 40 + nds.length * 16 },
                data: { label: `工序 ${nds.length + 1}` },
              },
            ])
          }
        >
          添加工序
        </button>
      </div>
      <section className="workspace-body processflow-canvas">
        <ReactFlow
          nodes={nodes}
          edges={edges}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          onConnect={onConnect}
          fitView
        >
          <Background gap={16} size={1} />
          <Controls />
        </ReactFlow>
      </section>
    </div>
  );
}
