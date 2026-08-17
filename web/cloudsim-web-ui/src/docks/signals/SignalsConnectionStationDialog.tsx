import { useCallback, useEffect, useMemo, useState } from "react";
import {
  ReactFlow,
  Controls,
  MiniMap,
  Handle,
  Position,
  applyNodeChanges,
  type Node,
  type Edge,
  type Connection,
  type NodeProps,
  type NodeChange,
  MarkerType,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import {
  deleteIoWire,
  fetchIoNetwork,
  patchOwnerLayout,
  postIoWire,
  type IoNetworkOwner,
  type IoNetworkWire,
} from "../../api";
import { useStatus } from "../../state/statusStore";

type OwnerNodeData = {
  ownerId: string;
  label: string;
  kind: string;
  di: string[];
  do: string[];
};

function OwnerNode({ data }: NodeProps) {
  const d = data as OwnerNodeData;
  const kindLabel = d.kind === "device" ? "设备" : d.kind === "robot" ? "机器人" : d.kind;
  return (
    <div className={`io-station-node kind-${d.kind === "device" ? "device" : "robot"}`}>
      <div className="io-station-node-card">
        <div className="io-station-node-title">
          <span className="io-station-label">{d.label}</span>
          <span className="io-station-kind">{kindLabel}</span>
        </div>
        <div className="io-station-ports">
          <div className="io-station-col">
            {d.di.length === 0 ? <div className="io-station-empty">无 DI</div> : null}
            {d.di.map((name) => (
              <div key={`di-${name}`} className="io-station-port in">
                <Handle type="target" position={Position.Left} id={`DI:${name}`} className="io-station-handle" />
                <span>DI {name}</span>
              </div>
            ))}
          </div>
          <div className="io-station-col">
            {d.do.length === 0 ? <div className="io-station-empty right">无 DO</div> : null}
            {d.do.map((name) => (
              <div key={`do-${name}`} className="io-station-port out">
                <span>DO {name}</span>
                <Handle type="source" position={Position.Right} id={`DO:${name}`} className="io-station-handle" />
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}

const nodeTypes = { owner: OwnerNode };

type Props = { open: boolean; onClose: () => void };

export default function SignalsConnectionStationDialog({ open, onClose }: Props) {
  const { setStatus } = useStatus();
  const [owners, setOwners] = useState<Record<string, IoNetworkOwner>>({});
  const [wires, setWires] = useState<IoNetworkWire[]>([]);
  const [nodes, setNodes] = useState<Node[]>([]);
  const [edges, setEdges] = useState<Edge[]>([]);

  const load = useCallback(async () => {
    const r = await fetchIoNetwork();
    if (!r.ok) {
      setStatus(r.error || "连接站加载失败", "err");
      return;
    }
    const own = r.owners || {};
    const w = r.wires || [];
    setOwners(own);
    setWires(w);
    const ns: Node[] = Object.entries(own).map(([id, o]) => {
      const signals = o.signals || [];
      return {
        id,
        type: "owner",
        position: { x: Number(o.canvasX) || 120, y: Number(o.canvasY) || 120 },
        data: {
          ownerId: id,
          label: o.displayName || id,
          kind: o.kind || "robot",
          di: signals.filter((s) => s.kind === "DI").map((s) => s.name),
          do: signals.filter((s) => s.kind === "DO").map((s) => s.name),
        } satisfies OwnerNodeData,
      };
    });
    setNodes(ns);
    setEdges(
      w.map((wire) => ({
        id: wire.id,
        source: wire.fromOwnerId,
        sourceHandle: `DO:${wire.fromSignal}`,
        target: wire.toOwnerId,
        targetHandle: `DI:${wire.toSignal}`,
        className: "io-station-edge",
        style: { stroke: "#2b79c2", strokeWidth: 2 },
        markerEnd: { type: MarkerType.ArrowClosed, color: "#2b79c2", width: 16, height: 16 },
      })),
    );
  }, [setStatus]);

  useEffect(() => {
    if (open) void load();
  }, [open, load]);

  const onConnect = async (c: Connection) => {
    if (!c.source || !c.target || !c.sourceHandle || !c.targetHandle) return;
    if (!c.sourceHandle.startsWith("DO:") || !c.targetHandle.startsWith("DI:")) {
      setStatus("只能从 DO 连到 DI", "warn");
      return;
    }
    const r = await postIoWire({
      fromOwnerId: c.source,
      fromSignal: c.sourceHandle.slice(3),
      toOwnerId: c.target,
      toSignal: c.targetHandle.slice(3),
    });
    if (!r.ok) setStatus(r.error || "接线失败", "err");
    else await load();
  };

  const onEdgesDelete = async (removed: Edge[]) => {
    for (const e of removed) {
      const r = await deleteIoWire(e.id);
      if (!r.ok) setStatus(r.error || "删线失败", "err");
    }
    await load();
  };

  const onNodeDragStop = async (_: unknown, node: Node) => {
    await patchOwnerLayout(node.id, node.position.x, node.position.y);
  };

  const onNodesChange = useCallback((chs: NodeChange[]) => {
    setNodes((nds) => applyNodeChanges(chs, nds));
  }, []);

  const title = useMemo(
    () => `信号连接站 · ${Object.keys(owners).length} Owner · ${wires.length} 线`,
    [owners, wires],
  );

  if (!open) return null;

  return (
    <div className="modal-overlay" role="dialog">
      <div className="modal-card io-station-modal">
        <div className="modal-header io-station-header">
          <div>
            <strong>{title}</strong>
            <p className="io-station-subtitle">从机器人 DO 拖到设备 DI；删除选中连线可拆除</p>
          </div>
          <button type="button" className="btn-ghost" onClick={onClose}>
            关闭
          </button>
        </div>
        <div className="io-station-canvas">
          <ReactFlow
            nodes={nodes}
            edges={edges}
            nodeTypes={nodeTypes}
            nodesDraggable
            onNodesChange={onNodesChange}
            onConnect={(c) => void onConnect(c)}
            onEdgesDelete={(e) => void onEdgesDelete(e)}
            onNodeDragStop={(e, n) => void onNodeDragStop(e, n)}
            connectionLineStyle={{ stroke: "#2b79c2", strokeWidth: 2 }}
            fitView
            proOptions={{ hideAttribution: true }}
          >
            <Controls showInteractive={false} />
            <MiniMap
              pannable
              zoomable
              nodeColor={(n) => ((n.data as OwnerNodeData)?.kind === "device" ? "#2a9a6a" : "#2b79c2")}
              maskColor="rgba(40,60,90,0.08)"
            />
          </ReactFlow>
        </div>
      </div>
    </div>
  );
}
