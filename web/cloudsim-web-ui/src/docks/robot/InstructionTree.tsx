import { useMemo, type ReactNode } from "react";
import type { Instruction, InstructionGroup } from "../../api";

const CMD_LABEL: Record<string, string> = {
  ptp: "点到点",
  line: "直线",
  arc: "圆弧",
  wait: "等待",
  if: "条件",
  while: "循环",
  set_do: "数字输出",
  set_ao: "模拟输出",
  path_plan: "路径规划",
};

function fmtXyz(pose?: { x?: number; y?: number; z?: number }) {
  if (!pose) return "0.0, 0.0, 0.0";
  return `${Number(pose.x || 0).toFixed(1)}, ${Number(pose.y || 0).toFixed(1)}, ${Number(pose.z || 0).toFixed(1)}`;
}

function fmtBool(v: unknown): "0" | "1" {
  if (v === false || v === 0 || v === "0" || v === "false") return "0";
  return "1";
}

/** 对齐桌面 InstructionProgramTreeWidget：IO/条件摘要，避免信号指令误显示 XYZ */
function fmtCondition(c?: Instruction["condition"]): string {
  if (!c) return "始终";
  const kind = String(c.kind || "").toLowerCase();
  if (kind === "never") return "永不";
  if (kind === "io") {
    const port = Number(c.port ?? c.ioPort);
    const sig = (c.signalName || "").trim() || (Number.isFinite(port) ? `IO${port}` : "IO");
    const eq = c.ioEquals === false ? 0 : 1;
    return `${sig}==${eq}`;
  }
  if (kind === "compare") {
    return `${c.compareLeft || "?"} ${c.compareOp || "=="} ${c.compareRight ?? 0}`;
  }
  return "始终";
}

function fmtSignalIo(ins: Instruction, analog: boolean): string {
  const name = String(ins.signalName || "").trim();
  const port = Number(ins.port ?? ins.ioPort);
  const left = name || `port ${Number.isFinite(port) ? port : 0}`;
  if (analog) {
    const raw = ins.value ?? ins.ioAnalogValue ?? 0;
    return `${left} = ${Number(raw).toFixed(2)}`;
  }
  const raw = ins.value ?? ins.ioBoolValue;
  return `${left} = ${fmtBool(raw)}`;
}

function renumber(steps: Instruction[]) {
  let next = 1;
  const walk = (arr?: Instruction[]) => {
    for (const ins of arr || []) {
      const t = String(ins.type || "").toLowerCase();
      if (t === "ptp" || t === "line" || t === "arc") ins.pointIndex = next++;
      else if (t === "if") {
        walk(ins.then);
        walk(ins.else);
      } else if (t === "while") walk(ins.body);
    }
  };
  walk(steps);
}

function labelOf(ins: Instruction) {
  const type = String(ins.type || "").toLowerCase();
  const typeLabel = CMD_LABEL[type] || type;
  if (type === "path_plan") {
    let title = ins.name || "路径规划";
    const phase = String(ins.phase || "").toLowerCase();
    const phaseZh = phase === "applied" ? "已应用" : phase === "raw_ready" ? "已离散" : "草稿";
    return `${title} · ${phaseZh}`;
  }
  if (type === "wait") {
    const cond = ins.condition;
    if (cond && String(cond.kind || "").toLowerCase() === "io") {
      let summary = fmtCondition(cond);
      const t = Number(ins.durationSec) || 0;
      if (t > 1e-9) summary += ` 超时${t.toFixed(1)}s`;
      return `[${typeLabel}] ${summary}`;
    }
    return `[${typeLabel}] 时长 ${Number(ins.durationSec || 0).toFixed(2)} s`;
  }
  if (type === "if" || type === "while") {
    return `[${typeLabel}] ${fmtCondition(ins.condition)}`;
  }
  if (type === "set_do") {
    return `[${typeLabel}] ${fmtSignalIo(ins, false)}`;
  }
  if (type === "set_ao") {
    return `[${typeLabel}] ${fmtSignalIo(ins, true)}`;
  }
  if (type === "arc") {
    const pi = Number(ins.pointIndex) || 0;
    return pi > 0 ? `P${pi} [${typeLabel}]` : `[${typeLabel}]`;
  }
  if (type === "ptp" || type === "line") {
    const pi = Number(ins.pointIndex) || 0;
    const xyz = fmtXyz(ins.pose);
    const summary = pi > 0 ? `P${pi} · 第${pi}点 · XYZ ${xyz}` : `XYZ ${xyz}`;
    return pi > 0 ? `P${pi} [${typeLabel}] ${summary}` : `[${typeLabel}] ${summary}`;
  }
  return `[${typeLabel}]`;
}

type Props = {
  steps: Instruction[];
  groups: InstructionGroup[];
  selectedId: string | null;
  onSelect: (id: string) => void;
};

function Row(props: {
  label: string;
  selected?: boolean;
  struct?: boolean;
  hasChildren?: boolean;
  onClick?: () => void;
  children?: ReactNode;
}) {
  return (
    <div className="prog-tree-node">
      <div
        className={`prog-tree-row ${props.selected ? "sel" : ""} ${props.struct ? "struct" : ""}`}
        onClick={props.onClick}
      >
        <span className={`prog-tree-toggle ${props.hasChildren ? "" : "empty"}`}>{props.hasChildren ? "▾" : ""}</span>
        <span className="t" title={props.label}>
          {props.label}
        </span>
      </div>
      {props.children ? <div className="prog-tree-kids">{props.children}</div> : null}
    </div>
  );
}

function InstrNode({
  ins,
  selectedId,
  onSelect,
}: {
  ins: Instruction;
  selectedId: string | null;
  onSelect: (id: string) => void;
}) {
  const type = String(ins.type || "").toLowerCase();
  return (
    <Row
      label={labelOf(ins)}
      selected={ins.id === selectedId}
      hasChildren={type === "arc" || type === "if" || type === "while"}
      onClick={() => onSelect(ins.id)}
    >
      {type === "arc" && (
        <>
          <Row label={`  途经  ${fmtXyz(ins.viaPose)}`} onClick={() => onSelect(ins.id)} />
          <Row label={`  终点  ${fmtXyz(ins.pose)}`} onClick={() => onSelect(ins.id)} />
        </>
      )}
      {type === "if" && (
        <>
          <Row label="Then（真）" struct hasChildren>
            {(ins.then || []).map((s) => (
              <InstrNode key={s.id} ins={s} selectedId={selectedId} onSelect={onSelect} />
            ))}
          </Row>
          <Row label="Else（假）" struct hasChildren>
            {(ins.else || []).map((s) => (
              <InstrNode key={s.id} ins={s} selectedId={selectedId} onSelect={onSelect} />
            ))}
          </Row>
        </>
      )}
      {type === "while" &&
        (ins.body || []).map((s) => <InstrNode key={s.id} ins={s} selectedId={selectedId} onSelect={onSelect} />)}
    </Row>
  );
}

export default function InstructionTree({ steps, groups, selectedId, onSelect }: Props) {
  const tree = useMemo(() => {
    const arr = steps.map((s) => ({ ...s }));
    renumber(arr);
    const instrToGroup = new Map<string, string>();
    for (const g of groups || []) {
      for (const mid of g.memberIds || []) instrToGroup.set(String(mid), g.id);
    }
    const rootPathPlans = arr.filter((i) => String(i.type).toLowerCase() === "path_plan");
    const motionRoots = arr.filter((i) => String(i.type).toLowerCase() !== "path_plan");
    return { arr, instrToGroup, rootPathPlans, motionRoots, groups };
  }, [steps, groups]);

  const renderedGroups = new Set<string>();

  return (
    <div className="prog-list">
      {tree.rootPathPlans.length > 0 && (
        <Row label="路径规划" struct hasChildren>
          {tree.rootPathPlans.map((pp) => {
            const out = (tree.groups || []).find(
              (g) => g.role === "path_plan_output" && g.pathPlanInstructionId === pp.id && (g.memberIds || []).length,
            );
            return (
              <Row key={pp.id} label={labelOf(pp)} selected={pp.id === selectedId} hasChildren={!!out} onClick={() => onSelect(pp.id)}>
                {out && (
                  <Row
                    label={`↳ 输出: ${out.name || out.id}（${(out.memberIds || []).length} 点）`}
                    onClick={() => onSelect(pp.id)}
                  />
                )}
              </Row>
            );
          })}
        </Row>
      )}
      {tree.motionRoots.map((ins) => {
        const gid = tree.instrToGroup.get(ins.id);
        if (gid) {
          if (renderedGroups.has(gid)) return null;
          renderedGroups.add(gid);
          const g = (tree.groups || []).find((x) => x.id === gid);
          if (!g) return <InstrNode key={ins.id} ins={ins} selectedId={selectedId} onSelect={onSelect} />;
          return (
            <Row key={gid} label={`分组: ${g.name || g.id}`} struct hasChildren>
              {tree.motionRoots
                .filter((s) => tree.instrToGroup.get(s.id) === gid)
                .map((s) => (
                  <InstrNode key={s.id} ins={s} selectedId={selectedId} onSelect={onSelect} />
                ))}
            </Row>
          );
        }
        return <InstrNode key={ins.id} ins={ins} selectedId={selectedId} onSelect={onSelect} />;
      })}
    </div>
  );
}

export { CMD_LABEL };
