import { useEffect, useMemo, useState } from "react";
import {
  fetchObjectDetail,
  fetchInstructionProperties,
  patchObject,
  patchInstruction,
  fetchIoSignalNames,
  type PropRow,
  type Instruction,
} from "../../api";
import { useScene } from "../../state/sceneStore";
import { useRobotProgram } from "../../state/robotProgramStore";
import { useStatus } from "../../state/statusStore";
import { propertyDisplayLabel } from "./propLabels";
import { buildInstrPropView, type InstrPropViewRow, type SignalNameOptions } from "./instrPropView";
import { eventHub } from "../../sse/EventHub";

function findInstruction(steps: Instruction[] | undefined, id: string | null): Instruction | null {
  if (!steps || !id) return null;
  for (const s of steps) {
    if (s.id === id) return s;
    const nested = findInstruction(s.then, id) || findInstruction(s.else, id) || findInstruction(s.body, id);
    if (nested) return nested;
  }
  return null;
}

export default function PropsPanel() {
  const { selectedId, refreshObjects } = useScene();
  const { selectedInstrId, activeProgram, reloadPrograms } = useRobotProgram();
  const { setStatus } = useStatus();
  const [rawProps, setRawProps] = useState<PropRow[]>([]);
  const [caption, setCaption] = useState("未选中");
  const [mode, setMode] = useState<"object" | "instr" | "empty">("empty");
  const [reloadTick, setReloadTick] = useState(0);
  const [signalOpts, setSignalOpts] = useState<SignalNameOptions>({ di: [], do: [], ao: [] });

  const instruction = useMemo(
    () => findInstruction(activeProgram?.instructions, selectedInstrId),
    [activeProgram?.instructions, selectedInstrId],
  );

  const viewRows: InstrPropViewRow[] = useMemo(() => {
    if (mode === "instr" && selectedInstrId) {
      return buildInstrPropView(rawProps, selectedInstrId, instruction, signalOpts);
    }
    return rawProps.map((p) => ({
      ...p,
      label: propertyDisplayLabel(p.key, p.label),
      kind: "text" as const,
    }));
  }, [mode, selectedInstrId, rawProps, instruction, signalOpts]);

  useEffect(() => {
    let cancelled = false;
    const loadNames = async () => {
      const [di, dout, ao] = await Promise.all([
        fetchIoSignalNames("DI"),
        fetchIoSignalNames("DO"),
        fetchIoSignalNames("AO"),
      ]);
      if (cancelled) return;
      setSignalOpts({
        di: (di.names || []).map(String),
        do: (dout.names || []).map(String),
        ao: (ao.names || []).map(String),
      });
    };
    void loadNames();
    const off = eventHub.on("IoSignalsChanged", () => {
      void loadNames();
    });
    return () => {
      cancelled = true;
      off();
    };
  }, []);

  useEffect(() => {
    let cancelled = false;
    (async () => {
      if (selectedInstrId) {
        const r = await fetchInstructionProperties(selectedInstrId);
        if (cancelled) return;
        if (r.ok) {
          setMode("instr");
          setCaption(`指令属性 · ${selectedInstrId}`);
          setRawProps(r.properties || []);
          return;
        }
        setMode("instr");
        setCaption(`指令属性 · ${selectedInstrId}`);
        setRawProps([]);
        return;
      }
      if (!selectedId) {
        if (cancelled) return;
        setMode("empty");
        setCaption("未选中");
        setRawProps([]);
        return;
      }
      const r = await fetchObjectDetail(selectedId);
      if (cancelled) return;
      setMode("object");
      setCaption(`对象 · ${selectedId}`);
      if (r.properties?.length) setRawProps(r.properties);
      else if (r.object) {
        setRawProps([
          { key: "name", label: "名称", value: r.object.name, editable: true },
          { key: "className", label: "类型", value: r.object.className, editable: false },
          { key: "id", label: "标识", value: r.object.id, editable: false },
        ]);
      } else setRawProps([]);
    })();
    return () => {
      cancelled = true;
    };
  }, [selectedId, selectedInstrId, reloadTick]);

  const commit = async (key: string, val: string) => {
    if (mode === "instr" && selectedInstrId) {
      const r = await patchInstruction(selectedInstrId, key, val);
      if (!r.ok) {
        setStatus(r.error || "属性更新失败", "err");
        return;
      }
      await reloadPrograms();
      setReloadTick((n) => n + 1);
      setStatus("属性已更新");
      return;
    }
    if (selectedId) {
      const r = await patchObject(selectedId, { key, value: val });
      if (!r.ok) setStatus(r.error || "属性更新失败", "err");
      else {
        await refreshObjects();
        setReloadTick((n) => n + 1);
      }
    }
  };

  return (
    <div className="dock-body">
      <div className="prop-head">
        <span>属性</span>
        <span>值</span>
      </div>
      <div className={`prop-table ${mode === "empty" ? "empty" : ""}`}>
        {mode === "empty" && <div className="muted">未选中</div>}
        {mode !== "empty" && (
          <>
            <div className="prop-caption">{caption}</div>
            {!viewRows.length && (
              <div className="prop-row">
                <span className="prop-key">—</span>
                <span className="prop-val">无可编辑属性</span>
              </div>
            )}
            {viewRows.map((p) => (
              <div key={p.key} className={`prop-row ${p.editable ? "editable" : ""}`}>
                <span className="prop-key" title={p.key}>
                  {p.label || p.key}
                </span>
                <span className="prop-val" title={p.value ?? ""}>
                  {p.editable ? (
                    p.kind === "enum" && p.options?.length ? (
                      <select
                        className="prop-input"
                        value={p.value ?? ""}
                        onChange={(e) => void commit(p.key, e.target.value)}
                      >
                        {!p.options.includes(String(p.value ?? "")) && (p.value ?? "") !== "" && (
                          <option value={p.value}>{p.value}</option>
                        )}
                        {p.options.map((opt) => (
                          <option key={opt || "__empty"} value={opt}>
                            {opt || "（空）"}
                          </option>
                        ))}
                      </select>
                    ) : (
                      <input
                        className="prop-input"
                        defaultValue={p.value ?? ""}
                        key={`${mode}-${selectedInstrId || selectedId}-${p.key}-${p.value ?? ""}`}
                        onBlur={(e) => {
                          if (e.target.value !== (p.value ?? "")) void commit(p.key, e.target.value);
                        }}
                      />
                    )
                  ) : (
                    p.value ?? ""
                  )}
                </span>
              </div>
            ))}
          </>
        )}
      </div>
    </div>
  );
}
