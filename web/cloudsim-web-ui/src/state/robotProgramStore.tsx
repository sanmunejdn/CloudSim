import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from "react";
import {
  fetchPrograms,
  putPrograms,
  type ProgramCatalog,
  type Instruction,
  type RobotProgram,
  fetchRobotInstances,
} from "../api";
import { eventHub } from "../sse/EventHub";
import { useScene } from "./sceneStore";
import { useStatus } from "./statusStore";

type RobotCtx = {
  catalogs: ProgramCatalog[];
  activeRootId: string;
  activeProgram: RobotProgram | null;
  selectedInstrId: string | null;
  setSelectedInstrId: (id: string | null) => void;
  playing: boolean;
  setPlaying: (v: boolean) => void;
  reloadPrograms: () => Promise<void>;
  savePrograms: (next: ProgramCatalog[]) => Promise<void>;
  updateActiveProgram: (mut: (p: RobotProgram) => RobotProgram) => Promise<void>;
  preferRootId: () => string;
};

const Ctx = createContext<RobotCtx | null>(null);

function normalizeCatalogs(raw: unknown): ProgramCatalog[] {
  if (Array.isArray(raw)) return raw as ProgramCatalog[];
  if (raw && typeof raw === "object" && Array.isArray((raw as { programs?: unknown }).programs)) {
    return (raw as { programs: ProgramCatalog[] }).programs;
  }
  return [];
}

export function RobotProgramProvider({ children }: { children: ReactNode }) {
  const { objects, selectedId } = useScene();
  const { setStatus } = useStatus();
  const [catalogs, setCatalogs] = useState<ProgramCatalog[]>([]);
  const [selectedInstrId, setSelectedInstrId] = useState<string | null>(null);
  const [playing, setPlaying] = useState(false);
  const [roots, setRoots] = useState<string[]>([]);

  const preferRootId = useCallback(() => {
    const withInstr = catalogs.find((c) =>
      (c.programs || []).some((p) => (p.instructions || []).length > 0),
    );
    if (withInstr?.sceneBackendId) return withInstr.sceneBackendId;
    if (roots[0]) return roots[0];
    return selectedId || "";
  }, [catalogs, roots, selectedId]);

  const activeRootId = preferRootId();
  const entry = catalogs.find((c) => c.sceneBackendId === activeRootId);
  const activeProgram =
    entry?.programs?.find((p) => p.id === entry.activeProgramId) ||
    entry?.programs?.find((p) => p.isMain) ||
    entry?.programs?.[0] ||
    null;

  const reloadPrograms = useCallback(async () => {
    const raw = await fetchPrograms();
    setCatalogs(normalizeCatalogs(raw));
    try {
      const inst = await fetchRobotInstances();
      setRoots((inst.instances || []).map((i) => i.sceneRootBackendId).filter(Boolean));
    } catch {
      /* ignore */
    }
  }, []);

  useEffect(() => {
    void reloadPrograms();
  }, [objects.length, reloadPrograms]);

  useEffect(() => {
    const off = eventHub.onAny((_d, type) => {
      if (type === "ProjectLoaded" || type === "BackendObjectCreated" || type === "message") {
        void reloadPrograms();
      }
    });
    return off;
  }, [reloadPrograms]);

  const savePrograms = useCallback(
    async (next: ProgramCatalog[]) => {
      setCatalogs(next);
      const r = await putPrograms(next);
      if (!r.ok) setStatus(r.error || "保存程序失败", "err");
    },
    [setStatus],
  );

  const updateActiveProgram = useCallback(
    async (mut: (p: RobotProgram) => RobotProgram) => {
      const root = preferRootId();
      if (!root) {
        setStatus("请先导入机器人", "warn");
        return;
      }
      let next = catalogs.map((c) => ({ ...c, programs: [...(c.programs || [])] }));
      let entryIdx = next.findIndex((c) => c.sceneBackendId === root);
      if (entryIdx < 0) {
        next = [
          ...next,
          {
            sceneBackendId: root,
            activeProgramId: "main",
            programs: [{ id: "main", name: "Main", isMain: true, instructions: [], groups: [] }],
          },
        ];
        entryIdx = next.length - 1;
      }
      const e = next[entryIdx];
      const progs = [...(e.programs || [])];
      let pi = progs.findIndex((p) => p.id === e.activeProgramId);
      if (pi < 0) pi = progs.findIndex((p) => p.isMain);
      if (pi < 0) pi = 0;
      if (!progs[pi]) {
        progs[0] = { id: "main", name: "Main", isMain: true, instructions: [], groups: [] };
        pi = 0;
      }
      progs[pi] = mut({ ...progs[pi], instructions: [...(progs[pi].instructions || [])], groups: [...(progs[pi].groups || [])] });
      next[entryIdx] = { ...e, programs: progs };
      await savePrograms(next);
    },
    [catalogs, preferRootId, savePrograms, setStatus],
  );

  const value = useMemo(
    () => ({
      catalogs,
      activeRootId,
      activeProgram,
      selectedInstrId,
      setSelectedInstrId,
      playing,
      setPlaying,
      reloadPrograms,
      savePrograms,
      updateActiveProgram,
      preferRootId,
    }),
    [
      catalogs,
      activeRootId,
      activeProgram,
      selectedInstrId,
      playing,
      reloadPrograms,
      savePrograms,
      updateActiveProgram,
      preferRootId,
    ],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useRobotProgram() {
  const v = useContext(Ctx);
  if (!v) throw new Error("RobotProgramProvider missing");
  return v;
}

export type { Instruction };
