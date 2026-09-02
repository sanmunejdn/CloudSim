import { useCallback, useMemo, useRef, useState, type CSSProperties } from "react";

const LS_LEFT = "cloudsim.web.dock.leftWidth";
const LS_RIGHT = "cloudsim.web.dock.rightWidth";
const LS_LEFT_VIS = "cloudsim.web.dock.leftVisible";
const LS_RIGHT_VIS = "cloudsim.web.dock.rightVisible";

const LEFT_DEFAULT = 360;
const RIGHT_DEFAULT = 400;
const LEFT_MIN = 280;
const LEFT_MAX = 720;
const RIGHT_MIN = 300;
const RIGHT_MAX = 720;

function readStored(key: string, fallback: number, min: number, max: number): number {
  try {
    const n = Number(localStorage.getItem(key));
    if (Number.isFinite(n)) return Math.min(max, Math.max(min, n));
  } catch {
    /* ignore */
  }
  return fallback;
}

function readBool(key: string, fallback: boolean): boolean {
  try {
    const v = localStorage.getItem(key);
    if (v === "0" || v === "false") return false;
    if (v === "1" || v === "true") return true;
  } catch {
    /* ignore */
  }
  return fallback;
}

function clamp(n: number, min: number, max: number) {
  return Math.min(max, Math.max(min, n));
}

function persistBool(key: string, v: boolean) {
  try {
    localStorage.setItem(key, v ? "1" : "0");
  } catch {
    /* ignore */
  }
}

/** 左右 Dock 宽度/显隐：拖拽 + localStorage；对齐桌面视图菜单面板开关 */
export function useResizableDocks() {
  const [leftWidth, setLeftWidth] = useState(() =>
    readStored(LS_LEFT, LEFT_DEFAULT, LEFT_MIN, LEFT_MAX),
  );
  const [rightWidth, setRightWidth] = useState(() =>
    readStored(LS_RIGHT, RIGHT_DEFAULT, RIGHT_MIN, RIGHT_MAX),
  );
  const [leftVisible, setLeftVisibleState] = useState(() => readBool(LS_LEFT_VIS, true));
  const [rightVisible, setRightVisibleState] = useState(() => readBool(LS_RIGHT_VIS, true));

  const leftAtDragStart = useRef(leftWidth);
  const rightAtDragStart = useRef(rightWidth);

  const setLeftVisible = useCallback((v: boolean) => {
    setLeftVisibleState(v);
    persistBool(LS_LEFT_VIS, v);
  }, []);

  const setRightVisible = useCallback((v: boolean) => {
    setRightVisibleState(v);
    persistBool(LS_RIGHT_VIS, v);
  }, []);

  const beginLeftDrag = useCallback(() => {
    leftAtDragStart.current = leftWidth;
  }, [leftWidth]);

  const beginRightDrag = useCallback(() => {
    rightAtDragStart.current = rightWidth;
  }, [rightWidth]);

  const onLeftDrag = useCallback((dx: number) => {
    setLeftWidth(clamp(leftAtDragStart.current + dx, LEFT_MIN, LEFT_MAX));
  }, []);

  const onRightDrag = useCallback((dx: number) => {
    setRightWidth(clamp(rightAtDragStart.current - dx, RIGHT_MIN, RIGHT_MAX));
  }, []);

  const persistLeftEnd = useCallback(() => {
    setLeftWidth((w) => {
      try {
        localStorage.setItem(LS_LEFT, String(w));
      } catch {
        /* ignore */
      }
      return w;
    });
  }, []);

  const persistRightEnd = useCallback(() => {
    setRightWidth((w) => {
      try {
        localStorage.setItem(LS_RIGHT, String(w));
      } catch {
        /* ignore */
      }
      return w;
    });
  }, []);

  const resetLayout = useCallback(() => {
    setLeftWidth(LEFT_DEFAULT);
    setRightWidth(RIGHT_DEFAULT);
    setLeftVisibleState(true);
    setRightVisibleState(true);
    try {
      localStorage.setItem(LS_LEFT, String(LEFT_DEFAULT));
      localStorage.setItem(LS_RIGHT, String(RIGHT_DEFAULT));
      persistBool(LS_LEFT_VIS, true);
      persistBool(LS_RIGHT_VIS, true);
    } catch {
      /* ignore */
    }
  }, []);

  const mainStyle = useMemo((): CSSProperties => {
    const cols: string[] = [];
    if (leftVisible) cols.push(`${leftWidth}px`, "5px");
    cols.push("1fr");
    if (rightVisible) cols.push("5px", `${rightWidth}px`);
    return { gridTemplateColumns: cols.join(" ") };
  }, [leftWidth, rightWidth, leftVisible, rightVisible]);

  const gridStyle = useCallback(
    (showLeft: boolean, showRight: boolean): CSSProperties => {
      const cols: string[] = [];
      if (showLeft) cols.push(`${leftWidth}px`, "5px");
      cols.push("1fr");
      if (showRight) cols.push("5px", `${rightWidth}px`);
      return { gridTemplateColumns: cols.join(" ") };
    },
    [leftWidth, rightWidth],
  );

  return {
    mainStyle,
    gridStyle,
    leftVisible,
    rightVisible,
    setLeftVisible,
    setRightVisible,
    resetLayout,
    beginLeftDrag,
    beginRightDrag,
    onLeftDrag,
    onRightDrag,
    persistLeftEnd,
    persistRightEnd,
  };
}
