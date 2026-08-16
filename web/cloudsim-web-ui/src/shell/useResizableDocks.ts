import { useCallback, useMemo, useRef, useState, type CSSProperties } from "react";

const LS_LEFT = "cloudsim.web.dock.leftWidth";
const LS_RIGHT = "cloudsim.web.dock.rightWidth";

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

function clamp(n: number, min: number, max: number) {
  return Math.min(max, Math.max(min, n));
}

/** 左右 Dock 宽度：拖拽 + localStorage 记忆 */
export function useResizableDocks() {
  const [leftWidth, setLeftWidth] = useState(() =>
    readStored(LS_LEFT, LEFT_DEFAULT, LEFT_MIN, LEFT_MAX),
  );
  const [rightWidth, setRightWidth] = useState(() =>
    readStored(LS_RIGHT, RIGHT_DEFAULT, RIGHT_MIN, RIGHT_MAX),
  );

  const leftAtDragStart = useRef(leftWidth);
  const rightAtDragStart = useRef(rightWidth);

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

  const mainStyle = useMemo(
    (): CSSProperties => ({
      gridTemplateColumns: `${leftWidth}px 5px 1fr 5px ${rightWidth}px`,
    }),
    [leftWidth, rightWidth],
  );

  return {
    mainStyle,
    beginLeftDrag,
    beginRightDrag,
    onLeftDrag,
    onRightDrag,
    persistLeftEnd,
    persistRightEnd,
  };
}
