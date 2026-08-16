import { useCallback, useEffect, useRef } from "react";

type Props = {
  onDragStart?: () => void;
  /** dx：相对按下点的水平位移（向右为正） */
  onDrag: (dx: number) => void;
  onDragEnd?: () => void;
  title?: string;
};

/** 主区左右栏拖拽分隔条 */
export default function DockSplitter({ onDragStart, onDrag, onDragEnd, title = "拖动调整宽度" }: Props) {
  const startX = useRef(0);
  const dragging = useRef(false);

  const onPointerMove = useCallback(
    (e: PointerEvent) => {
      if (!dragging.current) return;
      onDrag(e.clientX - startX.current);
    },
    [onDrag],
  );

  const endDrag = useCallback(() => {
    if (!dragging.current) return;
    dragging.current = false;
    document.body.classList.remove("dock-resizing");
    window.removeEventListener("pointermove", onPointerMove);
    window.removeEventListener("pointerup", endDrag);
    onDragEnd?.();
  }, [onDragEnd, onPointerMove]);

  useEffect(() => () => endDrag(), [endDrag]);

  return (
    <div
      className="dock-splitter"
      title={title}
      role="separator"
      aria-orientation="vertical"
      onPointerDown={(e) => {
        e.preventDefault();
        dragging.current = true;
        startX.current = e.clientX;
        document.body.classList.add("dock-resizing");
        onDragStart?.();
        window.addEventListener("pointermove", onPointerMove);
        window.addEventListener("pointerup", endDrag);
      }}
    />
  );
}
