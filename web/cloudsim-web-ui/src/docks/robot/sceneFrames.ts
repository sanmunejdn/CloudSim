import { fetchObjects, listCoordinateFrames, type BackendObject } from "../../api";
import { isSceneCoordinateFrame } from "../../scene/frameAxes";

export type SceneFrameOpt = { id: string; name: string };

/** 对齐旧网页 fetchSceneCoordinateFrames：Host 登记帧优先，再扫场景对象 */
export async function fetchSceneCoordinateFrames(localObjects?: BackendObject[]): Promise<SceneFrameOpt[]> {
  try {
    const r = await listCoordinateFrames();
    if (r?.ok && Array.isArray(r.frames) && r.frames.length) {
      return (r.frames as { id?: string; name?: string }[])
        .filter((f) => f?.id)
        .map((f) => ({ id: String(f.id), name: String(f.name || f.id) }));
    }
  } catch {
    /* 回落 */
  }

  const fromLocal = (localObjects || [])
    .filter((o) => isSceneCoordinateFrame(o))
    .map((o) => ({ id: String(o.id), name: String(o.name || o.id) }));
  if (fromLocal.length) return fromLocal;

  try {
    const list = await fetchObjects();
    return (list.objects || [])
      .filter((o) => isSceneCoordinateFrame(o))
      .map((o) => ({ id: String(o.id), name: String(o.name || o.id) }));
  } catch {
    return [];
  }
}
