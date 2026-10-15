import type { ReactNode } from "react";
import Scene3dShell from "../workspaces/Scene3dShell";

type Props = {
  scene3dChildren: ReactNode;
  /** 保留参数以免改 App 调用签名；主程序壳不使用 */
  sideVisible?: boolean;
};

/** 网页端暂仅主程序三维壳 */
export default function WorkspaceModeRouter({ scene3dChildren }: Props) {
  return <Scene3dShell>{scene3dChildren}</Scene3dShell>;
}
