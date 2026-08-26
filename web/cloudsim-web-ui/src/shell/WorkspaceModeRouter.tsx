import type { ReactNode } from "react";
import { useProject } from "../state/projectStore";
import Scene3dShell from "../workspaces/Scene3dShell";
import GeomodelingShell from "../workspaces/GeomodelingShell";
import ProcessFlowShell from "../workspaces/ProcessFlowShell";
import DrawingShell from "../workspaces/DrawingShell";
import LabelingShell from "../workspaces/LabelingShell";

type Props = {
  scene3dChildren: ReactNode;
};

/** 按 workspaceMode 侧车切换中央工作区壳 */
export default function WorkspaceModeRouter({ scene3dChildren }: Props) {
  const { mode } = useProject();

  switch (mode) {
    case "geomodeling":
      return <GeomodelingShell />;
    case "processflow":
      return <ProcessFlowShell />;
    case "drawing":
      return <DrawingShell />;
    case "labeling":
      return <LabelingShell />;
    case "scene3d":
    default:
      return <Scene3dShell>{scene3dChildren}</Scene3dShell>;
  }
}
