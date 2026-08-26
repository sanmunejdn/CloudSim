import type { ReactNode } from "react";
import { StatusProvider } from "./statusStore";
import { ProjectProvider } from "./projectStore";
import { SceneProvider } from "./sceneStore";
import { RobotProgramProvider } from "./robotProgramStore";
import { TrajectoryProvider } from "./trajectoryStore";
import { FrameProvider } from "./frameStore";
import { PointCloudProvider } from "./pointCloudStore";
import { DockNavProvider } from "./dockNavStore";
import { DeviceRuntimeProvider } from "./deviceRuntimeStore";
import { I18nProvider } from "../i18n/useI18n";

export function AppProviders({ children }: { children: ReactNode }) {
  return (
    <I18nProvider>
      <StatusProvider>
        <ProjectProvider>
        <SceneProvider>
          <DockNavProvider>
            <DeviceRuntimeProvider>
              <RobotProgramProvider>
                <TrajectoryProvider>
                  <FrameProvider>
                    <PointCloudProvider>{children}</PointCloudProvider>
                  </FrameProvider>
                </TrajectoryProvider>
              </RobotProgramProvider>
            </DeviceRuntimeProvider>
          </DockNavProvider>
        </SceneProvider>
        </ProjectProvider>
      </StatusProvider>
    </I18nProvider>
  );
}
