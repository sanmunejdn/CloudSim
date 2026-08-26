import type { ReactNode } from "react";

/** 主程序（三维场景）壳：由 App 注入左右坞与视口 */
export default function Scene3dShell({ children }: { children: ReactNode }) {
  return <>{children}</>;
}
