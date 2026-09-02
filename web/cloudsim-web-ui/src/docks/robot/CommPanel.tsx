/** 通讯页：依赖外部 RobotCommBridge；无进程时明确报错，不假成功 */

export default function CommPanel() {
  return (
    <div className="robot-pane" id="robotComm">
      <h4 className="section-title">机器人通讯</h4>
      <p className="hint">
        桌面通讯页通过 <code>RobotCommSDK</code> 连接本机 <code>RobotCommBridge</code>（默认 TCP
        19610），再连实体机器人做关节镜像。
      </p>
      <p className="hint muted">
        网页端完整镜像轮询尚未接入 Host 定时器；请在桌面端完成在线联机，或后续补
        <code>/api/robot/comm/*</code> 后再在此操作。当前不提供假成功按钮。
      </p>
      <ul className="hint">
        <li>启动 tools/RobotCommBridge</li>
        <li>桌面「通讯」页：连 Bridge → 连机器人 → 镜像</li>
      </ul>
    </div>
  );
}
