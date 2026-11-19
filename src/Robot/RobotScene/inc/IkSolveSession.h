#ifndef ROBOTSCENE_IKSOLVESESSION_H
#define ROBOTSCENE_IKSOLVESESSION_H

/// @file IkSolveSession.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 臂+外轴统一会话：路径选择后委托 TeachIk

#include "robot_scene_global.h"

#include "RobotTeachIk.h"

namespace RobotInstruction
{
/// 按机型 externalCoupleMode 调整外轴策略后求解
ROBOT_SCENE_API RobotTeachIk::TeachIkResult runIkSolveSession(RobotTeachIk::TeachIkContext ctx);

/// 拖动联立会话（同样应用 CoupleMode）
ROBOT_SCENE_API RobotTeachIk::TeachIkResult runIkSolveSessionCoordinatedDrag(RobotTeachIk::TeachIkContext ctx,
																			double qExternalHintMm,
																			bool hasExternalHint);

} // namespace RobotInstruction

#endif // ROBOTSCENE_IKSOLVESESSION_H
