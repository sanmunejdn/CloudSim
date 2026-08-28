#ifndef ROBOTSCENE_ROBOTINSTRUCTIONIKCONTEXT_H
#define ROBOTSCENE_ROBOTINSTRUCTIONIKCONTEXT_H

/// @file RobotInstructionIkContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 规划前工具上下文：sync 与 prepare 同一套解析（跟随 active 不用冻结 id）

#include "robot_scene_global.h"

#include "RobotCoordinateFrames.h"
#include "RobotInstructionModel.h"

#include <string>
#include <vector>

namespace RobotInstruction
{
/// motion.tool.frameId 为空或 "active" 时跟随全局激活工具系
ROBOT_SCENE_API bool motionUsesActiveToolFrame(const Base& ins);

/// 将工具系写入 instruction context；跟随 active 的路点用 frames.activeToolFrame，否则 resolve motion.tool.frameId
ROBOT_SCENE_API void syncToolContextFromFrames(Base& ins, const RobotCoordinate::RobotCoordinateFrameSet& frames);

/// 规划准备：先 sync 工具上下文，再写种子 / urdf / tcp / flange。禁止用 rollingQ 的 FK 覆盖指令位姿
ROBOT_SCENE_API void prepareInstructionIkContext(Base& ins, const std::vector<double>& rollingQ,
												 const std::string& urdfPath, const std::string& defaultTcpLinkName,
												 const RobotCoordinate::RobotCoordinateFrameSet* frames);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONIKCONTEXT_H
