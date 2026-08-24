#ifndef ROBOTSCENE_ROBOTEXTERNALAXISSCENEAPPLY_H
#define ROBOTSCENE_ROBOTEXTERNALAXISSCENEAPPLY_H

/// @file RobotExternalAxisSceneApply.h
/// @brief 外轴 q 写文档 + 工件 PAT（从 UI 层抽离，供 Composite apply 复用）

#include "robot_scene_global.h"

#include <vector>

class IRobotBackendPoseSink;
class IRobotSimulationDocument;

namespace RobotExternalAxisSceneApply
{
ROBOT_SCENE_API bool applyExternalAxisQ(IRobotSimulationDocument* doc, IRobotBackendPoseSink* sink, int instanceIndex,
										const std::vector<double>& fullQ);
}

#endif // ROBOTSCENE_ROBOTEXTERNALAXISSCENEAPPLY_H
