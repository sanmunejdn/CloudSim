#ifndef ROBOTSCENE_CUSTOMDEVICEASSEMBLYCOMMIT_H
#define ROBOTSCENE_CUSTOMDEVICEASSEMBLYCOMMIT_H

/// @file CustomDeviceAssemblyCommit.h
/// @brief 组装画布 Link/Joint → 设备 rest/烘焙/q 提交（无 UI）

#include "robot_scene_global.h"

#include "CustomDeviceBackendData.h"

#include <vector>

class BackendDataManager;
class IRobotBackendPoseSink;

namespace CustomDeviceAssemblyCommit
{
/// 写 links/joints/homes 并 applyQ；调用前应已 captureBaseWorldW0（本函数内也会再 capture）
ROBOT_SCENE_API bool commitGraph(CustomDeviceBackendData& device, const std::vector<CustomDeviceLink>& links,
								 const std::vector<CustomDeviceJoint>& joints, BackendDataManager& backend,
								 IRobotBackendPoseSink* sink);
} // namespace CustomDeviceAssemblyCommit

#endif // ROBOTSCENE_CUSTOMDEVICEASSEMBLYCOMMIT_H
