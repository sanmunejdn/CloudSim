#ifndef ROBOTWIDGET_OSGWORKPIECEWORLDMATRIXPROVIDER_H
#define ROBOTWIDGET_OSGWORKPIECEWORLDMATRIXPROVIDER_H

/// @file OsgWorkpieceWorldMatrixProvider.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 桌面：从 OSG 场景根矩阵取工件世界位姿（与拾取/预览同源）

#include "robotwidget_global.h"

#include "IWorkpieceWorldMatrixProvider.h"

#include <string>

class IRobotOsgSceneOps;
class BackendDataBase;

/// 与 FeaturePickTransform / getBackendRootWorldMatrixOsg 同源；失败不回落 Backend
class ROBOTWIDGET_EXPORT OsgWorkpieceWorldMatrixProvider final : public IWorkpieceWorldMatrixProvider
{
public:
	explicit OsgWorkpieceWorldMatrixProvider(IRobotOsgSceneOps* osg) : m_osg(osg) {}

	bool getWorldMatrix(const std::string& backendId, BackendMat4& out) const override;

private:
	IRobotOsgSceneOps* m_osg = nullptr;
};

/// 按 backendId 把当前 OSG 世界矩阵写入 Backend（不依赖当前选中）
ROBOTWIDGET_EXPORT bool syncBackendWorldPoseFromOsg(IRobotOsgSceneOps* osg, BackendDataBase& data);

#endif // ROBOTWIDGET_OSGWORKPIECEWORLDMATRIXPROVIDER_H
