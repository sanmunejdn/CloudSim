#ifndef BACKENDVISUAL_FRAMEBACKENDVISUAL_H
#define BACKENDVISUAL_FRAMEBACKENDVISUAL_H

/// @file FrameBackendVisual.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 坐标系后端：外层 MT + RGB 三轴

#include "backendvisual_global.h"

#include "IBackendVisual.h"

class BACKENDVISUAL_EXPORT FrameBackendVisual : public IBackendVisual
{
public:
	std::string typeKey() const override;
	bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
						  std::string* errorMessage) override;
	void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
									   float& outDiagonal) const override;
};

#endif // BACKENDVISUAL_FRAMEBACKENDVISUAL_H
