#ifndef BACKENDVISUAL_CUSTOMDEVICEBACKENDVISUAL_H
#define BACKENDVISUAL_CUSTOMDEVICEBACKENDVISUAL_H

/// @file CustomDeviceBackendVisual.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 自定义设备根：外层 MT + RGB 示意轴（几何在子件）

#include "backendvisual_global.h"

#include "IBackendVisual.h"

class BACKENDVISUAL_EXPORT CustomDeviceBackendVisual : public IBackendVisual
{
public:
	std::string typeKey() const override;
	bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
						  std::string* errorMessage) override;
	void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
									   float& outDiagonal) const override;
};

#endif // BACKENDVISUAL_CUSTOMDEVICEBACKENDVISUAL_H
