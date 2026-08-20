#ifndef BACKENDVISUAL_BREPBACKENDVISUAL_H
#define BACKENDVISUAL_BREPBACKENDVISUAL_H

/// @file BrepBackendVisual.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief BrepBackendVisual 接口

#include "backendvisual_global.h"

#include "IBackendVisual.h"

#include <osg/Node>

class BrepBackendData;

class BACKENDVISUAL_EXPORT BrepBackendVisual : public IBackendVisual
{
public:
	std::string typeKey() const override;
	bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
						  std::string* errorMessage) override;
	void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
									   float& outDiagonal) const override;
};

/// 视口线框：BRep outer 隐藏填充、显示拓扑边；非 BRep 返回 false
BACKENDVISUAL_EXPORT bool applyBrepViewportWireframe(osg::Node* outerBranch, bool enabled);

#endif // BACKENDVISUAL_BREPBACKENDVISUAL_H
