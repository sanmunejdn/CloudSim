#pragma once

#include "IBackendVisual.h"
#include "backendvisual_global.h"

#include <osg/Geode>
#include <osg/ref_ptr>

class PointCloudBackendData;

class BACKENDVISUAL_EXPORT PointCloudBackendVisual : public IBackendVisual
{
public:
	/// 导入预览：单 geode，无 PAT 包装
	osg::ref_ptr<osg::Geode> makeStagingGeode(const PointCloudBackendData& data, std::string* errorMessage) const;

	std::string typeKey() const override;
	bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
		std::string* errorMessage) override;
	void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter, float& outDiagonal) const override;
};
