#pragma once

#include "IBackendVisual.h"
#include "backendvisual_global.h"

#include <osg/Node>
#include <osg/ref_ptr>

class MeshBackendData;

class BACKENDVISUAL_EXPORT MeshBackendVisual : public IBackendVisual
{
public:
	osg::ref_ptr<osg::Node> makeDisplayNode(const MeshBackendData& data, const MeshVisualOptions& options,
		std::string* errorMessage) const;

	std::string typeKey() const override;
	bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
		std::string* errorMessage) override;
	void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter, float& outDiagonal) const override;
};
