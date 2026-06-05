#pragma once

#include "IBackendVisual.h"
#include "backendvisual_global.h"

class BrepBackendData;

class BACKENDVISUAL_EXPORT BrepBackendVisual : public IBackendVisual
{
public:
	std::string typeKey() const override;
	bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
		std::string* errorMessage) override;
	void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter, float& outDiagonal) const override;
};
