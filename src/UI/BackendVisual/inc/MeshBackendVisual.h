#ifndef BACKENDVISUAL_MESHBACKENDVISUAL_H
#define BACKENDVISUAL_MESHBACKENDVISUAL_H

/// @file MeshBackendVisual.h
/// @brief MeshBackendVisual 接口

#include "backendvisual_global.h"

#include "IBackendVisual.h"

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
	void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
									   float& outDiagonal) const override;
};

#endif // BACKENDVISUAL_MESHBACKENDVISUAL_H
