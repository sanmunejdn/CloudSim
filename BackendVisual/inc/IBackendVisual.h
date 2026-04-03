#pragma once

#include "backendvisual_global.h"

#include <memory>
#include <osg/PositionAttitudeTransform>
#include <osg/ref_ptr>
#include <osg/Vec3f>
#include <string>

class BackendDataBase;

/// Options only used by mesh visuals; point cloud builds ignore these.
struct MeshVisualOptions
{
	bool showWireOutline = true;
	bool useSceneLighting = false;
};

struct BranchBuildResult
{
	osg::ref_ptr<osg::PositionAttitudeTransform> outer;
	osg::Vec3f modelCenter{};
	float diagonal = 1.0f;
};

/// Per backend-type strategy: scene branch construction and metric extraction for gizmo/pick sync.
class BACKENDVISUAL_EXPORT IBackendVisual
{
public:
	virtual ~IBackendVisual() = default;

	virtual std::string typeKey() const = 0;

	virtual bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
		std::string* errorMessage) = 0;

	virtual void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter, float& outDiagonal) const = 0;
};
