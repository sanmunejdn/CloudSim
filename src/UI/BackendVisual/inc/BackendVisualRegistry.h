#pragma once

#include "IBackendVisual.h"
#include "backendvisual_global.h"

#include <functional>
#include <memory>
#include <osg/Geode>
#include <osg/Node>
#include <string>

#include "BackendDataBase.h"

class MeshBackendData;
class PointCloudBackendData;

/// Registers \ref IBackendVisual factories by \ref BackendDataBase::className().
class BACKENDVISUAL_EXPORT BackendVisualRegistry
{
public:
	using Factory = std::function<std::unique_ptr<IBackendVisual>()>;

	static void registerType(const std::string& className, Factory factory);
	static void ensureBuiltinsRegistered();

	static std::unique_ptr<IBackendVisual> createForClassName(const std::string& className);

	static bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
		std::string* errorMessage);

	static void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter, float& outDiagonal);

	/// Used by staging / import preview (single geode, no PAT wrapper).
	static osg::ref_ptr<osg::Geode> buildPointCloudGeode(const PointCloudBackendData& data, std::string* errorMessage);

	static osg::ref_ptr<osg::Node> buildMeshDisplayNode(const MeshBackendData& data, const MeshVisualOptions& options,
		std::string* errorMessage);
};
