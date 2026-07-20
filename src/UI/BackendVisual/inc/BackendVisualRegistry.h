#ifndef BACKENDVISUAL_BACKENDVISUALREGISTRY_H
#define BACKENDVISUAL_BACKENDVISUALREGISTRY_H

/// @file BackendVisualRegistry.h
/// @brief 按 BackendDataBase::className 注册 IBackendVisual 工厂

#include "backendvisual_global.h"

#include "BackendDataBase.h"
#include "IBackendVisual.h"

#include <functional>
#include <memory>
#include <string>

#include <osg/Geode>
#include <osg/Node>

class MeshBackendData;
class PointCloudBackendData;

/// 按 BackendDataBase::className 注册 IBackendVisual 工厂
class BACKENDVISUAL_EXPORT BackendVisualRegistry
{
public:
	using Factory = std::function<std::unique_ptr<IBackendVisual>()>;

	static void registerType(const std::string& className, Factory factory);
	static void ensureBuiltinsRegistered();

	static std::unique_ptr<IBackendVisual> createForClassName(const std::string& className);

	static bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions,
								 BranchBuildResult& out, std::string* errorMessage);

	static void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter, float& outDiagonal);

	/// 导入预览用（单 geode，无 PAT）
	static osg::ref_ptr<osg::Geode> buildPointCloudGeode(const PointCloudBackendData& data, std::string* errorMessage);

	static osg::ref_ptr<osg::Node> buildMeshDisplayNode(const MeshBackendData& data, const MeshVisualOptions& options,
														std::string* errorMessage);
};

#endif // BACKENDVISUAL_BACKENDVISUALREGISTRY_H
