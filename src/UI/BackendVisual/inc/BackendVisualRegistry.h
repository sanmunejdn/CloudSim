#ifndef BACKENDVISUAL_BACKENDVISUALREGISTRY_H
#define BACKENDVISUAL_BACKENDVISUALREGISTRY_H

/// @file BackendVisualRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 按 BackendDataBase::className 注册 IBackendVisual 工厂

#include "backendvisual_global.h"

#include "BackendDataBase.h"
#include "IBackendVisual.h"

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <osg/Geode>
#include <osg/Node>

class MeshBackendData;
class PointCloudBackendData;

/// 按 BackendDataBase::className 注册 IBackendVisual 工厂
/// @note 公开构造供 ServiceRegistry 持有；业务路径须先 setProcessInstance（组合根）
class BACKENDVISUAL_EXPORT BackendVisualRegistry
{
public:
	using Factory = std::function<std::unique_ptr<IBackendVisual>()>;

	BackendVisualRegistry() = default;

	/// 宿主注册 ServiceRegistry 时写入，使 BackendVisual 层 instance() 与宿主同一对象
	static void setProcessInstance(BackendVisualRegistry* registry)
	{
		processInstanceSlot() = registry;
	}

	/// 业务路径须先 setProcessInstance（组合根）
	static BackendVisualRegistry& instance();

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

private:
	/// 定义在 .cpp，保证跨 DLL 同一 override 槽
	static BackendVisualRegistry*& processInstanceSlot();

	void registerTypeImpl(const std::string& className, Factory factory);
	void ensureBuiltinsRegisteredImpl();
	void registerBuiltins();
	std::unique_ptr<IBackendVisual> createForClassNameImpl(const std::string& className) const;
	bool buildOuterBranchImpl(const BackendDataBase& data, const MeshVisualOptions& meshOptions, BranchBuildResult& out,
							  std::string* errorMessage) const;
	void computeModelCenterAndDiagonalImpl(const BackendDataBase& data, osg::Vec3f& outCenter, float& outDiagonal) const;

	std::unordered_map<std::string, Factory> m_factories;
	mutable std::once_flag m_builtinsOnce;
};

#endif // BACKENDVISUAL_BACKENDVISUALREGISTRY_H
