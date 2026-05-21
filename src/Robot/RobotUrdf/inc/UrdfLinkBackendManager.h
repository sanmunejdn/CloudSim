#pragma once

#include "roboturdf_global.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <QString>
#include <QHash>

#include <osg/ref_ptr>
#include <osg/Node>

class MeshBackendData;
struct MeshVisualOptions;

namespace osg {
class Group;
class MatrixTransform;
}

/// URDF连杆后端管理器：为每个连杆创建MeshBackendData后端对象
/// 实现"一个连杆 = 一个后端对象 = 一个OSG节点"的映射关系
///
/// 设计目标：
/// 1. 加速mesh加载（使用MeshBackendData::loadFromFile替代osgDB::readNodeFile）
/// 2. 支持属性编辑（颜色、位置、旋转）
/// 3. 支持项目序列化（保存/加载）
/// 4. 统一后端管理接口
///
/// 使用流程：
/// 1. 为每个连杆调用 createLinkBackend() 创建后端对象
/// 2. 调用 createLinkVisualNode() 生成OSG可视化节点
/// 3. 将OSG节点挂接到连杆容器
/// 4. 可选：通过 getLinkBackend() 获取后端进行属性修改
class ROBOTURDF_EXPORT UrdfLinkBackendManager
{
public:
	UrdfLinkBackendManager();
	~UrdfLinkBackendManager() = default;

	/// 清除所有管理的后端对象
	void clear();

	/// 检查指定连杆是否有后端对象
	bool hasLinkBackend(const QString& linkName) const;

	/// 为连杆创建后端对象（如果已存在则返回现有的）
	/// @param linkName 连杆名称
	/// @param meshPath mesh文件绝对路径
	/// @param visualOrigin 视觉原点变换矩阵（来自URDF <visual><origin>）
	/// @param errorMessage 错误信息输出
	/// @return 创建的后端对象，失败返回nullptr
	std::shared_ptr<MeshBackendData> createLinkBackend(
		const QString& linkName,
		const QString& meshPath,
		const std::vector<double>& visualOriginMatrix, // 4x4 column-major
		QString* errorMessage = nullptr);

	/// 通过BackendVisualRegistry为指定连杆创建OSG可视化节点
	/// @param linkName 连杆名称
	/// @param options 可视化选项（光照、线框等）
	/// @param errorMessage 错误信息输出
	/// @return OSG节点，失败返回nullptr
	osg::ref_ptr<osg::Node> createLinkVisualNode(
		const QString& linkName,
		const MeshVisualOptions& options,
		QString* errorMessage = nullptr);

	/// 获取连杆的后端对象
	std::shared_ptr<MeshBackendData> getLinkBackend(const QString& linkName) const;

	/// 获取连杆的后端ID（用于注册到OsgWidget）
	std::string getLinkBackendId(const QString& linkName) const;

	/// 获取所有管理的连杆名称列表
	std::vector<QString> getAllLinkNames() const;

	/// 获取所有后端对象数量
	size_t getBackendCount() const { return m_linkNameToBackend.size(); }

	/// 批量创建连杆后端（用于整个URDF加载）
	/// @param linkMeshPaths 连杆名到mesh路径的映射
	/// @param robotName 机器人名称（用于生成backendId前缀）
	/// @return 成功创建的连杆数量
	int batchCreateLinkBackends(
		const QHash<QString, QString>& linkMeshPaths,
		const QString& robotName,
		QString* errorMessage = nullptr);

	/// 批量创建OSG节点（用于整个场景构建）
	/// @param parentGroup 父级OSG Group节点
	/// @param linkContainers 连杆名到容器Group的映射
	/// @param options 可视化选项
	/// @return 成功创建的节点数量
	int batchCreateVisualNodes(
		const QHash<QString, osg::Group*>& linkContainers,
		const MeshVisualOptions& options,
		QString* errorMessage = nullptr);

	/// 设置机器人名称前缀（用于生成唯一的backendId）
	void setRobotName(const QString& robotName) { m_robotName = robotName; }
	QString robotName() const { return m_robotName; }

	/// 切换使用OSG直接读取或后端对象读取（用于A/B测试和回退）
	static void setUseBackendLoading(bool use) { s_useBackendLoading = use; }
	static bool useBackendLoading() { return s_useBackendLoading; }

	/// 统计信息
	struct Stats {
		size_t totalBackends = 0;
		size_t totalTriangleCount = 0;
		double avgLoadTimeMs = 0.0;
	};
	Stats getStats() const;

private:
	std::unordered_map<QString, std::shared_ptr<MeshBackendData>> m_linkNameToBackend;
	std::unordered_map<QString, std::string> m_linkNameToBackendId;
	QString m_robotName;

	// 统计
	mutable std::vector<double> m_loadTimes; // 毫秒

	// 配置开关
	static bool s_useBackendLoading;

	std::string generateBackendId(const QString& linkName) const;
};
