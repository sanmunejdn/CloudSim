#ifndef ROBOTURDF_URDFLINKBACKENDMANAGER_H
#define ROBOTURDF_URDFLINKBACKENDMANAGER_H

/// @file UrdfLinkBackendManager.h
/// @brief 连杆 mesh 后端：一连杆一 MeshBackendData，替代 osgDB 直读以支持属性与序列化

#include "roboturdf_global.h"

#include <QHash>
#include <QString>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <osg/Node>
#include <osg/ref_ptr>

class MeshBackendData;
struct MeshVisualOptions;

namespace osg
{
class Group;
class MatrixTransform;
} // namespace osg

/// 连杆 mesh 后端：一连杆一 MeshBackendData，替代 osgDB 直读以支持属性与序列化
class ROBOTURDF_EXPORT UrdfLinkBackendManager
{
public:
	UrdfLinkBackendManager();
	~UrdfLinkBackendManager() = default;

	void clear();

	bool hasLinkBackend(const QString& linkName) const;

	/// @param visualOriginMatrix 4×4 列主序，来自 URDF visual origin
	std::shared_ptr<MeshBackendData> createLinkBackend(const QString& linkName, const QString& meshPath,
													   const std::vector<double>& visualOriginMatrix,
													   QString* errorMessage = nullptr);

	osg::ref_ptr<osg::Node> createLinkVisualNode(const QString& linkName, const MeshVisualOptions& options,
												 QString* errorMessage = nullptr);

	std::shared_ptr<MeshBackendData> getLinkBackend(const QString& linkName) const;

	std::string getLinkBackendId(const QString& linkName) const;

	std::vector<QString> getAllLinkNames() const;

	size_t getBackendCount() const { return m_linkNameToBackend.size(); }

	int batchCreateLinkBackends(const QHash<QString, QString>& linkMeshPaths, const QString& robotName,
								QString* errorMessage = nullptr);

	int batchCreateVisualNodes(const QHash<QString, osg::Group*>& linkContainers, const MeshVisualOptions& options,
							   QString* errorMessage = nullptr);

	void setRobotName(const QString& robotName) { m_robotName = robotName; }
	QString robotName() const { return m_robotName; }

	/// A/B 或回退：false 时用 osgDB 直读
	static void setUseBackendLoading(bool use) { s_useBackendLoading = use; }
	static bool useBackendLoading() { return s_useBackendLoading; }

	struct Stats
	{
		size_t totalBackends = 0;
		size_t totalTriangleCount = 0;
		double avgLoadTimeMs = 0.0;
	};
	Stats getStats() const;

private:
	std::unordered_map<QString, std::shared_ptr<MeshBackendData>> m_linkNameToBackend;
	std::unordered_map<QString, std::string> m_linkNameToBackendId;
	QString m_robotName;

	mutable std::vector<double> m_loadTimes;

	static bool s_useBackendLoading;

	std::string generateBackendId(const QString& linkName) const;
};

#endif // ROBOTURDF_URDFLINKBACKENDMANAGER_H
