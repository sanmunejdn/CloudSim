#include "UrdfLinkBackendManager.h"

#include "MeshBackendData.h"
#include "BackendVisualRegistry.h"
#include "MeshBackendVisual.h"

#include <QDebug>
#include <QElapsedTimer>

#include <osg/Group.h>
#include <osg/MatrixTransform.h>

#include <algorithm>
#include <numeric>

// 默认开启后端加载
bool UrdfLinkBackendManager::s_useBackendLoading = true;

UrdfLinkBackendManager::UrdfLinkBackendManager()
	: m_robotName("URDF_Robot")
{
}

void UrdfLinkBackendManager::clear()
{
	m_linkNameToBackend.clear();
	m_linkNameToBackendId.clear();
	m_loadTimes.clear();
}

bool UrdfLinkBackendManager::hasLinkBackend(const QString& linkName) const
{
	return m_linkNameToBackend.find(linkName) != m_linkNameToBackend.end();
}

std::string UrdfLinkBackendManager::generateBackendId(const QString& linkName) const
{
	// 生成格式: URDF_{RobotName}_{LinkName}_{Counter}
	static int s_counter = 0;
	return "URDF_" + m_robotName.toStdString() + "_" + linkName.toStdString() + "_" + std::to_string(++s_counter);
}

std::shared_ptr<MeshBackendData> UrdfLinkBackendManager::createLinkBackend(
	const QString& linkName,
	const QString& meshPath,
	const std::vector<double>& visualOriginMatrix,
	QString* errorMessage)
{
	if (linkName.isEmpty()) {
		if (errorMessage) *errorMessage = QStringLiteral("Link name is empty");
		return nullptr;
	}

	// 检查是否已存在
	auto it = m_linkNameToBackend.find(linkName);
	if (it != m_linkNameToBackend.end()) {
		qDebug() << "[UrdfLinkBackendManager] Backend already exists for link:" << linkName;
		return it->second;
	}

	// 创建新的后端对象
	auto backend = std::make_shared<MeshBackendData>();
	backend->setName(linkName.toStdString());

	std::string backendId = generateBackendId(linkName);
	backend->setId(backendId);

	// 计时开始
	QElapsedTimer timer;
	timer.start();

	// 加载mesh文件
	std::string nativePath = meshPath.toStdString();
	std::string loadErr;
	bool loaded = backend->loadFromFile(nativePath, &loadErr);

	// 记录加载时间
	double elapsedMs = timer.elapsed();
	m_loadTimes.push_back(elapsedMs);

	if (!loaded) {
		if (errorMessage) {
			*errorMessage = QStringLiteral("Failed to load mesh for link '%1': %2")
				.arg(linkName).arg(QString::fromStdString(loadErr));
		}
		qWarning() << "[UrdfLinkBackendManager] Failed to load mesh:" << meshPath
				   << "Error:" << QString::fromStdString(loadErr);
		// 创建一个空的后端对象（允许无几何的连杆）
		backend->clearGeometry();
	}

	// 设置默认颜色（淡蓝色）
	BackendColor color;
	color.r = 0.65f;
	color.g = 0.82f;
	color.b = 0.95f;
	color.a = 1.0f;
	backend->setColor(color);

	// 如果有visual origin，转换为后端的位置/旋转
	if (visualOriginMatrix.size() == 16) {
		// 提取平移部分作为位置
		BackendVec3 pos;
		pos.x = static_cast<float>(visualOriginMatrix[12]);  // m[3][0] in column-major
		pos.y = static_cast<float>(visualOriginMatrix[13]);  // m[3][1]
		pos.z = static_cast<float>(visualOriginMatrix[14]);  // m[3][2]
		backend->setPose(pos);

		// 从矩阵提取欧拉角（简化版本，实际可能需要更复杂的分解）
		// 这里仅作为示例，实际应使用mat4ToEuler函数
		BackendVec3 rot{0.0f, 0.0f, 0.0f};
		backend->setRotation(rot);
	}

	// 存储映射
	m_linkNameToBackend[linkName] = backend;
	m_linkNameToBackendId[linkName] = backendId;

	qDebug().nospace() << "[UrdfLinkBackendManager] Created backend for link: " << linkName
					   << " ID: " << QString::fromStdString(backendId)
					   << " Load time: " << elapsedMs << "ms"
					   << " Triangles: " << backend->geometryElementCount();

	return backend;
}

osg::ref_ptr<osg::Node> UrdfLinkBackendManager::createLinkVisualNode(
	const QString& linkName,
	const MeshVisualOptions& options,
	QString* errorMessage)
{
	auto backend = getLinkBackend(linkName);
	if (!backend) {
		if (errorMessage) *errorMessage = QStringLiteral("No backend found for link: %1").arg(linkName);
		return nullptr;
	}

	// 通过BackendVisualRegistry创建OSG节点
	std::string err;
	osg::ref_ptr<osg::Node> visualNode = BackendVisualRegistry::buildMeshDisplayNode(
		*backend, options, &err);

	if (!visualNode) {
		if (errorMessage) *errorMessage = QString::fromStdString(err);
		qWarning() << "[UrdfLinkBackendManager] Failed to create visual node for link:" << linkName
				   << "Error:" << QString::fromStdString(err);
		return nullptr;
	}

	// 设置节点名称便于调试
	visualNode->setName(linkName.toStdString() + "_Visual");

	return visualNode;
}

std::shared_ptr<MeshBackendData> UrdfLinkBackendManager::getLinkBackend(const QString& linkName) const
{
	auto it = m_linkNameToBackend.find(linkName);
	if (it != m_linkNameToBackend.end()) {
		return it->second;
	}
	return nullptr;
}

std::string UrdfLinkBackendManager::getLinkBackendId(const QString& linkName) const
{
	auto it = m_linkNameToBackendId.find(linkName);
	if (it != m_linkNameToBackendId.end()) {
		return it->second;
	}
	return {};
}

std::vector<QString> UrdfLinkBackendManager::getAllLinkNames() const
{
	std::vector<QString> names;
	names.reserve(m_linkNameToBackend.size());
	for (const auto& kv : m_linkNameToBackend) {
		names.push_back(kv.first);
	}
	return names;
}

int UrdfLinkBackendManager::batchCreateLinkBackends(
	const QHash<QString, QString>& linkMeshPaths,
	const QString& robotName,
	QString* errorMessage)
{
	m_robotName = robotName;
	int successCount = 0;

	for (auto it = linkMeshPaths.begin(); it != linkMeshPaths.end(); ++it) {
		const QString& linkName = it.key();
		const QString& meshPath = it.value();

		QString err;
		auto backend = createLinkBackend(linkName, meshPath, {}, &err);
		if (backend) {
			++successCount;
		} else {
			qWarning() << "[UrdfLinkBackendManager] Failed to create backend for link:" << linkName
					   << "Error:" << err;
			if (errorMessage && errorMessage->isEmpty()) {
				*errorMessage = err;
			}
		}
	}

	qDebug() << "[UrdfLinkBackendManager] Batch created" << successCount << "/" << linkMeshPaths.size() << "backends";
	return successCount;
}

int UrdfLinkBackendManager::batchCreateVisualNodes(
	const QHash<QString, osg::Group*>& linkContainers,
	const MeshVisualOptions& options,
	QString* errorMessage)
{
	int successCount = 0;

	for (auto it = linkContainers.begin(); it != linkContainers.end(); ++it) {
		const QString& linkName = it.key();
		osg::Group* container = it.value();

		if (!container) {
			qWarning() << "[UrdfLinkBackendManager] Null container for link:" << linkName;
			continue;
		}

		QString err;
		osg::ref_ptr<osg::Node> visualNode = createLinkVisualNode(linkName, options, &err);
		if (visualNode) {
			container->addChild(visualNode.get());
			++successCount;
		} else {
			qWarning() << "[UrdfLinkBackendManager] Failed to create visual for link:" << linkName
					   << "Error:" << err;
			if (errorMessage && errorMessage->isEmpty()) {
				*errorMessage = err;
			}
		}
	}

	qDebug() << "[UrdfLinkBackendManager] Batch created" << successCount << "/" << linkContainers.size() << "visual nodes";
	return successCount;
}

UrdfLinkBackendManager::Stats UrdfLinkBackendManager::getStats() const
{
	Stats stats;
	stats.totalBackends = m_linkNameToBackend.size();

	size_t totalTris = 0;
	for (const auto& kv : m_linkNameToBackend) {
		totalTris += kv.second->geometryElementCount();
	}
	stats.totalTriangleCount = totalTris;

	if (!m_loadTimes.empty()) {
		double sum = std::accumulate(m_loadTimes.begin(), m_loadTimes.end(), 0.0);
		stats.avgLoadTimeMs = sum / m_loadTimes.size();
	}

	return stats;
}
