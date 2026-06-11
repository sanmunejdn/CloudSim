#pragma once

#include "CoreTypes.h"
#include "HierarchyMeshImport.h"
#include "cloudsim_host_global.h"

#include <QString>
#include <functional>
#include <memory>

class BrepBackendData;
class MeshBackendData;
class PointCloudBackendData;

namespace cloudsim::host {

class DocumentHost;

enum class ImportFileKind { Mesh, PointCloud };

struct CLOUDSIM_HOST_EXPORT ImportFileResult {
	QString rootBackendId; ///< 层级导入多为 importParent id
	bool ok = false;
	bool hierarchyImport = false;
	bool skipFollowOnImport = false; ///< DXF/STEP 分件为世界坐标，导入期勿 Follow
	HierarchyMeshImportResult hierarchyDetail;

	/// 层级导入后树聚焦 id（Widget 不接触 BackendDataBase）
	QString hierarchyFocusBackendId() const;
	QString hierarchyLastMeshBackendId() const;
};

/// 统一导入路由
CLOUDSIM_HOST_EXPORT ImportFileResult importFileIntoDocument(DocumentHost& host, const QString& filePath,
	ImportFileKind kind, const cloudsim::core::ImportOptionsDto& options, QString* outError = nullptr);

struct AdoptMeshOptions {
	QString sourcePath;
	QString catalogTypeName = QStringLiteral("Model");
	QString parentId;
	bool resetViewToHome = true;
	bool linkOsgSceneParent = true;
};

struct AdoptPointCloudOptions {
	QString sourcePath;
	QString catalogTypeName = QStringLiteral("PointCloud");
	bool resetViewToHome = true;
};

struct AdoptRegistrationResult {
	QString backendId;
	bool ok = false;
};

/// 已构造几何注册
CLOUDSIM_HOST_EXPORT AdoptRegistrationResult registerAdoptedMesh(DocumentHost& host,
	const std::shared_ptr<MeshBackendData>& mesh, const AdoptMeshOptions& options, QString* outError = nullptr);
CLOUDSIM_HOST_EXPORT AdoptRegistrationResult registerAdoptedPointCloud(DocumentHost& host,
	const std::shared_ptr<PointCloudBackendData>& pointCloud, const AdoptPointCloudOptions& options,
	QString* outError = nullptr);

/// 后台 Job 读点云文件（Widget 不接触 PointCloudBackendData）
class CLOUDSIM_HOST_EXPORT PointCloudBackgroundLoadState
{
public:
	explicit PointCloudBackgroundLoadState(const QString& filePath, const QString& displayName);
	~PointCloudBackgroundLoadState();

	PointCloudBackgroundLoadState(const PointCloudBackgroundLoadState&) = delete;
	PointCloudBackgroundLoadState& operator=(const PointCloudBackgroundLoadState&) = delete;

	bool executeLoad(const std::function<void(double progress01, const QString& status)>& progress,
		QString* outError = nullptr);
	AdoptRegistrationResult adoptIntoDocument(DocumentHost& host, const AdoptPointCloudOptions& options,
		QString* outError = nullptr);

private:
	struct Impl;
	std::unique_ptr<Impl> m_impl;
};

/// 后台 Job 读 STEP/Mesh/BREP（Widget 不阻塞 UI）
class CLOUDSIM_HOST_EXPORT ModelBackgroundLoadState
{
public:
	explicit ModelBackgroundLoadState(
		const QString& filePath,
		const QString& displayName,
		const QString& catalogTypeName,
		int meshImportQuality);
	~ModelBackgroundLoadState();

	ModelBackgroundLoadState(const ModelBackgroundLoadState&) = delete;
	ModelBackgroundLoadState& operator=(const ModelBackgroundLoadState&) = delete;

	bool executeLoad(const std::function<void(double progress01, const QString& status)>& progress,
		QString* outError = nullptr);
	ImportFileResult finishIntoDocument(DocumentHost& host, const cloudsim::core::ImportOptionsDto& options,
		QString* outError = nullptr);

	/// BREP 导入且 Phase1 已缓存时可后台预热 Phase2（边拾取/线框）
	bool needsPickArtifactWarm() const;
	bool warmPickArtifacts(QString* outError = nullptr);

private:
	struct Impl;
	std::unique_ptr<Impl> m_impl;
};

} // namespace cloudsim::host
