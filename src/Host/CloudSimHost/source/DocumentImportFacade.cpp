#include "DocumentImportFacade.h"

#include "BackendFileImport.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"

#include <QFile>
#include <QFileInfo>

namespace cloudsim::host
{

ImportFileResult importFileIntoDocument(DocumentHost& host, const QString& filePath, const ImportFileKind kind,
	const cloudsim::core::ImportOptionsDto& options, QString* outError)
{
	ImportFileResult result;
	const QFileInfo fileInfo(filePath);
	if (filePath.isEmpty() || !fileInfo.exists())
	{
		if (outError)
		{
			*outError = QStringLiteral("File not found.");
		}
		return result;
	}

	if (kind == ImportFileKind::PointCloud)
	{
		const QString id = importPointCloudFile(host, filePath, options, outError);
		if (id.isEmpty())
		{
			return result;
		}
		result.ok = true;
		result.rootBackendId = id;
		return result;
	}

	const QString ext = fileInfo.suffix().toLower();
	static const QStringList kSimpleMesh{QStringLiteral("obj"), QStringLiteral("stl"), QStringLiteral("ply"),
		QStringLiteral("off")};
	if (kSimpleMesh.contains(ext))
	{
		const QString id = importMeshFile(host, filePath, options, outError);
		if (id.isEmpty())
		{
			return result;
		}
		result.ok = true;
		result.rootBackendId = id;
		return result;
	}

	// 空回调：层级分件世界坐标，勿在导入期写 Follow（见 HierarchyMeshImport）
	const HierarchyFollowBindingFn followBinding;
	QString extendedErr;
	if (!importMeshFileExtended(host, filePath, options.catalogTypeName, options.quietUi, followBinding,
			result.hierarchyDetail, &extendedErr))
	{
		if (outError)
		{
			*outError = extendedErr.isEmpty() ? QStringLiteral("Unsupported mesh import path.") : extendedErr;
		}
		return result;
	}
	if (!result.hierarchyDetail.ok)
	{
		if (outError)
		{
			*outError = extendedErr.isEmpty() ? QStringLiteral("Import failed.") : extendedErr;
		}
		return result;
	}

	result.ok = true;
	result.hierarchyImport = true;
	result.skipFollowOnImport = true;

	OsgWidget* osg = osgWidgetFrom(host);
	if (osg)
	{
		if (result.hierarchyDetail.importParent)
		{
			result.rootBackendId = QString::fromStdString(result.hierarchyDetail.importParent->id());
			// 逻辑子树聚合包围球，非单片 mesh home
			osg->focusCameraOnBackend(result.hierarchyDetail.importParent->id());
			osg->requestRedraw();
		}
		else if (result.hierarchyDetail.lastRegisteredMesh)
		{
			result.rootBackendId = QString::fromStdString(result.hierarchyDetail.lastRegisteredMesh->id());
			QString homeErr;
			(void)osg->loadMeshFromBackendData(*result.hierarchyDetail.lastRegisteredMesh, &homeErr, options.resetViewToHome);
		}
	}
	return result;
}

AdoptRegistrationResult registerAdoptedMesh(DocumentHost& host, const std::shared_ptr<MeshBackendData>& mesh,
	const AdoptMeshOptions& options, QString* outError)
{
	AdoptRegistrationResult result;
	if (!mesh)
	{
		if (outError)
		{
			*outError = QStringLiteral("Mesh is null.");
		}
		return result;
	}
	QString regErr;
	if (!registerAdoptedMeshAndLoadScene(host, mesh, options.sourcePath, options.catalogTypeName, options.parentId,
			options.resetViewToHome, &regErr, options.skipInnerModelCenterRebase, options.linkOsgSceneParent))
	{
		if (outError)
		{
			*outError = regErr.isEmpty() ? QStringLiteral("Failed to register mesh.") : regErr;
		}
		return result;
	}
	result.ok = true;
	result.backendId = QString::fromStdString(mesh->id());
	return result;
}

AdoptRegistrationResult registerAdoptedPointCloud(DocumentHost& host,
	const std::shared_ptr<PointCloudBackendData>& pointCloud, const AdoptPointCloudOptions& options, QString* outError)
{
	AdoptRegistrationResult result;
	if (!pointCloud)
	{
		if (outError)
		{
			*outError = QStringLiteral("Point cloud is null.");
		}
		return result;
	}
	QString regErr;
	if (!registerAdoptedPointCloudAndLoadScene(host, pointCloud, options.sourcePath, options.catalogTypeName,
			options.resetViewToHome, &regErr))
	{
		if (outError)
		{
			*outError = regErr.isEmpty() ? QStringLiteral("Failed to register point cloud.") : regErr;
		}
		return result;
	}
	result.ok = true;
	result.backendId = QString::fromStdString(pointCloud->id());
	return result;
}

struct PointCloudBackgroundLoadState::Impl
{
	QString filePath;
	std::shared_ptr<PointCloudBackendData> pointCloud;
};

PointCloudBackgroundLoadState::PointCloudBackgroundLoadState(const QString& filePath, const QString& displayName)
	: m_impl(std::make_unique<Impl>())
{
	m_impl->filePath = filePath;
	m_impl->pointCloud = std::make_shared<PointCloudBackendData>();
	m_impl->pointCloud->setName(displayName.toStdString());
	BackendColor color;
	color.r = 0.65f;
	color.g = 0.82f;
	color.b = 0.95f;
	color.a = 1.0f;
	m_impl->pointCloud->setColor(color);
	m_impl->pointCloud->setPose(BackendVec3{});
	m_impl->pointCloud->setRotation(BackendVec3{});
}

PointCloudBackgroundLoadState::~PointCloudBackgroundLoadState() = default;

bool PointCloudBackgroundLoadState::executeLoad(
	const std::function<void(double progress01, const QString& status)>& progress, QString* outError)
{
	if (!m_impl || !m_impl->pointCloud)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid point cloud load state.");
		}
		return false;
	}
	if (progress)
	{
		progress(0.05, QStringLiteral("CGAL read / decode..."));
	}
	const QByteArray nativeEnc = QFile::encodeName(m_impl->filePath);
	const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));
	std::string loadErr;
	const bool ok = m_impl->pointCloud->loadFromFile(nativePath, &loadErr);
	if (progress)
	{
		progress(1.0, QString());
	}
	if (!ok && outError)
	{
		*outError = loadErr.empty() ? QStringLiteral("Failed to load point cloud.") : QString::fromStdString(loadErr);
	}
	return ok;
}

AdoptRegistrationResult PointCloudBackgroundLoadState::adoptIntoDocument(DocumentHost& host,
	const AdoptPointCloudOptions& options, QString* outError)
{
	if (!m_impl || !m_impl->pointCloud)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid point cloud load state.");
		}
		return {};
	}
	return registerAdoptedPointCloud(host, m_impl->pointCloud, options, outError);
}

QString ImportFileResult::hierarchyFocusBackendId() const
{
	if (hierarchyDetail.importParent)
	{
		return QString::fromStdString(hierarchyDetail.importParent->id());
	}
	return {};
}

QString ImportFileResult::hierarchyLastMeshBackendId() const
{
	if (hierarchyDetail.lastRegisteredMesh)
	{
		return QString::fromStdString(hierarchyDetail.lastRegisteredMesh->id());
	}
	return {};
}

} // namespace cloudsim::host
