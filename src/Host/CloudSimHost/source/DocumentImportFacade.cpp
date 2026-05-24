#include "DocumentImportFacade.h"

#include "BackendFileImport.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"

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

} // namespace cloudsim::host
