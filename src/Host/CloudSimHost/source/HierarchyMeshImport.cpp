#include "HierarchyMeshImport.h"

#include "BackendFileImport.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"

#include "BackendDataBase.h"
#include "MeshBackendData.h"
#include "Types.h"
#include "OsgWidget.h"
#include "OsgWidgetCaptureController.h"

#include <QByteArray>
#include <QFile>
#include <QFileInfo>
#include <QHash>

namespace cloudsim::host {

namespace {

bool registerHierarchyPartMeshes(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
	const QString& defaultBaseName, const QString& importParentDisplayName, const std::vector<MeshHierarchyPart>& parts,
	const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out, QString* outError)
{
	if (parts.empty())
	{
		return false;
	}
	auto importParent = std::make_shared<MeshBackendData>();
	const QString parentLabel = importParentDisplayName.isEmpty() ? defaultBaseName : importParentDisplayName;
	importParent->setName(parentLabel.toStdString());
	if (!registerAdoptedBackendObject(host, importParent, sourceFilePath, catalogTypeName, QString(), outError))
	{
		return false;
	}
	const QString importParentId = QString::fromStdString(importParent->id());
	QHash<QString, QString> pathToBackendId;
	std::shared_ptr<MeshBackendData> lastLoadedMesh;
	int registered = 0;
	for (const MeshHierarchyPart& p : parts)
	{
		if (p.triangleSoup.empty())
		{
			continue;
		}
		auto partMesh = std::make_shared<MeshBackendData>();
		partMesh->setTriangleSoup(p.triangleSoup);
		const QString displayName = p.displayName.empty() ? defaultBaseName : QString::fromStdString(p.displayName);
		partMesh->setName(displayName.toStdString());
		const QString partPath = QString::fromStdString(p.partPath);
		const QString parentPartPath = QString::fromStdString(p.parentPartPath);
		const QString parentId =
			pathToBackendId.contains(parentPartPath) ? pathToBackendId.value(parentPartPath) : importParentId;
		QString meshRegErr;
		if (!registerAdoptedMeshAndLoadScene(host, partMesh, sourceFilePath, catalogTypeName, parentId, false, &meshRegErr,
				true, false))
		{
			if (outError)
			{
				*outError = meshRegErr.isEmpty()
					? QStringLiteral("Failed to register hierarchical mesh part.")
					: meshRegErr;
			}
			return false;
		}
		const QString selfId = QString::fromStdString(partMesh->id());
		// 世界坐标分件：skipInnerModelCenterRebase + 无 Follow，避免 pose≈-质心
		(void)onParentFollow;
		pathToBackendId[partPath] = selfId;
		lastLoadedMesh = partMesh;
		++registered;
	}
	if (registered == 0)
	{
		return false;
	}
	out.ok = true;
	out.importParent = importParent;
	out.lastRegisteredMesh = lastLoadedMesh;
	out.registeredPartCount = registered;
	return true;
}

bool registerCapturedHierarchyParts(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
	const QString& defaultBaseName, const std::vector<MeshCapturedPart>& parts,
	const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out, QString* outError)
{
	if (parts.size() <= 1U)
	{
		return false;
	}
	auto importParent = std::make_shared<MeshBackendData>();
	importParent->setName(defaultBaseName.toStdString());
	if (!registerAdoptedBackendObject(host, importParent, sourceFilePath, catalogTypeName, QString(), outError))
	{
		return false;
	}
	const QString importParentId = QString::fromStdString(importParent->id());
	QHash<QString, QString> pathToBackendId;
	std::shared_ptr<MeshBackendData> lastLoadedMesh;
	int registered = 0;
	for (const MeshCapturedPart& p : parts)
	{
		if (p.triangleSoup.empty())
		{
			continue;
		}
		auto partMesh = std::make_shared<MeshBackendData>();
		partMesh->setTriangleSoup(p.triangleSoup);
		const QString displayName = p.displayName.isEmpty() ? defaultBaseName : p.displayName;
		partMesh->setName(displayName.toStdString());
		const QString parentId =
			pathToBackendId.contains(p.parentPartPath) ? pathToBackendId.value(p.parentPartPath) : importParentId;
		QString meshRegErr;
		if (!registerAdoptedMeshAndLoadScene(host, partMesh, sourceFilePath, catalogTypeName, parentId, false, &meshRegErr,
				true, false))
		{
			if (outError)
			{
				*outError = meshRegErr.isEmpty()
					? QStringLiteral("Failed to register hierarchical backend object.")
					: meshRegErr;
			}
			return false;
		}
		const QString selfId = QString::fromStdString(partMesh->id());
		(void)onParentFollow;
		pathToBackendId[p.partPath] = selfId;
		lastLoadedMesh = partMesh;
		++registered;
	}
	if (registered == 0)
	{
		return false;
	}
	out.ok = true;
	out.importParent = importParent;
	out.lastRegisteredMesh = lastLoadedMesh;
	out.registeredPartCount = registered;
	return true;
}

} // namespace

bool importMeshHierarchyParts(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
	const std::vector<MeshHierarchyPart>& parts, const QString& defaultBaseName,
	const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out, QString* outError,
	const QString& importParentDisplayName)
{
	out = {};
	if (!registerHierarchyPartMeshes(host, sourceFilePath, catalogTypeName, defaultBaseName, importParentDisplayName,
			parts, onParentFollow, out, outError))
	{
		if (outError && outError->isEmpty())
		{
			*outError = QStringLiteral("No mesh parts were registered.");
		}
		return false;
	}
	return true;
}

bool importMeshFileExtended(DocumentHost& host, const QString& filePath, const QString& catalogTypeName, const bool quietUi,
	const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out, QString* outError)
{
	(void)quietUi;
	out = {};
	const QFileInfo fileInfo(filePath);
	const QString ext = fileInfo.suffix().toLower();
	const QByteArray nativeEnc = QFile::encodeName(filePath);
	const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));
	const QString defaultBaseName = fileInfo.completeBaseName();

	if (ext == QLatin1String("dxf"))
	{
		std::vector<MeshHierarchyPart> dxfParts;
		std::string dxfErr;
		if (MeshBackendData::loadDxfHierarchyFromFile(nativePath, dxfParts, &dxfErr) && !dxfParts.empty())
		{
			if (importMeshHierarchyParts(host, filePath, catalogTypeName, dxfParts, defaultBaseName, onParentFollow, out,
					outError, fileInfo.fileName()))
			{
				return true;
			}
			out.ok = false;
			if (outError && outError->isEmpty())
			{
				*outError = QStringLiteral("DXF hierarchy import produced no registrable mesh parts.");
			}
			return true;
		}
	}
	if (ext == QLatin1String("step") || ext == QLatin1String("stp"))
	{
		std::vector<MeshHierarchyPart> stepParts;
		std::string stepErr;
		if (MeshBackendData::loadStepHierarchyFromFile(nativePath, stepParts, &stepErr) && stepParts.size() > 1U)
		{
			if (importMeshHierarchyParts(host, filePath, catalogTypeName, stepParts, defaultBaseName, onParentFollow, out,
					outError, fileInfo.fileName()))
			{
				return true;
			}
			out.ok = false;
			if (outError && outError->isEmpty())
			{
				*outError = QStringLiteral("STEP hierarchy import produced no registrable mesh parts.");
			}
			return true;
		}
	}

	auto mesh = std::make_shared<MeshBackendData>();
	mesh->setName(fileInfo.fileName().toStdString());
	std::string backendErr;
	const bool cgalOk = mesh->loadFromFile(nativePath, &backendErr);
	if (!cgalOk)
	{
		static const QStringList kOsgOnly{
			QStringLiteral("dae"), QStringLiteral("3ds"), QStringLiteral("fbx"),
		};
		OsgWidget* osg = osgWidgetFrom(host);
		if (!kOsgOnly.contains(ext) || !osg)
		{
			if (outError)
			{
				*outError = QString::fromStdString(
					backendErr.empty() ? std::string("Failed to load mesh.") : backendErr);
			}
			out.ok = false;
			return true;
		}
		QString importErr;
		if (!osg->importModelFile(filePath, &importErr))
		{
			if (outError)
			{
				*outError = importErr.isEmpty() ? QStringLiteral("Failed to import model.") : importErr;
			}
			out.ok = false;
			return true;
		}
		std::vector<MeshCapturedPart> parts;
		QString hierarchyErr;
		const bool hasHierarchy = osg->captureImportedMeshBackendHierarchy(parts, &hierarchyErr);
		if (hasHierarchy && parts.size() > 1U)
		{
			if (registerCapturedHierarchyParts(host, filePath, catalogTypeName, fileInfo.fileName(), parts, onParentFollow,
					out, outError))
			{
				osg->clearStagingGeometry();
				return true;
			}
			if (outError && outError->isEmpty())
			{
				*outError = hierarchyErr.isEmpty()
					? QStringLiteral("Failed to register hierarchical model parts.")
					: hierarchyErr;
			}
			osg->clearStagingGeometry();
			out.ok = false;
			return true;
		}
		QString capErr;
		if (!osg->captureImportedMeshBackend(*mesh, &capErr) || mesh->triangleSoup().empty())
		{
			osg->clearStagingGeometry();
			if (outError)
			{
				*outError = QStringLiteral("Could not copy mesh into backend.\n%1").arg(capErr);
			}
			out.ok = false;
			return true;
		}
		osg->clearStagingGeometry();
		QString regErr;
		if (!registerAdoptedMeshAndLoadScene(host, mesh, filePath, catalogTypeName, QString(), true, &regErr))
		{
			if (outError)
			{
				*outError = regErr.isEmpty() ? QStringLiteral("Failed to register mesh.") : regErr;
			}
			out.ok = false;
			return true;
		}
		out.ok = true;
		out.lastRegisteredMesh = mesh;
		out.registeredPartCount = 1;
		return true;
	}

	QString regErr;
	if (!registerAdoptedMeshAndLoadScene(host, mesh, filePath, catalogTypeName, QString(), true, &regErr))
	{
		if (outError)
		{
			*outError = regErr.isEmpty() ? QStringLiteral("Failed to register mesh.") : regErr;
		}
		out.ok = false;
		return true;
	}
	out.ok = true;
	out.lastRegisteredMesh = mesh;
	out.registeredPartCount = 1;
	return true;
}

} // namespace cloudsim::host
