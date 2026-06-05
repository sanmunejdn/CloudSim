#include "HierarchyMeshImport.h"

#include "BackendFileImport.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"

#include "BackendDataBase.h"
#include "BrepBackendData.h"
#include "MeshBackendData.h"
#include "Types.h"
#include "OsgWidget.h"
#include "OsgWidgetCaptureController.h"

#include <ShapeHandle.h>

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

bool registerBrepHierarchyPartMeshes(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
	const QString& defaultBaseName, const QString& importParentDisplayName, const std::vector<BrepHierarchyPart>& parts,
	const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out, QString* outError)
{
	if (parts.empty())
	{
		return false;
	}
	auto importParent = std::make_shared<BrepBackendData>();
	const QString parentLabel = importParentDisplayName.isEmpty() ? defaultBaseName : importParentDisplayName;
	importParent->setName(parentLabel.toStdString());
	if (!parts.front().shapeRef.isNull())
	{
		importParent->setShape(parts.front().shapeRef);
	}
	if (!registerAdoptedBackendObject(host, importParent, sourceFilePath, QStringLiteral("BrepModel"), QString(),
			outError))
	{
		return false;
	}
	const QString importParentId = QString::fromStdString(importParent->id());
	const std::string importParentStdId = importParent->id();
	if (importParent->hasGeometry())
	{
		if (OsgWidget* osg = osgWidgetFrom(host))
		{
			QString sceneErr;
			if (!osg->loadBackendFromBackendData(*importParent, &sceneErr, false, true, true, true) && outError)
			{
				*outError = sceneErr.isEmpty() ? QStringLiteral("OSG B-rep assembly display failed.") : sceneErr;
				return false;
			}
		}
	}
	QHash<QString, QString> pathToBackendId;
	std::shared_ptr<BrepBackendData> lastLoaded;
	int registered = 0;
	for (const BrepHierarchyPart& p : parts)
	{
		if (p.shapeRef.isNull())
		{
			continue;
		}
		auto partBrep = std::make_shared<BrepBackendData>();
		partBrep->setShape(p.shapeRef);
		const QString displayName = p.displayName.empty() ? defaultBaseName : QString::fromStdString(p.displayName);
		partBrep->setName(displayName.toStdString());
		const QString partPath = QString::fromStdString(p.partPath);
		const QString parentPartPath = QString::fromStdString(p.parentPartPath);
		const QString parentId =
			pathToBackendId.contains(parentPartPath) ? pathToBackendId.value(parentPartPath) : importParentId;
		QString regErr;
		if (!registerAdoptedBrepAndLoadScene(host, partBrep, sourceFilePath, QStringLiteral("BrepModel"), parentId,
				false, &regErr, true, false, false))
		{
			if (outError)
			{
				*outError = regErr.isEmpty() ? QStringLiteral("Failed to register hierarchical B-rep part.") : regErr;
			}
			return false;
		}
		if (OsgWidget* osg = osgWidgetFrom(host))
		{
			osg->setPickVisualAlias(partBrep->id(), importParentStdId);
		}
		const QString selfId = QString::fromStdString(partBrep->id());
		(void)onParentFollow;
		pathToBackendId[partPath] = selfId;
		lastLoaded = partBrep;
		++registered;
	}
	if (registered == 0)
	{
		return false;
	}
	out.ok = true;
	out.importParent = importParent;
	out.lastRegisteredBrep = lastLoaded;
	out.registeredPartCount = registered;
	return true;
}

} // namespace

bool importBrepHierarchyParts(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
	const std::vector<BrepHierarchyPart>& parts, const QString& defaultBaseName,
	const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out, QString* outError,
	const QString& importParentDisplayName)
{
	out = {};
	if (!registerBrepHierarchyPartMeshes(host, sourceFilePath, catalogTypeName, defaultBaseName, importParentDisplayName,
			parts, onParentFollow, out, outError))
	{
		if (outError && outError->isEmpty())
		{
			*outError = QStringLiteral("No B-rep parts were registered.");
		}
		return false;
	}
	return true;
}

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
	const int meshImportQuality, const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
	QString* outError)
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
		std::vector<BrepHierarchyPart> brepParts;
		std::string stepErr;
		if (BrepBackendData::loadStepHierarchyFromFile(nativePath, brepParts, &stepErr) && brepParts.size() > 1U)
		{
			if (importBrepHierarchyParts(host, filePath, catalogTypeName, brepParts, defaultBaseName, onParentFollow, out,
					outError, fileInfo.fileName()))
			{
				return true;
			}
			out.ok = false;
			if (outError && outError->isEmpty())
			{
				*outError = QStringLiteral("STEP hierarchy import produced no registrable B-rep parts.");
			}
			return true;
		}
		auto brep = std::make_shared<BrepBackendData>();
		brep->setName(fileInfo.fileName().toStdString());
		if (brep->loadFromStepFile(nativePath, &stepErr) && brep->hasGeometry())
		{
			QString regErr;
			if (!registerAdoptedBrepAndLoadScene(host, brep, filePath, QStringLiteral("BrepModel"), QString(), true,
					&regErr))
			{
				if (outError)
				{
					*outError = regErr.isEmpty() ? QStringLiteral("Failed to register B-rep.") : regErr;
				}
				out.ok = false;
				return true;
			}
			out.ok = true;
			out.lastRegisteredBrep = brep;
			out.registeredPartCount = 1;
			return true;
		}
		if (outError)
		{
			*outError = QString::fromStdString(stepErr.empty() ? std::string("Failed to load STEP as B-rep.") : stepErr);
		}
		out.ok = false;
		return true;
	}

	auto mesh = std::make_shared<MeshBackendData>();
	mesh->setName(fileInfo.fileName().toStdString());
	std::string backendErr;
	const bool cgalOk = mesh->loadFromFile(nativePath, &backendErr, meshImportQuality);
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
