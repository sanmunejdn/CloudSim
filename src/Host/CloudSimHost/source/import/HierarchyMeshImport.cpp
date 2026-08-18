/// @file HierarchyMeshImport.cpp
/// @brief 层级网格导入

#include "HierarchyMeshImport.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendTypeIds.h"
#include "BrepBackendData.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "OsgWidgetCaptureController.h"
#include "Types.h"

#include <QByteArray>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QLatin1String>

#include <Discretize.h>
#include <ShapeHandle.h>
#include <ShapeQuery.h>

namespace cloudsim::host
{
namespace
{
bool registerHierarchyPartMeshes(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
								 const QString& defaultBaseName, const QString& importParentDisplayName,
								 const std::vector<MeshHierarchyPart>& parts,
								 const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
								 QString* outError)
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
		if (!registerAdoptedMeshAndLoadScene(host, partMesh, sourceFilePath, catalogTypeName, parentId, false,
											 &meshRegErr, false))
		{
			if (outError)
			{
				*outError =
					meshRegErr.isEmpty() ? QStringLiteral("Failed to register hierarchical mesh part.") : meshRegErr;
			}
			return false;
		}
		const QString selfId = QString::fromStdString(partMesh->id());
		// 世界坐标分件：skipInnerModelCenterRebase + 无 Follow，避免 pose≈-质心
		(void)onParentFollow;
		pathToBackendId[partPath] = selfId;
		lastLoadedMesh = partMesh;
		out.partBackendIds.append(selfId);
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
									const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
									QString* outError)
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
		if (!registerAdoptedMeshAndLoadScene(host, partMesh, sourceFilePath, catalogTypeName, parentId, false,
											 &meshRegErr, false))
		{
			if (outError)
			{
				*outError = meshRegErr.isEmpty() ? QStringLiteral("Failed to register hierarchical backend object.")
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
									 const QString& defaultBaseName, const QString& importParentDisplayName,
									 const std::vector<BrepHierarchyPart>& parts,
									 const HierarchyFollowBindingFn& onParentFollow,
									 const geoalgo::ShapeHandle& assemblyShape, HierarchyMeshImportResult& out,
									 QString* outError)
{
	if (parts.empty())
	{
		return false;
	}
	auto importParent = std::make_shared<BrepBackendData>();
	const QString parentLabel = importParentDisplayName.isEmpty() ? defaultBaseName : importParentDisplayName;
	importParent->setName(parentLabel.toStdString());
	if (!assemblyShape.isNull())
	{
		importParent->setShape(assemblyShape);
	}
	if (!registerAdoptedBackendObject(host, importParent, sourceFilePath, QLatin1String(backend_type::kCatalogBrepModel), QString(),
									  outError))
	{
		return false;
	}
	const QString importParentId = QString::fromStdString(importParent->id());
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
		if (!registerAdoptedBrepAndLoadScene(host, partBrep, sourceFilePath, QLatin1String(backend_type::kCatalogBrepModel), parentId,
											 false, &regErr, false, true))
		{
			if (outError)
			{
				*outError = regErr.isEmpty() ? QStringLiteral("Failed to register hierarchical B-rep part.") : regErr;
			}
			return false;
		}
		const QString selfId = QString::fromStdString(partBrep->id());
		(void)onParentFollow;
		pathToBackendId[partPath] = selfId;
		lastLoaded = partBrep;
		out.partBackendIds.append(selfId);
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
							  const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
							  QString* outError, const QString& importParentDisplayName,
							  const geoalgo::ShapeHandle& assemblyShape)
{
	out = {};
	if (!registerBrepHierarchyPartMeshes(host, sourceFilePath, catalogTypeName, defaultBaseName,
										 importParentDisplayName, parts, onParentFollow, assemblyShape, out, outError))
	{
		if (outError && outError->isEmpty())
		{
			*outError = QStringLiteral("No B-rep parts were registered.");
		}
		return false;
	}
	return true;
}

bool extractBrepSolidByFace(DocumentHost& host, const std::string& brepId, const int faceIndex, QString* outPartId,
							QString* outError)
{
	if (outPartId)
	{
		outPartId->clear();
	}
	const auto brep = std::dynamic_pointer_cast<BrepBackendData>(host.findObject(brepId));
	if (!brep || brep->shapeRef().isNull())
	{
		if (outError)
		{
			*outError = QStringLiteral("Not a B-rep object.");
		}
		return false;
	}

	geoalgo::ShapeHandle solid;
	geoalgo::ShapeHandle remaining;
	std::string extractErr;
	if (!geoalgo::extractSolidByFaceIndex(brep->shapeRef(), faceIndex, solid, remaining, &extractErr) || solid.isNull())
	{
		if (outError)
		{
			*outError = extractErr.empty() ? QStringLiteral("Failed to resolve Solid from face.")
										   : QString::fromStdString(extractErr);
		}
		return false;
	}
	if (remaining.isNull())
	{
		if (outPartId)
		{
			*outPartId = QString::fromStdString(brepId);
		}
		return true;
	}

	osg::Matrixd world;
	bool haveWorld = false;
	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		haveWorld = osg->getBackendRootWorldMatrix(brepId, world);
	}
	const BackendMat4 parentWorld = brep->worldMatrix();
	brep->setShape(remaining);
	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		osg->removeBackendObjectVisual(brepId);
		QString visErr;
		(void)osg->loadBackendFromBackendData(*brep, &visErr, false, false, true);
	}

	auto partBrep = std::make_shared<BrepBackendData>();
	partBrep->setShape(solid);
	partBrep->setWorldMatrix(parentWorld);
	partBrep->setColor(brep->color());
	const QString partName = QStringLiteral("%1 Solid").arg(QString::fromStdString(brep->name()));
	partBrep->setName(partName.toStdString());
	const QString sourcePath = host.backendSourcePath().value(QString::fromStdString(brepId));
	QString regErr;
	if (!registerAdoptedBrepAndLoadScene(host, partBrep, sourcePath, QLatin1String(backend_type::kCatalogBrepModel),
										 QString::fromStdString(brepId), false, &regErr, false, true))
	{
		if (outError)
		{
			*outError = regErr.isEmpty() ? QStringLiteral("Failed to register extracted Solid.") : regErr;
		}
		return false;
	}
	if (haveWorld)
	{
		if (OsgWidget* osg = osgWidgetFrom(host))
		{
			osg->setBackendRootWorldMatrixFromWorld(partBrep->id(), world);
		}
	}
	if (outPartId)
	{
		*outPartId = QString::fromStdString(partBrep->id());
	}
	return true;
}

bool importMeshHierarchyParts(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
							  const std::vector<MeshHierarchyPart>& parts, const QString& defaultBaseName,
							  const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
							  QString* outError, const QString& importParentDisplayName)
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

bool importMeshFileExtended(DocumentHost& host, const QString& filePath, const QString& catalogTypeName,
							const bool quietUi, const int meshImportQuality,
							const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
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
			if (importMeshHierarchyParts(host, filePath, catalogTypeName, dxfParts, defaultBaseName, onParentFollow,
										 out, outError, fileInfo.fileName()))
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
		// 导入保持整件；拆件改由视口点面抽出
		std::string stepErr;
		auto brep = std::make_shared<BrepBackendData>();
		brep->setName(fileInfo.fileName().toStdString());
		if (brep->loadFromStepFile(nativePath, &stepErr) && brep->hasGeometry())
		{
			QString regErr;
			if (!registerAdoptedBrepAndLoadScene(host, brep, filePath, QLatin1String(backend_type::kCatalogBrepModel), QString(), true,
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
			*outError =
				QString::fromStdString(stepErr.empty() ? std::string("Failed to load STEP as B-rep.") : stepErr);
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
			QStringLiteral("dae"),
			QStringLiteral("3ds"),
			QStringLiteral("fbx"),
		};
		OsgWidget* osg = osgWidgetFrom(host);
		if (!kOsgOnly.contains(ext) || !osg)
		{
			if (outError)
			{
				*outError =
					QString::fromStdString(backendErr.empty() ? std::string("Failed to load mesh.") : backendErr);
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
			if (registerCapturedHierarchyParts(host, filePath, catalogTypeName, fileInfo.fileName(), parts,
											   onParentFollow, out, outError))
			{
				osg->clearStagingGeometry();
				return true;
			}
			if (outError && outError->isEmpty())
			{
				*outError = hierarchyErr.isEmpty() ? QStringLiteral("Failed to register hierarchical model parts.")
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
