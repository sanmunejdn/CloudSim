/// @file HierarchyMeshImport.cpp
/// @brief 层级网格导入

#include "HierarchyMeshImport.h"

#include "ApplyGeometryImportParse.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendTypeIds.h"
#include "BrepBackendData.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "GeometryFileImporterRegistry.h"
#include "IGeometryFileImporter.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "Types.h"

#include <QByteArray>
#include <QFile>
#include <QFileInfo>
#include <QHash>

#include <BrepImportArtifacts.h>
#include <Discretize.h>
#include <QLatin1String>
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
	// 空壳父：勿挂整装配 Shape，否则选中根节点会 ensureSelectionVisual 把整件再上屏，子件勾选隐藏无效
	(void)assemblyShape;
	if (!registerAdoptedBackendObject(host, importParent, sourceFilePath,
									  QLatin1String(backend_type::kCatalogBrepModel), QString(), outError))
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
		if (!registerAdoptedBrepAndLoadScene(host, partBrep, sourceFilePath,
											 QLatin1String(backend_type::kCatalogBrepModel), parentId, false, &regErr,
											 false, true))
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

bool warmBrepHierarchyPartsDisplayFromAssembly(
	const geoalgo::ShapeHandle& assembly, const std::vector<BrepHierarchyPart>& parts,
	const std::function<void(double progress01, const QString& status)>& progress, QString* outError)
{
	auto report = [&](const double p, const QString& status)
	{
		if (progress)
		{
			progress(p, status);
		}
	};
	if (parts.empty())
	{
		report(1.0, QString());
		return true;
	}

	auto warmOne = [&](const geoalgo::ShapeHandle& shape, const double pEnd) -> bool
	{
		if (shape.isNull())
		{
			return true;
		}
		geoalgo::BrepImportBuildTimings timings;
		std::string artifactErr;
		const auto artifacts = geoalgo::getOrBuildBrepImportArtifacts(shape, &artifactErr, &timings);
		if (!artifacts)
		{
			if (outError)
			{
				*outError = artifactErr.empty() ? QStringLiteral("B-rep tessellation failed.")
												: QString::fromStdString(artifactErr);
			}
			return false;
		}
		(void)timings;
		report(pEnd, QStringLiteral("Meshing B-rep..."));
		return true;
	};

	if (assembly.isNull())
	{
		const std::size_t n = parts.size();
		for (std::size_t i = 0; i < n; ++i)
		{
			const double p1 = 0.4 + 0.5 * (static_cast<double>(i + 1) / static_cast<double>(n));
			report(0.4 + 0.5 * (static_cast<double>(i) / static_cast<double>(n)),
				   QStringLiteral("Meshing B-rep parts..."));
			if (!warmOne(parts[i].shapeRef, p1))
			{
				return false;
			}
		}
		report(1.0, QString());
		return true;
	}

	report(0.4, QStringLiteral("Meshing B-rep assembly..."));
	geoalgo::BrepImportBuildTimings timings;
	std::string artifactErr;
	const auto assemblyArt = geoalgo::getOrBuildBrepImportArtifacts(assembly, &artifactErr, &timings);
	if (!assemblyArt)
	{
		if (outError)
		{
			*outError = artifactErr.empty() ? QStringLiteral("B-rep tessellation failed.")
											: QString::fromStdString(artifactErr);
		}
		return false;
	}
	(void)timings;
	report(0.7, QStringLiteral("Slicing B-rep parts..."));

	const std::size_t n = parts.size();
	for (std::size_t i = 0; i < n; ++i)
	{
		const geoalgo::ShapeHandle& partShape = parts[i].shapeRef;
		if (partShape.isNull())
		{
			continue;
		}
		std::string sliceErr;
		auto sliced = geoalgo::sliceBrepImportArtifactsForShape(assembly, *assemblyArt, partShape, &sliceErr);
		if (sliced)
		{
			geoalgo::putBrepImportArtifacts(partShape, std::move(sliced));
		}
		else if (!warmOne(partShape, 0.7 + 0.25 * (static_cast<double>(i + 1) / static_cast<double>(n))))
		{
			return false;
		}
		report(0.7 + 0.25 * (static_cast<double>(i + 1) / static_cast<double>(n)),
			   QStringLiteral("Slicing B-rep parts..."));
	}
	report(1.0, QString());
	return true;
}

bool importBrepHierarchyParts(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
							  const std::vector<BrepHierarchyPart>& parts, const QString& defaultBaseName,
							  const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
							  QString* outError, const QString& importParentDisplayName,
							  const geoalgo::ShapeHandle& assemblyShape)
{
	out = {};
	// 同步导入无 Worker 预热时在此补一次；异步路径缓存命中几乎无开销
	if (!warmBrepHierarchyPartsDisplayFromAssembly(assemblyShape, parts, {}, outError))
	{
		return false;
	}
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

	const geoalgo::ShapeHandle origShape = brep->shapeRef();
	std::string artErr;
	// 复用装配三角网；拆掉再重剖会把剩余件显示掏空
	if (const auto origArt = geoalgo::getOrBuildBrepImportArtifacts(origShape, &artErr))
	{
		if (auto remainArt = geoalgo::sliceBrepImportArtifactsForShape(origShape, *origArt, remaining, &artErr))
		{
			geoalgo::putBrepImportArtifacts(remaining, std::move(remainArt));
		}
		if (auto solidArt = geoalgo::sliceBrepImportArtifactsForShape(origShape, *origArt, solid, &artErr))
		{
			geoalgo::putBrepImportArtifacts(solid, std::move(solidArt));
		}
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
		QString visErr;
		if (!osg->loadBackendFromBackendData(*brep, &visErr, false, false, true))
		{
			brep->setShape(origShape);
			if (outError)
			{
				*outError = visErr.isEmpty() ? QStringLiteral("剩余件显示重建失败") : visErr;
			}
			return false;
		}
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

	GeometryFileImporterRegistry& registry = GeometryFileImporterRegistry::instance();
	const IGeometryFileImporter* importer = registry.find(ext.toStdString());
	if (!importer)
	{
		if (outError)
		{
			*outError = QStringLiteral("Unsupported mesh import path.");
		}
		out.ok = false;
		return true;
	}

	ImportParseOptions opt;
	opt.meshImportQuality = meshImportQuality;
	ImportParseResult parsed;
	std::string parseErr;
	if (!importer->parse(nativePath, opt, parsed, &parseErr) || !parsed.ok)
	{
		if (outError)
		{
			*outError = QString::fromStdString(parseErr.empty() ? std::string("Import failed.") : parseErr);
		}
		out.ok = false;
		return true;
	}
	return applyGeometryImportParse(host, filePath, catalogTypeName, parsed, onParentFollow, out, outError);
}

} // namespace cloudsim::host
