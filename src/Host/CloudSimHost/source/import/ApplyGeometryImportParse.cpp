/// @file ApplyGeometryImportParse.cpp
/// @brief ImportParseResult → DocumentHost 注册

#include "ApplyGeometryImportParse.h"

#include "BackendFileImport.h"
#include "BackendTypeIds.h"
#include "BrepBackendData.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "OsgWidgetCaptureController.h"

#include <QFileInfo>
#include <QHash>
#include <QLatin1String>

namespace cloudsim::host
{
namespace
{
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

bool applyOsgCapture(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
					 const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out, QString* outError)
{
	const QFileInfo fileInfo(sourceFilePath);
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		if (outError)
		{
			*outError = QStringLiteral("OSG formats (dae/3ds/fbx) require desktop 3D view.");
		}
		out.ok = false;
		return true;
	}
	QString importErr;
	if (!osg->importModelFile(sourceFilePath, &importErr))
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
		if (registerCapturedHierarchyParts(host, sourceFilePath, catalogTypeName, fileInfo.fileName(), parts,
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
	auto mesh = std::make_shared<MeshBackendData>();
	mesh->setName(fileInfo.fileName().toStdString());
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
	if (!registerAdoptedMeshAndLoadScene(host, mesh, sourceFilePath, catalogTypeName, QString(), true, &regErr))
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
} // namespace

bool applyGeometryImportParse(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
							  ImportParseResult& parsed, const HierarchyFollowBindingFn& onParentFollow,
							  HierarchyMeshImportResult& out, QString* outError)
{
	out = {};
	if (!parsed.ok)
	{
		if (outError && outError->isEmpty())
		{
			*outError = QStringLiteral("Import parse failed.");
		}
		return false;
	}

	const QFileInfo fileInfo(sourceFilePath);
	const QString defaultBaseName = fileInfo.completeBaseName();
	const QString displayParent =
		parsed.displayNameHint.empty() ? fileInfo.fileName() : QString::fromStdString(parsed.displayNameHint);

	switch (parsed.kind)
	{
	case ImportParseKind::MeshHierarchy:
		if (!importMeshHierarchyParts(host, sourceFilePath, catalogTypeName, parsed.meshParts, defaultBaseName,
									  onParentFollow, out, outError, displayParent))
		{
			if (outError && outError->isEmpty())
			{
				*outError = QStringLiteral("Hierarchy import produced no registrable mesh parts.");
			}
			out.ok = false;
			return true;
		}
		return true;

	case ImportParseKind::MeshSingleSoup:
	{
		if (parsed.meshSoup.empty())
		{
			if (outError)
			{
				*outError = QStringLiteral("Failed to load mesh.");
			}
			out.ok = false;
			return true;
		}
		auto mesh = std::make_shared<MeshBackendData>();
		mesh->setName(displayParent.toStdString());
		mesh->setTriangleSoup(std::move(parsed.meshSoup));
		QString regErr;
		if (!registerAdoptedMeshAndLoadScene(host, mesh, sourceFilePath, catalogTypeName, QString(), true, &regErr))
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

	case ImportParseKind::BrepHierarchy:
		if (!importBrepHierarchyParts(host, sourceFilePath, catalogTypeName, parsed.brepParts, defaultBaseName,
									  onParentFollow, out, outError, displayParent, parsed.brepAssembly))
		{
			if (outError && outError->isEmpty())
			{
				*outError = QStringLiteral("STEP hierarchy import produced no registrable B-rep parts.");
			}
			out.ok = false;
			return true;
		}
		return true;

	case ImportParseKind::BrepSingle:
	{
		if (parsed.brepSingle.isNull())
		{
			if (outError)
			{
				*outError = QStringLiteral("Failed to load STEP as B-rep.");
			}
			out.ok = false;
			return true;
		}
		auto brep = std::make_shared<BrepBackendData>();
		brep->setName(displayParent.toStdString());
		brep->setShape(parsed.brepSingle);
		QString regErr;
		if (!registerAdoptedBrepAndLoadScene(host, brep, sourceFilePath, QLatin1String(backend_type::kCatalogBrepModel),
											 QString(), true, &regErr))
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

	case ImportParseKind::OsgCapture:
		return applyOsgCapture(host, sourceFilePath, catalogTypeName, onParentFollow, out, outError);
	}

	if (outError)
	{
		*outError = QStringLiteral("Unsupported mesh import path.");
	}
	return false;
}

} // namespace cloudsim::host
