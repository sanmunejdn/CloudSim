#include "DocumentImportFacade.h"

#include "BackendFileImport.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "HierarchyMeshImport.h"
#include "BrepBackendData.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"

#include <BrepImportArtifacts.h>

#include <QFile>
#include <QFileInfo>

#include <chrono>
#include <RunLogger.h>

namespace cloudsim::host
{

ImportFileResult importFileIntoDocument(DocumentHost& host, const QString& filePath, const ImportFileKind kind,
	const cloudsim::core::ImportOptionsDto& options, QString* outError)
{
	const auto t0 = std::chrono::steady_clock::now();
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
	if (!importMeshFileExtended(host, filePath, options.catalogTypeName, options.quietUi, options.meshImportQuality,
			followBinding, result.hierarchyDetail, &extendedErr))
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
		else if (result.hierarchyDetail.lastRegisteredBrep)
		{
			result.rootBackendId = QString::fromStdString(result.hierarchyDetail.lastRegisteredBrep->id());
			QString homeErr;
			(void)osg->loadBackendFromBackendData(*result.hierarchyDetail.lastRegisteredBrep, &homeErr,
				options.resetViewToHome);
		}
		else if (result.hierarchyDetail.lastRegisteredMesh)
		{
			result.rootBackendId = QString::fromStdString(result.hierarchyDetail.lastRegisteredMesh->id());
			QString homeErr;
			(void)osg->loadMeshFromBackendData(*result.hierarchyDetail.lastRegisteredMesh, &homeErr, options.resetViewToHome);
		}
	}
	const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - t0);
	RunLogger::info("[Import] importFileIntoDocument " + std::to_string(ms.count()) + " ms, file="
		+ filePath.toStdString());
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
			options.resetViewToHome, &regErr, options.linkOsgSceneParent))
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

enum class ModelLoadKind
{
	UseSyncExtended,
	SimpleBrep,
	SimpleMesh,
	BrepHierarchy,
};

void logBrepImportTimings(const geoalgo::BrepImportBuildTimings& timings)
{
	RunLogger::info("[Import] brep mesh_ms=" + std::to_string(timings.meshMs) + " pick_ms="
		+ std::to_string(timings.pickMs) + " tri=" + std::to_string(timings.triangleCount));
}

bool warmBrepImportArtifactsDisplayOnly(
	const geoalgo::ShapeHandle& shape,
	const std::function<void(double progress01, const QString& status)>& report,
	const double progressAfterMesh,
	const double progressEnd,
	QString* outError)
{
	if (shape.isNull())
	{
		report(progressEnd, QString());
		return true;
	}
	geoalgo::BrepImportBuildTimings timings;
	std::string artifactErr;
	const std::shared_ptr<geoalgo::BrepImportArtifacts> artifacts =
		geoalgo::getOrBuildBrepImportArtifacts(shape, &artifactErr, &timings);
	if (!artifacts)
	{
		if (outError)
		{
			*outError = artifactErr.empty() ? QStringLiteral("B-rep tessellation failed.")
											: QString::fromStdString(artifactErr);
		}
		return false;
	}
	report(progressAfterMesh, QStringLiteral("Meshing B-rep..."));
	logBrepImportTimings(timings);
	report(progressEnd, QString());
	return true;
}

bool warmBrepImportPickArtifactsForShape(const geoalgo::ShapeHandle& shape, QString* outError)
{
	if (shape.isNull())
	{
		return true;
	}
	std::string artifactErr;
	const std::shared_ptr<geoalgo::BrepImportArtifacts> artifacts =
		geoalgo::getOrBuildBrepImportArtifacts(shape, &artifactErr);
	if (!artifacts)
	{
		if (outError)
		{
			*outError = artifactErr.empty() ? QStringLiteral("B-rep artifacts missing.")
											: QString::fromStdString(artifactErr);
		}
		return false;
	}
	if (artifacts->pickReady.load(std::memory_order_acquire))
	{
		return true;
	}
	geoalgo::BrepImportBuildTimings timings;
	std::string pickErr;
	if (!geoalgo::buildBrepImportArtifactsPick(shape, *artifacts, &timings, &pickErr))
	{
		RunLogger::warn("[Import] brep pick warm failed: "
			+ (pickErr.empty() ? std::string("unknown") : pickErr));
		if (outError)
		{
			*outError = pickErr.empty() ? QStringLiteral("B-rep pick artifacts failed.")
										: QString::fromStdString(pickErr);
		}
		return false;
	}
	logBrepImportTimings(timings);
	return true;
}

struct ModelBackgroundLoadState::Impl
{
	QString filePath;
	QString displayName;
	QString catalogTypeName;
	int meshImportQuality = 1;
	ModelLoadKind kind = ModelLoadKind::UseSyncExtended;
	std::shared_ptr<BrepBackendData> brep;
	std::shared_ptr<MeshBackendData> mesh;
	std::vector<BrepHierarchyPart> brepHierarchyParts;
};

ModelBackgroundLoadState::ModelBackgroundLoadState(
	const QString& filePath,
	const QString& displayName,
	const QString& catalogTypeName,
	const int meshImportQuality)
	: m_impl(std::make_unique<Impl>())
{
	m_impl->filePath = filePath;
	m_impl->displayName = displayName;
	m_impl->catalogTypeName = catalogTypeName;
	m_impl->meshImportQuality = meshImportQuality;
}

ModelBackgroundLoadState::~ModelBackgroundLoadState() = default;

bool ModelBackgroundLoadState::executeLoad(
	const std::function<void(double progress01, const QString& status)>& progress, QString* outError)
{
	if (!m_impl)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid model load state.");
		}
		return false;
	}
	const QFileInfo fileInfo(m_impl->filePath);
	const QString ext = fileInfo.suffix().toLower();
	const QByteArray nativeEnc = QFile::encodeName(m_impl->filePath);
	const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));

	auto report = [&](const double p, const QString& status) {
		if (progress)
		{
			progress(p, status);
		}
	};

	if (ext == QLatin1String("step") || ext == QLatin1String("stp"))
	{
		report(0.05, QStringLiteral("Reading STEP..."));
		std::vector<BrepHierarchyPart> parts;
		std::string stepErr;
		if (BrepBackendData::loadStepHierarchyFromFile(nativePath, parts, &stepErr) && parts.size() > 1U)
		{
			m_impl->kind = ModelLoadKind::BrepHierarchy;
			m_impl->brepHierarchyParts = std::move(parts);
			if (!warmBrepImportArtifactsDisplayOnly(m_impl->brepHierarchyParts.front().shapeRef, report, 0.25, 1.0, outError))
			{
				return false;
			}
			return true;
		}
		m_impl->brep = std::make_shared<BrepBackendData>();
		m_impl->brep->setName(m_impl->displayName.toStdString());
		report(0.15, QStringLiteral("Reading STEP..."));
		if (!m_impl->brep->loadFromStepFile(nativePath, &stepErr))
		{
			if (outError)
			{
				*outError = stepErr.empty() ? QStringLiteral("Failed to load STEP.") : QString::fromStdString(stepErr);
			}
			return false;
		}
		m_impl->kind = ModelLoadKind::SimpleBrep;
		if (!warmBrepImportArtifactsDisplayOnly(m_impl->brep->shapeRef(), report, 0.35, 1.0, outError))
		{
			return false;
		}
		return true;
	}

	if (ext == QLatin1String("brep"))
	{
		m_impl->brep = std::make_shared<BrepBackendData>();
		m_impl->brep->setName(m_impl->displayName.toStdString());
		report(0.15, QStringLiteral("Reading BREP..."));
		std::string brepErr;
		if (!m_impl->brep->loadFromBrepFile(nativePath, &brepErr))
		{
			if (outError)
			{
				*outError = brepErr.empty() ? QStringLiteral("Failed to load BREP.") : QString::fromStdString(brepErr);
			}
			return false;
		}
		m_impl->kind = ModelLoadKind::SimpleBrep;
		if (!warmBrepImportArtifactsDisplayOnly(m_impl->brep->shapeRef(), report, 0.35, 1.0, outError))
		{
			return false;
		}
		return true;
	}

	static const QStringList kSimpleMesh{
		QStringLiteral("obj"), QStringLiteral("stl"), QStringLiteral("ply"), QStringLiteral("off")};
	if (kSimpleMesh.contains(ext))
	{
		m_impl->mesh = std::make_shared<MeshBackendData>();
		m_impl->mesh->setName(m_impl->displayName.toStdString());
		report(0.1, QStringLiteral("Reading mesh..."));
		std::string meshErr;
		if (!m_impl->mesh->loadFromFile(nativePath, &meshErr, m_impl->meshImportQuality))
		{
			if (outError)
			{
				*outError = meshErr.empty() ? QStringLiteral("Failed to load mesh.") : QString::fromStdString(meshErr);
			}
			return false;
		}
		m_impl->kind = ModelLoadKind::SimpleMesh;
		report(1.0, QString());
		return true;
	}

	m_impl->kind = ModelLoadKind::UseSyncExtended;
	report(1.0, QString());
	return true;
}

ImportFileResult ModelBackgroundLoadState::finishIntoDocument(
	DocumentHost& host,
	const cloudsim::core::ImportOptionsDto& options,
	QString* outError)
{
	if (!m_impl)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid model load state.");
		}
		return {};
	}
	if (m_impl->kind == ModelLoadKind::UseSyncExtended)
	{
		return importFileIntoDocument(host, m_impl->filePath, ImportFileKind::Mesh, options, outError);
	}

	ImportFileResult result;
	const HierarchyFollowBindingFn followBinding;
	const QFileInfo fileInfo(m_impl->filePath);

	if (m_impl->kind == ModelLoadKind::BrepHierarchy)
	{
		if (!importBrepHierarchyParts(host, m_impl->filePath, m_impl->catalogTypeName, m_impl->brepHierarchyParts,
				fileInfo.completeBaseName(), followBinding, result.hierarchyDetail, outError, fileInfo.fileName()))
		{
			if (outError && outError->isEmpty())
			{
				*outError = QStringLiteral("STEP hierarchy import failed.");
			}
			return result;
		}
		result.ok = true;
		result.hierarchyImport = true;
		result.skipFollowOnImport = true;
		if (OsgWidget* osg = osgWidgetFrom(host))
		{
			if (result.hierarchyDetail.importParent)
			{
				result.rootBackendId = QString::fromStdString(result.hierarchyDetail.importParent->id());
				osg->focusCameraOnBackend(result.hierarchyDetail.importParent->id());
				osg->requestRedraw();
			}
		}
		return result;
	}

	if (m_impl->kind == ModelLoadKind::SimpleBrep && m_impl->brep)
	{
		QString regErr;
		if (!registerAdoptedBrepAndLoadScene(host, m_impl->brep, m_impl->filePath, QStringLiteral("BrepModel"),
				QString(), options.resetViewToHome, &regErr))
		{
			if (outError)
			{
				*outError = regErr.isEmpty() ? QStringLiteral("Failed to register B-rep.") : regErr;
			}
			return result;
		}
		result.ok = true;
		result.rootBackendId = QString::fromStdString(m_impl->brep->id());
		return result;
	}

	if (m_impl->kind == ModelLoadKind::SimpleMesh && m_impl->mesh)
	{
		QString regErr;
		if (!registerAdoptedMeshAndLoadScene(host, m_impl->mesh, m_impl->filePath, m_impl->catalogTypeName, QString(),
				options.resetViewToHome, &regErr))
		{
			if (outError)
			{
				*outError = regErr.isEmpty() ? QStringLiteral("Failed to register mesh.") : regErr;
			}
			return result;
		}
		result.ok = true;
		result.rootBackendId = QString::fromStdString(m_impl->mesh->id());
		return result;
	}

	if (outError)
	{
		*outError = QStringLiteral("Model load state has no geometry.");
	}
	return result;
}

bool ModelBackgroundLoadState::needsPickArtifactWarm() const
{
	if (!m_impl)
	{
		return false;
	}
	return m_impl->kind == ModelLoadKind::SimpleBrep || m_impl->kind == ModelLoadKind::BrepHierarchy;
}

bool ModelBackgroundLoadState::warmPickArtifacts(QString* outError)
{
	if (!needsPickArtifactWarm() || !m_impl)
	{
		return true;
	}
	geoalgo::ShapeHandle shape;
	if (m_impl->kind == ModelLoadKind::BrepHierarchy && !m_impl->brepHierarchyParts.empty())
	{
		shape = m_impl->brepHierarchyParts.front().shapeRef;
	}
	else if (m_impl->kind == ModelLoadKind::SimpleBrep && m_impl->brep)
	{
		shape = m_impl->brep->shapeRef();
	}
	return warmBrepImportPickArtifactsForShape(shape, outError);
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
