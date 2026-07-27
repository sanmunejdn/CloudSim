/// @file BackendFileImport.cpp
/// @brief BackendFileImport 实现

#include "BackendFileImport.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendTypeIds.h"
#include "BrepBackendData.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "DocumentHostEvents.h"
#include "FrameBackendData.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PlyIo.h"
#include "PointCloudBackendData.h"

#include <QFile>
#include <QFileInfo>
#include <QLatin1String>
#include <memory>

namespace cloudsim::host
{
namespace
{
std::shared_ptr<PointCloudBackendData> makeEmptyPointCloudShell(const QString& displayName)
{
	auto pointCloud = std::make_shared<PointCloudBackendData>();
	pointCloud->setName(displayName.toStdString());
	BackendColor color;
	color.r = 0.65f;
	color.g = 0.82f;
	color.b = 0.95f;
	color.a = 1.0f;
	pointCloud->setColor(color);
	pointCloud->setWorldMatrix(BackendMat4::identity());
	return pointCloud;
}

bool applyPersistedIdIfRequested(DocumentHost& host, const std::shared_ptr<PointCloudBackendData>& pointCloud,
								 const core::ImportOptionsDto& options, QString* outError)
{
	if (!pointCloud || options.persistedId.isEmpty())
	{
		return pointCloud != nullptr;
	}
	if (host.backend().contains(options.persistedId.toStdString()))
	{
		if (outError)
		{
			*outError = QStringLiteral("persisted id already exists: %1").arg(options.persistedId);
		}
		return false;
	}
	pointCloud->setId(options.persistedId.toStdString());
	return true;
}

core::ObjectId importPointCloudLasLazFile(DocumentHost& host, const QString& filePath,
										  const core::ImportOptionsDto& options, QString* outError)
{
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		if (outError)
		{
			*outError = QStringLiteral("no active 3D view");
		}
		return {};
	}
	const QFileInfo fileInfo(filePath);
	auto pointCloud = makeEmptyPointCloudShell(fileInfo.fileName());
	if (!applyPersistedIdIfRequested(host, pointCloud, options, outError))
	{
		return {};
	}
	QString importErr;
	if (!osg->importPointCloudFile(filePath, &importErr))
	{
		if (outError)
		{
			*outError = importErr.isEmpty() ? QStringLiteral("Failed to load LAS/LAZ.") : importErr;
		}
		return {};
	}
	QString capErr;
	// LAS/LAZ 先解码到 OSG staging，再 capture 进 Data
	if (!osg->captureImportedPointCloudBackend(*pointCloud, &capErr) || pointCloud->pointPositionsXyz().empty())
	{
		osg->clearStagingGeometry();
		if (outError)
		{
			*outError = capErr.isEmpty() ? QStringLiteral("Could not copy LAS/LAZ into backend.")
										 : QStringLiteral("Could not copy LAS/LAZ into backend.\n%1").arg(capErr);
		}
		return {};
	}
	osg->clearStagingGeometry();
	const QString catalog =
		options.catalogTypeName.isEmpty() ? QLatin1String(backend_type::kCatalogPointCloud) : options.catalogTypeName;
	if (!registerAdoptedPointCloudAndLoadScene(host, pointCloud, filePath, catalog, options.resetViewToHome, outError))
	{
		return {};
	}
	return QString::fromStdString(pointCloud->id());
}

} // namespace

core::ObjectId importMeshFile(DocumentHost& host, const QString& filePath, const core::ImportOptionsDto& options,
							  QString* outError)
{
	if (options.isPointCloud)
	{
		if (outError)
		{
			*outError = QStringLiteral("Point cloud import: use registerAdoptedPointCloudAndLoadScene");
		}
		return {};
	}

	const QFileInfo fileInfo(filePath);
	const QString ext = fileInfo.suffix().toLower();
	static const QStringList kHostMeshOnly{
		QStringLiteral("obj"),
		QStringLiteral("stl"),
		QStringLiteral("ply"),
		QStringLiteral("off"),
	};
	if (!kHostMeshOnly.contains(ext))
	{
		if (outError)
		{
			*outError = QStringLiteral("importFromFile: format '%1' requires Widget import path").arg(ext);
		}
		return {};
	}

	const QByteArray nativeEnc = QFile::encodeName(filePath);
	const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));

	auto mesh = std::make_shared<MeshBackendData>();
	mesh->setName(fileInfo.fileName().toStdString());
	mesh->setWorldMatrix(BackendMat4::identity());
	std::string loadErr;
	if (!mesh->loadFromFile(nativePath, &loadErr, options.meshImportQuality) || !mesh->hasGeometry())
	{
		if (outError)
		{
			*outError = loadErr.empty() ? QStringLiteral("Failed to load mesh (no triangle geometry).")
										: QString::fromStdString(loadErr);
		}
		return {};
	}

	if (!host.backend().registerData(mesh))
	{
		if (outError)
		{
			*outError = QStringLiteral("registerData failed");
		}
		return {};
	}

	const QString id = QString::fromStdString(mesh->id());
	host.backendSourcePath()[id] = filePath;
	host.backendSourceType()[id] =
		options.catalogTypeName.isEmpty() ? QLatin1String(backend_type::kCatalogModel) : options.catalogTypeName;
	if (!options.parentId.isEmpty())
	{
		if (!host.backend().attachChild(options.parentId.toStdString(), mesh->id()))
		{
			host.backend().unregisterData(mesh->id());
			if (outError)
			{
				*outError = QStringLiteral("attachChild failed");
			}
			return {};
		}
		host.backendParentId()[id] = options.parentId;
		if (OsgWidget* osg = osgWidgetFrom(host))
		{
			osg->setBackendParent(mesh->id(), options.parentId.toStdString());
		}
	}
	else
	{
		host.backendParentId()[id] = QString();
	}

	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		QString sceneErr;
		if (!osg->loadMeshFromBackendData(*mesh, &sceneErr, options.resetViewToHome))
		{
			host.backend().unregisterData(mesh->id());
			host.backendSourcePath().remove(id);
			host.backendSourceType().remove(id);
			host.backendParentId().remove(id);
			if (outError)
			{
				*outError = sceneErr.isEmpty() ? QStringLiteral("OSG mesh display failed") : sceneErr;
			}
			return {};
		}
	}

	publishBackendObjectRegistered(host, id, QLatin1String(backend_type::kClassModel));
	// 工程恢复：稳定 id 与 JSON 一致
	if (!options.persistedId.isEmpty() && options.persistedId != id)
	{
		const QString rekeyed = rekeyBackendObject(host, id, options.persistedId, outError);
		if (!rekeyed.isEmpty())
		{
			return rekeyed;
		}
	}
	return id;
}

core::ObjectId importPointCloudFile(DocumentHost& host, const QString& filePath, const core::ImportOptionsDto& options,
									QString* outError)
{
	const QFileInfo fileInfo(filePath);
	const QString ext = fileInfo.suffix().toLower();
	if (ext == QStringLiteral("las") || ext == QStringLiteral("laz"))
	{
		return importPointCloudLasLazFile(host, filePath, options, outError);
	}

	const QByteArray nativeEnc = QFile::encodeName(filePath);
	const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));

	if (ext == QStringLiteral("ply") && plyFileHasTriangleFaces(nativePath))
	{
		core::ImportOptionsDto meshOpt = options;
		meshOpt.catalogTypeName = QLatin1String(backend_type::kCatalogModel);
		return importMeshFile(host, filePath, meshOpt, outError);
	}

	auto pointCloud = makeEmptyPointCloudShell(fileInfo.fileName());
	if (!applyPersistedIdIfRequested(host, pointCloud, options, outError))
	{
		return {};
	}

	std::string loadErr;
	if (!pointCloud->loadFromFile(nativePath, &loadErr))
	{
		if (outError)
		{
			*outError =
				loadErr.empty() ? QStringLiteral("Failed to load point cloud.") : QString::fromStdString(loadErr);
		}
		return {};
	}
	pointCloud->setWorldMatrix(BackendMat4::identity());

	const QString catalog =
		options.catalogTypeName.isEmpty() ? QLatin1String(backend_type::kCatalogPointCloud) : options.catalogTypeName;
	if (!registerAdoptedPointCloudAndLoadScene(host, pointCloud, filePath, catalog, options.resetViewToHome, outError))
	{
		return {};
	}
	return QString::fromStdString(pointCloud->id());
}

bool registerAdoptedBackendObject(DocumentHost& host, const std::shared_ptr<BackendDataBase>& object,
								  const QString& sourcePath, const QString& catalogTypeName, const QString& parentId,
								  QString* outError, const bool linkOsgSceneParent)
{
	if (!object)
	{
		if (outError)
		{
			*outError = QStringLiteral("null backend object");
		}
		return false;
	}
	// 工程重载后计数器会重置，ctor 生成的 backend_data_N 可能撞号
	if (object->id().empty() || host.backend().contains(object->id()))
	{
		std::string uniqueId;
		do
		{
			uniqueId = BackendDataBase::generateId();
		} while (host.backend().contains(uniqueId));
		object->setId(uniqueId);
	}
	if (!host.backend().registerData(object))
	{
		if (outError)
		{
			*outError = QStringLiteral("registerData failed");
		}
		return false;
	}
	const QString id = QString::fromStdString(object->id());
	host.backendSourcePath()[id] = sourcePath;
	host.backendSourceType()[id] =
		catalogTypeName.isEmpty() ? QLatin1String(backend_type::kCatalogModel) : catalogTypeName;
	if (!parentId.isEmpty())
	{
		if (!host.backend().attachChild(parentId.toStdString(), object->id()))
		{
			host.backend().unregisterData(object->id());
			if (outError)
			{
				*outError = QStringLiteral("attachChild failed");
			}
			return false;
		}
		host.backendParentId()[id] = parentId;
		if (OsgWidget* osg = osgWidgetFrom(host))
		{
			if (linkOsgSceneParent)
			{
				osg->setBackendParent(object->id(), parentId.toStdString());
			}
			else
			{
				// DXF 分件：仅逻辑父链，OSG 仍在 flat 组
				osg->setBackendLogicalParent(object->id(), parentId.toStdString());
			}
		}
	}
	else
	{
		host.backendParentId()[id] = QString();
	}
	publishBackendObjectRegistered(host, id, QString::fromStdString(object->className()));
	return true;
}

bool registerAdoptedMeshAndLoadScene(DocumentHost& host, const std::shared_ptr<MeshBackendData>& mesh,
									 const QString& sourcePath, const QString& catalogTypeName, const QString& parentId,
									 const bool resetViewToHome, QString* outError, const bool linkOsgSceneParent)
{
	if (!mesh)
	{
		if (outError)
		{
			*outError = QStringLiteral("null mesh");
		}
		return false;
	}
	if (!registerAdoptedBackendObject(host, mesh, sourcePath, catalogTypeName, parentId, outError, linkOsgSceneParent))
	{
		return false;
	}
	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		QString sceneErr;
		if (!osg->loadMeshFromBackendData(*mesh, &sceneErr, resetViewToHome, true, true) && outError)
		{
			*outError = sceneErr.isEmpty() ? QStringLiteral("OSG mesh display failed") : sceneErr;
			return false;
		}
	}
	return true;
}

bool registerAdoptedBrepAndLoadScene(DocumentHost& host, const std::shared_ptr<BrepBackendData>& brep,
									 const QString& sourcePath, const QString& catalogTypeName, const QString& parentId,
									 const bool resetViewToHome, QString* outError, const bool linkOsgSceneParent,
									 const bool loadScene)
{
	if (!brep)
	{
		if (outError)
		{
			*outError = QStringLiteral("null brep");
		}
		return false;
	}
	const QString catalog =
		catalogTypeName.isEmpty() ? QLatin1String(backend_type::kCatalogBrepModel) : catalogTypeName;
	if (!registerAdoptedBackendObject(host, brep, sourcePath, catalog, parentId, outError, linkOsgSceneParent))
	{
		return false;
	}
	if (!loadScene)
	{
		return true;
	}
	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		QString sceneErr;
		if (!osg->loadBackendFromBackendData(*brep, &sceneErr, resetViewToHome, false, true) && outError)
		{
			*outError = sceneErr.isEmpty() ? QStringLiteral("OSG B-rep display failed") : sceneErr;
			return false;
		}
	}
	return true;
}

bool registerAdoptedFrameAndLoadScene(DocumentHost& host, const std::shared_ptr<FrameBackendData>& frame,
									  const QString& catalogTypeName, const QString& parentId,
									  const bool resetViewToHome, QString* outError)
{
	if (!frame)
	{
		if (outError)
		{
			*outError = QStringLiteral("null frame");
		}
		return false;
	}
	const QString catalog =
		catalogTypeName.isEmpty() ? QLatin1String(backend_type::kCatalogCoordinateFrame) : catalogTypeName;
	if (!registerAdoptedBackendObject(host, frame, QString(), catalog, parentId, outError))
	{
		return false;
	}
	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		QString sceneErr;
		if (!osg->loadBackendFromBackendData(*frame, &sceneErr, resetViewToHome, false, false) && outError)
		{
			*outError = sceneErr.isEmpty() ? QStringLiteral("OSG frame display failed") : sceneErr;
			return false;
		}
	}
	return true;
}

bool registerAdoptedPointCloudAndLoadScene(DocumentHost& host, const std::shared_ptr<PointCloudBackendData>& pointCloud,
										   const QString& sourcePath, const QString& catalogTypeName,
										   const bool resetViewToHome, QString* outError)
{
	if (!pointCloud)
	{
		if (outError)
		{
			*outError = QStringLiteral("null point cloud");
		}
		return false;
	}
	if (!registerAdoptedBackendObject(host, pointCloud, sourcePath, catalogTypeName, QString(), outError))
	{
		return false;
	}
	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		QString sceneErr;
		if (!osg->loadPointCloudFromBackendData(*pointCloud, &sceneErr, resetViewToHome) && outError)
		{
			*outError = sceneErr.isEmpty() ? QStringLiteral("OSG point cloud display failed") : sceneErr;
			return false;
		}
	}
	return true;
}

QString rekeyBackendObject(DocumentHost& host, const QString& fromId, const QString& toId, QString* outError)
{
	if (fromId.isEmpty())
	{
		return {};
	}
	if (toId.isEmpty() || fromId == toId)
	{
		return fromId;
	}
	if (host.backend().contains(toId.toStdString()))
	{
		if (outError)
		{
			*outError = QStringLiteral("rekey target id already exists: %1").arg(toId);
		}
		return fromId;
	}
	std::shared_ptr<BackendDataBase> obj = host.backend().getData(fromId.toStdString());
	if (!obj)
	{
		if (outError)
		{
			*outError = QStringLiteral("rekey source not found: %1").arg(fromId);
		}
		return fromId;
	}
	const std::vector<std::string> parents = host.backend().parentsOf(fromId.toStdString());
	const QString catalogType = host.backendSourceType().value(fromId);
	const QString sourcePath = host.backendSourcePath().value(fromId);
	const QString parentMirror = host.backendParentId().value(fromId);

	// Data 不支持原地改 id，须摘链再注册并同步 OSG/旁路表
	publishBackendObjectRemoved(host, fromId);
	host.backend().unregisterData(fromId.toStdString());
	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		osg->removeBackendObjectVisual(fromId.toStdString());
	}

	obj->setId(toId.toStdString());
	if (!host.backend().registerData(obj))
	{
		obj->setId(fromId.toStdString());
		if (!host.backend().registerData(obj))
		{
			if (outError)
			{
				*outError = QStringLiteral("rekey failed and rollback failed");
			}
			return {};
		}
		if (OsgWidget* osg = osgWidgetFrom(host))
		{
			QString sceneErr;
			if (const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(obj))
			{
				(void)osg->loadMeshFromBackendData(*mesh, &sceneErr, false);
			}
			else if (const auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(obj))
			{
				(void)osg->loadPointCloudFromBackendData(*pc, &sceneErr, false);
			}
		}
		publishBackendObjectRegistered(host, fromId, QString::fromStdString(obj->className()));
		if (outError)
		{
			*outError = QStringLiteral("rekey registerData failed");
		}
		return fromId;
	}
	for (const std::string& parentId : parents)
	{
		(void)host.backend().attachChild(parentId, obj->id());
		if (OsgWidget* osg = osgWidgetFrom(host))
		{
			osg->setBackendParent(obj->id(), parentId);
		}
	}
	host.backendSourcePath().remove(fromId);
	host.backendSourceType().remove(fromId);
	host.backendParentId().remove(fromId);
	if (!sourcePath.isEmpty())
	{
		host.backendSourcePath()[toId] = sourcePath;
	}
	if (!catalogType.isEmpty())
	{
		host.backendSourceType()[toId] = catalogType;
	}
	if (!parentMirror.isEmpty())
	{
		host.backendParentId()[toId] = parentMirror;
	}
	if (OsgWidget* osg = osgWidgetFrom(host))
	{
		QString sceneErr;
		if (const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(obj))
		{
			(void)osg->loadMeshFromBackendData(*mesh, &sceneErr, false);
		}
		else if (const auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(obj))
		{
			(void)osg->loadPointCloudFromBackendData(*pc, &sceneErr, false);
		}
	}
	publishBackendObjectRegistered(host, toId, QString::fromStdString(obj->className()));
	return toId;
}

} // namespace cloudsim::host
