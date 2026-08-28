/// @file BackendProjectObjectIo.cpp
/// @brief 工程对象 JSON IO

#include "BackendProjectObjectIo.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendFollowSolve.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"
#include "BackendTypeIdentity.h"
#include "BrepBackendData.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceRobotMountComponent.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "FollowAttachmentComponent.h"
#include "FrameBackendData.h"
#include "IDataService.h"
#include "io/CustomDeviceHostOps.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "ParametricBrepBackendData.h"
#include "PointCloudBackendData.h"
#include "RobotProjectKinematicsRestore.h"
#include "RunLogger.h"
#include "ViewTessellate.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonValue>
#include <QMap>

#include <json.hpp>
#include <osg/Vec3f>

namespace cloudsim::host
{
namespace
{
QString resolveProjectObjectLoadPath(const QString& projectDir, const QString& sourcePath,
									 const QString& assetRelativePath)
{
	if (!sourcePath.isEmpty() && QFileInfo::exists(sourcePath))
	{
		return sourcePath;
	}
	if (!assetRelativePath.isEmpty())
	{
		const QString bundled = QDir(projectDir).filePath(assetRelativePath);
		if (QFileInfo::exists(bundled))
		{
			return QDir::cleanPath(bundled);
		}
	}
	return {};
}

void appendProjectLoadWarning(QStringList* outWarnings, const QString& message)
{
	if (outWarnings)
	{
		outWarnings->append(message);
	}
}

// 加载工程时用于共享 stepSidecar 对应 Shape 的缓存
QMap<QString, std::shared_ptr<BrepBackendData>> g_stepSidecarCache;

// id 软引用完整性校验：DAG 只管层级边，对象内的 backendId 引用（follow.targetId、
// CustomDeviceLink::geometryBackendId、RobotMount 三个 id、motionCenterFrameBackendId、
// upToFaceBackendId）删除被引用对象后会留下悬空 id。加载收尾时全量对账一次并告警，
// 不再静默让求解器跳过
void collectDanglingBackendRefs(DocumentHost& host, QStringList* outWarnings)
{
	const auto all = host.backend().listData();
	for (const auto& d : all)
	{
		if (!d)
		{
			continue;
		}
		auto checkRef = [&](const std::string& refId, const char* fieldDesc)
		{
			if (!refId.empty() && !host.backend().contains(refId))
			{
				appendProjectLoadWarning(
					outWarnings,
					QStringLiteral("Dangling reference: %1 (id=%2) field %3 points to missing backend \"%4\"")
						.arg(QString::fromStdString(d->name()), QString::fromStdString(d->id()),
							 QString::fromLatin1(fieldDesc), QString::fromStdString(refId)));
			}
		};
		if (const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
				d->getComponent(FollowAttachmentComponent::typeKeyStatic())))
		{
			checkRef(follow->targetBackendId(), "follow.targetId");
		}
		if (const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(d))
		{
			for (const auto& link : device->links())
			{
				checkRef(link.geometryBackendId, "link.geometryBackendId");
			}
			for (const auto& axis : device->axes().axes)
			{
				checkRef(axis.motionCenterFrameBackendId, "axis.motionCenterFrameBackendId");
			}
			if (const auto mount = std::dynamic_pointer_cast<CustomDeviceRobotMountComponent>(
					device->getComponent(CustomDeviceRobotMountComponent::typeKeyStatic())))
			{
				checkRef(mount->robotSceneBackendId(), "mount.robotSceneBackendId");
				checkRef(mount->mountFrameBackendId(), "mount.mountFrameBackendId");
				checkRef(mount->flangeBackendId(), "mount.flangeBackendId");
			}
		}
		if (const auto param = std::dynamic_pointer_cast<ParametricBrepBackendData>(d))
		{
			for (const auto& f : param->features())
			{
				checkRef(f.upToFaceBackendId, "feature.upToFaceBackendId");
			}
		}
	}
}

// RobotURDF_<型号> 为根；RobotURDF_<型号>_<link> 为连杆（型号段常含 '-' 而非 '_'）
bool looksLikeUrdfLinkMeshBackendId(const QString& persistedId)
{
	if (!persistedId.startsWith(QStringLiteral("RobotURDF_")))
	{
		return false;
	}
	return persistedId.mid(10).contains(QLatin1Char('_'));
}

bool isUrdfRobotSceneRootShellCandidate(const QString& persistedId, const ProjectObjectLoadOptions& options)
{
	if (!persistedId.startsWith(QStringLiteral("RobotURDF_")))
	{
		return false;
	}
	if (options.robotLinkMeshBackendIds.contains(persistedId))
	{
		return false;
	}
	if (!options.robotSceneRootBackendIds.isEmpty())
	{
		return options.robotSceneRootBackendIds.contains(persistedId);
	}
	return !looksLikeUrdfLinkMeshBackendId(persistedId);
}

} // namespace

QJsonObject saveProjectObject(DocumentHost& host, const QString& objectId, const QString& sourcePath,
							  const QString& sourceType, const QString& parentId)
{
	QJsonObject obj = host.data().saveObjectToJson(objectId);
	if (obj.isEmpty())
	{
		return {};
	}
	obj.insert(QStringLiteral("sourcePath"), sourcePath);
	obj.insert(QStringLiteral("sourceType"), sourceType);
	obj.insert(QStringLiteral("parentId"), parentId);
	return obj;
}

bool decodeBackendObjectFromProjectJson(const QJsonObject& objectJson, std::shared_ptr<BackendDataBase>& out,
										QString* outError)
{
	out.reset();
	ensureBackendBuiltinsRegistered();
	const QString className = objectJson.value(QStringLiteral("className")).toString();
	if (className.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("className is empty");
		}
		return false;
	}
	auto obj = BackendRegistry::instance().create(className.toStdString());
	if (!obj)
	{
		if (outError)
		{
			*outError = QStringLiteral("unknown className: %1").arg(className);
		}
		return false;
	}
	const QJsonDocument doc(objectJson);
	const std::string jsonStd = doc.toJson(QJsonDocument::Compact).toStdString();
	nlohmann::json j = nlohmann::json::parse(jsonStd, nullptr, false);
	if (j.is_discarded())
	{
		if (outError)
		{
			*outError = QStringLiteral("JSON parse failed");
		}
		return false;
	}
	std::string loadErr;
	if (!obj->loadFromJson(j, &loadErr))
	{
		if (outError)
		{
			*outError = loadErr.empty() ? QStringLiteral("loadFromJson failed") : QString::fromStdString(loadErr);
		}
		return false;
	}
	out = std::move(obj);
	return true;
}

bool registerEmbeddedProjectObject(DocumentHost& host, const QJsonObject& objectJson, const QString& persistedId,
								   const QString& sourcePath, const QString& catalogTypeName, const QString& parentId,
								   const bool robotLinkMeshVisual, const QString& projectDir,
								   const RobotLinkUrdfReloadHint* robotLinkReloadHint, QString* outVisualError,
								   QString* outError)
{
	std::shared_ptr<BackendDataBase> backendObject;
	if (!decodeBackendObjectFromProjectJson(objectJson, backendObject, outError))
	{
		return false;
	}
	if (!persistedId.isEmpty())
	{
		backendObject->setId(persistedId.toStdString());
	}
	if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(backendObject))
	{
		if (!mesh->hasGeometry() && robotLinkReloadHint)
		{
			QString reloadErr;
			if (!reloadRobotLinkMeshFromUrdfHint(*mesh, *robotLinkReloadHint, &reloadErr) && outError && !reloadErr.isEmpty())
			{
				*outError = reloadErr;
			}
		}
	}
	if (auto brep = std::dynamic_pointer_cast<BrepBackendData>(backendObject))
	{
		if (!brep->hasGeometry())
		{
			const QJsonObject emb = objectJson.value(QStringLiteral("geometry")).toObject();

			// 优先尝试 stepSidecar（新保存格式，原始 STEP 拷贝），并支持多个 BREP 共享同一个 Shape
			QString stepRel = emb.value(QStringLiteral("stepSidecar")).toString();
			bool attemptedSidecar = false;
			if (!stepRel.isEmpty())
			{
				attemptedSidecar = true;
				if (g_stepSidecarCache.contains(stepRel))
				{
					// 直接共享已加载的 Shape
					brep->shareShapeFrom(*g_stepSidecarCache.value(stepRel));
					brep->setBrepSidecarRelativePath(stepRel.toStdString());
				}
				else
				{
					const QString stepPath = resolveProjectObjectLoadPath(projectDir, sourcePath, stepRel);
					if (!stepPath.isEmpty())
					{
						const QByteArray enc = QFile::encodeName(stepPath);
						const std::string nativePath(enc.constData(), static_cast<std::size_t>(enc.size()));
						std::string loadErr;
						if (!brep->loadFromStepFile(nativePath, &loadErr))
						{
							if (outError)
							{
								*outError = loadErr.empty() ? QStringLiteral("Failed to load STEP sidecar")
															: QString::fromStdString(loadErr);
							}
							return false;
						}
						brep->setBrepSidecarRelativePath(stepRel.toStdString());
						g_stepSidecarCache.insert(stepRel, brep); // 缓存以供后续对象共享
					}
				}
			}
			else
			{
			// 回退到旧的 brepSidecar 格式
			QString brepRel = emb.value(QStringLiteral("brepSidecar")).toString();
			if (brepRel.isEmpty())
			{
				brepRel = objectJson.value(QStringLiteral("assetRelativePath")).toString();
			}
			const QString brepPath = resolveProjectObjectLoadPath(projectDir, sourcePath, brepRel);
			if (!brepPath.isEmpty())
			{
				attemptedSidecar = true;
				const QByteArray enc = QFile::encodeName(brepPath);
				const std::string nativePath(enc.constData(), static_cast<std::size_t>(enc.size()));
				std::string loadErr;
				if (!brep->loadFromBrepFile(nativePath, &loadErr))
				{
					if (outError)
					{
						*outError = loadErr.empty() ? QStringLiteral("Failed to load B-rep sidecar")
													: QString::fromStdString(loadErr);
					}
					return false;
				}
				if (!brepRel.isEmpty())
				{
					brep->setBrepSidecarRelativePath(brepRel.toStdString());
				}
			}
			// sidecar 路径存在但文件实际缺失/加载失败后仍空：告警提示工程文件不完整
			if (!brep->hasGeometry() && attemptedSidecar)
			{
				RunLogger::warn("[ProjectIo] brep \"" + brep->id() +
								"\" sidecar referenced but geometry still empty after load attempt. "
								"Sidecar file may be missing or renamed.");
			}
		}
	}
	}
	if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(backendObject))
	{
		if (pc->pointPositionsXyz().empty())
		{
			const QJsonObject emb = objectJson.value(QStringLiteral("geometry")).toObject();
			QString plyRel = emb.value(QStringLiteral("plySidecar")).toString();
			if (plyRel.isEmpty())
			{
				plyRel = objectJson.value(QStringLiteral("assetRelativePath")).toString();
			}
			const QString plyPath = resolveProjectObjectLoadPath(projectDir, sourcePath, plyRel);
			if (!plyPath.isEmpty())
			{
				const QByteArray enc = QFile::encodeName(plyPath);
				const std::string nativePath(enc.constData(), static_cast<std::size_t>(enc.size()));
				std::string loadErr;
				if (!pc->readPointCloudPlySidecar(nativePath, &loadErr))
				{
					if (outError)
					{
						*outError = loadErr.empty() ? QStringLiteral("Failed to load point cloud PLY sidecar")
													: QString::fromStdString(loadErr);
					}
					return false;
				}
			}
		}
		// sidecar 对账：元数据声明点数与实际加载不一致即告警（sidecar 丢失/改名后不再静默空对象）
		const QJsonObject geomJson = objectJson.value(QStringLiteral("geometry")).toObject();
		const qint64 declared = static_cast<qint64>(geomJson.value(QStringLiteral("pointCount")).toDouble(0.0));
		if (declared > 0 && static_cast<qint64>(pc->geometryElementCount()) != declared)
		{
			RunLogger::warn("[ProjectIo] point cloud \"" + pc->id() + "\" declares pointCount=" +
							std::to_string(declared) + " but loaded " +
							std::to_string(pc->geometryElementCount()) +
							". PLY sidecar may be missing or renamed.");
		}
	}
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		// Web/Headless：无 OSG 时仍注册 Data，浏览器经 /api/mesh 取 triangleSoup
		if (const auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(backendObject))
		{
			if (pc->pointPositionsXyz().empty())
			{
				if (outError)
				{
					*outError = QStringLiteral("Point cloud has no geometry (missing PLY sidecar and embedded data).");
				}
				return false;
			}
		}
		(void)robotLinkMeshVisual;
		return registerAdoptedBackendObject(host, backendObject, sourcePath, catalogTypeName, parentId, outError);
	}
	QString visualErr;
	bool visualOk = false;
	if (const auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(backendObject))
	{
		if (pc->pointPositionsXyz().empty())
		{
			if (outError)
			{
				*outError = QStringLiteral("Point cloud has no geometry (missing PLY sidecar and embedded data).");
			}
			return false;
		}
		visualOk = osg->loadPointCloudFromBackendData(*pc, &visualErr, true);
	}
	else if (const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(backendObject))
	{
		(void)robotLinkMeshVisual;
		visualOk = osg->loadMeshFromBackendData(*mesh, &visualErr, true, true, true);
	}
	else if (const auto brep = std::dynamic_pointer_cast<BrepBackendData>(backendObject))
	{
		visualOk = osg->loadBackendFromBackendData(*brep, &visualErr, true, true, true);
	}
	else if (const auto frame = std::dynamic_pointer_cast<FrameBackendData>(backendObject))
	{
		visualOk = osg->loadBackendFromBackendData(*frame, &visualErr, true, false, false);
	}
	else if (const auto customDevice = std::dynamic_pointer_cast<CustomDeviceBackendData>(backendObject))
	{
		visualOk = osg->loadBackendFromBackendData(*customDevice, &visualErr, true, false, false);
	}
	else
	{
		if (outError)
		{
			*outError =
				QStringLiteral("unsupported backend class: %1").arg(QString::fromStdString(backendObject->className()));
		}
		return false;
	}
	if (!visualOk)
	{
		if (outVisualError)
		{
			*outVisualError = visualErr;
		}
		return false;
	}
	// 视觉默认显示；按 Data 真源恢复隐藏态
	osg->setBackendObjectVisible(backendObject->id(), backendObject->isVisible());
	return registerAdoptedBackendObject(host, backendObject, sourcePath, catalogTypeName, parentId, outError);
}

QString importProjectObjectFromFile(DocumentHost& host, const QString& loadPath, const QString& persistedId,
									const QString& catalogTypeName, const bool isPointCloud, QString* outError)
{
	core::ImportOptionsDto opt;
	opt.quietUi = true;
	opt.resetViewToHome = false;
	opt.persistedId = persistedId;
	opt.catalogTypeName = catalogTypeName.isEmpty()
							  ? (isPointCloud ? QLatin1String(backend_type::kCatalogPointCloud)
											  : QLatin1String(backend_type::kCatalogModel))
							  : catalogTypeName;
	if (isPointCloud)
	{
		return importPointCloudFile(host, loadPath, opt, outError);
	}
	return importMeshFile(host, loadPath, opt, outError);
}

QVector<ProjectHierarchyEdge> parseProjectEdgesJson(const QJsonArray& edgesJson)
{
	QVector<ProjectHierarchyEdge> edges;
	edges.reserve(edgesJson.size());
	for (const QJsonValue& v : edgesJson)
	{
		if (!v.isObject())
		{
			continue;
		}
		const QJsonObject edgeObj = v.toObject();
		const QString parentId = edgeObj.value(QStringLiteral("parentId")).toString();
		const QString childId = edgeObj.value(QStringLiteral("childId")).toString();
		if (parentId.isEmpty() || childId.isEmpty() || parentId == childId)
		{
			continue;
		}
		edges.push_back(ProjectHierarchyEdge{parentId, childId});
	}
	return edges;
}

void applyProjectEdgesToBackend(DocumentHost& host, const QVector<ProjectHierarchyEdge>& edges,
								QStringList* outWarnings)
{
	for (const ProjectHierarchyEdge& edge : edges)
	{
		const std::string parentId = edge.parentId.toStdString();
		const std::string childId = edge.childId.toStdString();
		if (!host.backend().contains(parentId) || !host.backend().contains(childId))
		{
			if (outWarnings)
			{
				outWarnings->append(QStringLiteral("Skip dangling edge: %1 -> %2").arg(edge.parentId, edge.childId));
			}
			continue;
		}
		if (!host.backend().attachChild(parentId, childId) && outWarnings)
		{
			outWarnings->append(
				QStringLiteral("Skip invalid edge (cycle or duplicate): %1 -> %2").arg(edge.parentId, edge.childId));
		}
	}
}

void syncOsgBackendParentsFromBackend(DocumentHost& host)
{
	// 工程 edges 只写 Data 时，须把 OSG 场景父链与 topo 对齐
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		return;
	}
	// 父先于子，避免一轮 sync 里子节点因父尚未入库而挂不上
	for (const std::string& id : host.backend().topoOrder())
	{
		const auto data = host.backend().getData(id);
		if (!data)
		{
			continue;
		}
		const std::vector<std::string> parents = host.backend().parentsOf(id);
		const std::string parent = parents.empty() ? std::string() : parents.front();
		osg->setBackendParent(id, parent);
	}
}

void rebuildBackendParentIdMirror(DocumentHost& host)
{
	QMap<QString, QString>& parentMap = host.backendParentId();
	parentMap.clear();
	for (const auto& data : host.backend().listData())
	{
		if (!data)
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		const std::vector<std::string> parents = host.backend().parentsOf(data->id());
		parentMap[id] = parents.empty() ? QString() : QString::fromStdString(parents.front());
	}
}

void applyPointCloudPoseFromProjectJson(PointCloudBackendData& pc, OsgWidget* osgWidget, const QJsonObject& obj)
{
	const QJsonObject pose = obj.value(QStringLiteral("pose")).toObject();
	const QJsonObject rot = obj.value(QStringLiteral("rotation")).toObject();
	const QJsonObject col = obj.value(QStringLiteral("color")).toObject();
	const BackendVec3 p{pose.value(QStringLiteral("x")).toDouble(), pose.value(QStringLiteral("y")).toDouble(),
						pose.value(QStringLiteral("z")).toDouble()};
	const BackendVec3 r{rot.value(QStringLiteral("x")).toDouble(), rot.value(QStringLiteral("y")).toDouble(),
						rot.value(QStringLiteral("z")).toDouble()};
	const BackendColor c{static_cast<float>(col.value(QStringLiteral("r")).toDouble(1.0)),
						 static_cast<float>(col.value(QStringLiteral("g")).toDouble(1.0)),
						 static_cast<float>(col.value(QStringLiteral("b")).toDouble(1.0)),
						 static_cast<float>(col.value(QStringLiteral("a")).toDouble(1.0))};
	pc.setPose(p);
	pc.setRotation(r);
	pc.setColor(c);
	if (!osgWidget)
	{
		return;
	}
	osgWidget->setSelectedPosition(
		osg::Vec3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)));
	osgWidget->setSelectedRotationEulerDeg(
		osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z)));
	osgWidget->setSelectedColor(c.r, c.g, c.b, c.a);
}

void loadProjectObjectsFromJson(DocumentHost& host, const QJsonArray& objects, const ProjectObjectLoadOptions& options,
								const ProjectObjectLoadCallbacks& callbacks, QStringList* outWarnings)
{
	// 每次加载新工程时清空 stepSidecar 缓存，避免跨工程污染
	g_stepSidecarCache.clear();

	OsgWidget* osg = osgWidgetFrom(host);
	for (const QJsonValue& v : objects)
	{
		if (!v.isObject())
		{
			continue;
		}
		const QJsonObject obj = v.toObject();
		const QString sourcePath = obj.value(QStringLiteral("sourcePath")).toString();
		const QString assetRelativePath = obj.value(QStringLiteral("assetRelativePath")).toString();
		const QString sourceType = obj.value(QStringLiteral("sourceType")).toString();
		const QString legacyParentId = obj.value(QStringLiteral("parentId")).toString();
		const QString persistedId = obj.value(QStringLiteral("id")).toString();
		const QString classNameVal = obj.value(QStringLiteral("className")).toString();
		const QJsonObject emb = obj.value(QStringLiteral("geometry")).toObject();
		const bool hasEmb = !emb.isEmpty();

		if (classNameVal == QStringLiteral("Compass") ||
			sourceType.compare(QStringLiteral("Compass"), Qt::CaseInsensitive) == 0)
		{
			continue;
		}

		if (classNameVal.isEmpty())
		{
			appendProjectLoadWarning(outWarnings,
									 QStringLiteral("Skip object with empty className: %1").arg(persistedId));
			continue;
		}

		// 坐标系无文件几何，仅靠 pose/worldMatrix；仍须走内嵌注册
		const std::string classNameUtf8 = classNameVal.toStdString();
		const bool isCoordinateFrame =
			backend_type::isCoordinateFrameClassName(classNameUtf8) ||
			sourceType.compare(QLatin1String(backend_type::kCatalogCoordinateFrame), Qt::CaseInsensitive) == 0;
		const bool isCustomDevice =
			backend_type::isCustomDeviceClassName(classNameUtf8) ||
			sourceType.compare(QLatin1String(backend_type::kCatalogCustomDevice), Qt::CaseInsensitive) == 0;
		// URDF 空壳根常无内嵌几何；勿因缺 sourcePath 直接跳过
		const bool isUrdfRobotShellHint =
			sourceType.compare(QStringLiteral("URDF"), Qt::CaseInsensitive) == 0 ||
			persistedId.startsWith(QStringLiteral("RobotURDF_"));
		const bool isRobotLinkMesh = options.robotLinkMeshBackendIds.contains(persistedId);

		if (!hasEmb && sourcePath.isEmpty() && assetRelativePath.isEmpty() && !isCoordinateFrame && !isCustomDevice &&
			!isUrdfRobotShellHint && !isRobotLinkMesh)
		{
			continue;
		}

		const RobotLinkUrdfReloadHint* linkReloadHint = nullptr;
		RobotLinkUrdfReloadHint linkReloadHintStorage;
		if (options.robotLinkUrdfReloadHints.contains(persistedId))
		{
			linkReloadHintStorage = options.robotLinkUrdfReloadHints.value(persistedId);
			linkReloadHint = &linkReloadHintStorage;
		}

		if (hasEmb || isCoordinateFrame || isCustomDevice || isRobotLinkMesh)
		{
			const QString catalogType =
				sourceType.isEmpty() ? QString::fromStdString(backend_type::catalogTypeFromClassName(classNameUtf8))
									 : sourceType;
			// edges 模式父链由 edges[] 统一写，勿用 JSON parentId
			const QString parentId = options.useEdgesRelation ? QString() : legacyParentId;
			QString visualErr;
			QString regErr;
			if (registerEmbeddedProjectObject(host, obj, persistedId, sourcePath, catalogType, parentId,
											  options.robotLinkMeshBackendIds.contains(persistedId), options.projectDir,
											  linkReloadHint, &visualErr, &regErr))
			{
				if (!parentId.isEmpty() && callbacks.legacyParentFollow)
				{
					if (const std::shared_ptr<BackendDataBase> registered =
							host.backend().getData(persistedId.toStdString()))
					{
						callbacks.legacyParentFollow(registered->id(), parentId.toStdString());
					}
				}
				continue;
			}
			if (!visualErr.isEmpty())
			{
				appendProjectLoadWarning(
					outWarnings,
					QStringLiteral("Embedded backend visual load failed (id=%1): %2").arg(persistedId, visualErr));
			}
			else if (!regErr.isEmpty())
			{
				appendProjectLoadWarning(
					outWarnings,
					QStringLiteral("Embedded backend register failed (id=%1): %2").arg(persistedId, regErr));
			}
			// 坐标系无文件回退路径；注册失败则跳过，避免误报 Missing data
			if (isCoordinateFrame)
			{
				continue;
			}
		}

		const QString loadPath = resolveProjectObjectLoadPath(options.projectDir, sourcePath, assetRelativePath);

		// URDF 层级空壳根：无三角面；勿当网格文件导入（否则根丢失，树顶变成 base_link）
		const QString urdfProbePath =
			!loadPath.isEmpty() ? loadPath : (!sourcePath.isEmpty() ? sourcePath : assetRelativePath);
		const bool isUrdfRobotRootShell =
			isUrdfRobotShellHint && isUrdfRobotSceneRootShellCandidate(persistedId, options) &&
			(urdfProbePath.isEmpty() ||
			 QFileInfo(urdfProbePath).suffix().compare(QStringLiteral("urdf"), Qt::CaseInsensitive) == 0);
		if (isUrdfRobotRootShell)
		{
			const QString parentId = options.useEdgesRelation ? QString() : legacyParentId;
			const QString catalogType = QStringLiteral("URDF");
			QString visualErr;
			QString regErr;
			if (registerEmbeddedProjectObject(host, obj, persistedId, sourcePath, catalogType, parentId, false,
											  options.projectDir, nullptr, &visualErr, &regErr))
			{
				if (!parentId.isEmpty() && callbacks.legacyParentFollow)
				{
					if (const std::shared_ptr<BackendDataBase> registered =
							host.backend().getData(persistedId.toStdString()))
					{
						callbacks.legacyParentFollow(registered->id(), parentId.toStdString());
					}
				}
				continue;
			}
			appendProjectLoadWarning(
				outWarnings,
				QStringLiteral("URDF robot root shell register failed (id=%1): %2")
					.arg(persistedId, !regErr.isEmpty() ? regErr : visualErr));
			continue;
		}

		if (loadPath.isEmpty())
		{
			appendProjectLoadWarning(outWarnings,
									 QStringLiteral("Missing data (no usable embedded geometry and file missing): %1")
										 .arg(sourcePath.isEmpty() ? assetRelativePath : sourcePath));
			continue;
		}
		// sourceType 可能为空；须同时认 className，否则纯顶点 PLY 会当网格导入并失败跳过
		const bool isPc =
			sourceType.compare(QLatin1String(backend_type::kCatalogPointCloud), Qt::CaseInsensitive) == 0 ||
			backend_type::isPointCloudClassName(classNameUtf8);
		const QString catalogType =
			sourceType.isEmpty()
				? (isPc ? QLatin1String(backend_type::kCatalogPointCloud) : QLatin1String(backend_type::kCatalogModel))
				: sourceType;
		QString importErr;
		QString importedId = importProjectObjectFromFile(host, loadPath, persistedId, catalogType, isPc, &importErr);
		// las/laz 等 importPointCloudFile 失败时的 Widget 回退（与 importFileIntoDocument 一致）
		if (importedId.isEmpty() && isPc && callbacks.pointCloudWidgetImport)
		{
			if (!callbacks.pointCloudWidgetImport(host, loadPath, persistedId, importedId, &importErr))
			{
				appendProjectLoadWarning(
					outWarnings, QStringLiteral("Failed to load object from file: %1 (%2)").arg(loadPath, importErr));
				continue;
			}
		}
		else if (importedId.isEmpty())
		{
			appendProjectLoadWarning(
				outWarnings, QStringLiteral("Failed to load object from file: %1 (%2)").arg(loadPath, importErr));
			continue;
		}
		if (!importedId.isEmpty())
		{
			// 文件导入只恢复几何；visible/pose 等从工程 JSON 写回
			if (const auto data = host.backend().getData(importedId.toStdString()))
			{
				const bool visible = obj.value(QStringLiteral("visible")).toBool(true);
				data->setVisible(visible);
				if (osg)
				{
					osg->setBackendObjectVisible(importedId.toStdString(), visible);
				}
			}
			if (auto pc =
					std::dynamic_pointer_cast<PointCloudBackendData>(host.backend().getData(importedId.toStdString())))
			{
				applyPointCloudPoseFromProjectJson(*pc, osg, obj);
			}
		}
	}
}

void finalizeProjectHierarchyAfterObjects(DocumentHost& host, const bool useEdgesRelation,
										  const QVector<ProjectHierarchyEdge>& edges, QStringList* outWarnings)
{
	if (useEdgesRelation)
	{
		applyProjectEdgesToBackend(host, edges, outWarnings);
	}
	rebuildBackendParentIdMirror(host);
	collectDanglingBackendRefs(host, outWarnings);
}

bool exportBackendTriangleSoupMm(DocumentHost& host, const QString& backendId, std::vector<float>& outSoup,
								 QString* outError)
{
	outSoup.clear();
	const auto data = host.backend().getData(backendId.toStdString());
	if (!data)
	{
		if (outError)
			*outError = QStringLiteral("Unknown id.");
		return false;
	}
	if (const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
	{
		if (mesh->triangleSoup().empty())
		{
			if (outError)
				*outError = QStringLiteral("No mesh geometry.");
			return false;
		}
		outSoup = mesh->triangleSoup();
		return true;
	}
	if (const auto brep = std::dynamic_pointer_cast<BrepBackendData>(data))
	{
		std::string tessErr;
		if (!geoalgo::tessellateShapeMedium(brep->shapeRef(), outSoup, nullptr, &tessErr) || outSoup.size() < 9U)
		{
			if (outError)
			{
				*outError = tessErr.empty() ? QStringLiteral("B-rep tessellation failed.")
											: QString::fromStdString(tessErr);
			}
			outSoup.clear();
			return false;
		}
		return true;
	}
	if (outError)
		*outError = QStringLiteral("No mesh geometry.");
	return false;
}

void applyProjectEdgesFollowBindingAndSolve(DocumentHost& host, const QVector<ProjectHierarchyEdge>& edges,
											const FollowSolveContext* solveCtx)
{
	(void)edges;
	registerAllCustomDeviceLinkGeometryOwnership(host);
	host.stripKinematicsOwnedFollowAttachments();
	host.stripHierarchyDrivenFollowAttachments();
	// edges 只表示 Data 父子；跨部件 Follow 靠对象上显式组件
	runBackendFollowSolveAndSync(host, osgWidgetFrom(host), solveCtx);
}

} // namespace cloudsim::host
