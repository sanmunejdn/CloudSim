#include "BackendProjectObjectIo.h"

#include "BackendFileImport.h"
#include "BackendFollowSolve.h"
#include "BackendHierarchyFollow.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "IDataService.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "FollowAttachmentComponent.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"

#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonValue>
#include <QMap>

#include <osg/Vec3f>

#include <json.hpp>

namespace cloudsim::host {

namespace {

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
	const QString& sourcePath, const QString& catalogTypeName, const QString& parentId, const bool robotLinkMeshVisual,
	QString* outVisualError, QString* outError)
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
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		if (outError)
		{
			*outError = QStringLiteral("no active 3D view");
		}
		return false;
	}
	QString visualErr;
	bool visualOk = false;
	if (const auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(backendObject))
	{
		visualOk = osg->loadPointCloudFromBackendData(*pc, &visualErr, true);
	}
	else if (const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(backendObject))
	{
		// robotLinkMeshVisual：连杆网格已在 link 系，勿二次中心化
		visualOk = osg->loadMeshFromBackendData(*mesh, &visualErr, true, true, true, robotLinkMeshVisual);
	}
	else
	{
		if (outError)
		{
			*outError = QStringLiteral("unsupported backend class: %1")
				.arg(QString::fromStdString(backendObject->className()));
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
		? (isPointCloud ? QStringLiteral("PointCloud") : QStringLiteral("Model"))
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

void applyProjectEdgesToBackend(DocumentHost& host, const QVector<ProjectHierarchyEdge>& edges, QStringList* outWarnings)
{
	for (const ProjectHierarchyEdge& edge : edges)
	{
		const std::string parentId = edge.parentId.toStdString();
		const std::string childId = edge.childId.toStdString();
		if (!host.backend().contains(parentId) || !host.backend().contains(childId))
		{
			if (outWarnings)
			{
				outWarnings->append(
					QStringLiteral("Skip dangling edge: %1 -> %2").arg(edge.parentId, edge.childId));
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
	for (const auto& data : host.backend().listData())
	{
		if (!data)
		{
			continue;
		}
		const std::vector<std::string> parents = host.backend().parentsOf(data->id());
		const std::string parent = parents.empty() ? std::string() : parents.front();
		osg->setBackendParent(data->id(), parent);
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
	const BackendVec3 p{
		pose.value(QStringLiteral("x")).toDouble(),
		pose.value(QStringLiteral("y")).toDouble(),
		pose.value(QStringLiteral("z")).toDouble()
	};
	const BackendVec3 r{
		rot.value(QStringLiteral("x")).toDouble(),
		rot.value(QStringLiteral("y")).toDouble(),
		rot.value(QStringLiteral("z")).toDouble()
	};
	const BackendColor c{
		static_cast<float>(col.value(QStringLiteral("r")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("g")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("b")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("a")).toDouble(1.0))
	};
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

		if (classNameVal == QStringLiteral("Compass")
			|| sourceType.compare(QStringLiteral("Compass"), Qt::CaseInsensitive) == 0)
		{
			continue;
		}

		if (classNameVal.isEmpty())
		{
			appendProjectLoadWarning(outWarnings,
				QStringLiteral("Skip object with empty className: %1").arg(persistedId));
			continue;
		}

		if (!hasEmb && sourcePath.isEmpty() && assetRelativePath.isEmpty())
		{
			continue;
		}

		if (hasEmb)
		{
			const QString catalogType =
				sourceType.isEmpty()
				? (classNameVal == QStringLiteral("PointCloudBackendData") ? QStringLiteral("PointCloud")
																		  : QStringLiteral("Model"))
				: sourceType;
			// edges 模式父链由 edges[] 统一写，勿用 JSON parentId
			const QString parentId = options.useEdgesRelation ? QString() : legacyParentId;
			QString visualErr;
			QString regErr;
			if (registerEmbeddedProjectObject(host, obj, persistedId, sourcePath, catalogType, parentId,
					options.robotLinkMeshBackendIds.contains(persistedId), &visualErr, &regErr))
			{
				if (!parentId.isEmpty() && callbacks.legacyParentFollow)
				{
					if (const std::shared_ptr<BackendDataBase> registered = host.backend().getData(persistedId.toStdString()))
					{
						callbacks.legacyParentFollow(registered->id(), parentId.toStdString());
					}
				}
				continue;
			}
			if (!visualErr.isEmpty())
			{
				appendProjectLoadWarning(outWarnings,
					QStringLiteral("Embedded backend visual load failed (id=%1): %2").arg(persistedId, visualErr));
			}
			else if (!regErr.isEmpty())
			{
				appendProjectLoadWarning(outWarnings,
					QStringLiteral("Embedded backend register failed (id=%1): %2").arg(persistedId, regErr));
			}
		}

		const QString loadPath = resolveProjectObjectLoadPath(options.projectDir, sourcePath, assetRelativePath);
		if (loadPath.isEmpty())
		{
			appendProjectLoadWarning(outWarnings,
				QStringLiteral("Missing data (no usable embedded geometry and file missing): %1")
					.arg(sourcePath.isEmpty() ? assetRelativePath : sourcePath));
			continue;
		}
		const bool isPc = sourceType.compare(QStringLiteral("PointCloud"), Qt::CaseInsensitive) == 0;
		const QString catalogType =
			sourceType.isEmpty()
			? (isPc ? QStringLiteral("PointCloud") : QStringLiteral("Model"))
			: sourceType;
		QString importErr;
		QString importedId = importProjectObjectFromFile(host, loadPath, persistedId, catalogType, isPc, &importErr);
		// las/laz 等 importPointCloudFile 失败时的 Widget 回退（与 importFileIntoDocument 一致）
		if (importedId.isEmpty() && isPc && callbacks.pointCloudWidgetImport)
		{
			if (!callbacks.pointCloudWidgetImport(host, loadPath, persistedId, importedId, &importErr))
			{
				appendProjectLoadWarning(outWarnings,
					QStringLiteral("Failed to load object from file: %1 (%2)").arg(loadPath, importErr));
				continue;
			}
		}
		else if (importedId.isEmpty())
		{
			appendProjectLoadWarning(outWarnings,
				QStringLiteral("Failed to load object from file: %1 (%2)").arg(loadPath, importErr));
			continue;
		}
		if (!importedId.isEmpty())
		{
			// 文件导入只恢复几何，pose/color 仍从工程 JSON 写回
			if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(host.backend().getData(importedId.toStdString())))
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
}

void applyProjectEdgesFollowBindingAndSolve(DocumentHost& host, OsgWidget& osg, const QVector<ProjectHierarchyEdge>& edges,
	const FollowSolveContext* solveCtx)
{
	for (const ProjectHierarchyEdge& edge : edges)
	{
		const std::shared_ptr<BackendDataBase> childData = host.backend().getData(edge.childId.toStdString());
		// 工程里已带 followAttachment 的节点不再重复 binding
		if (childData && childData->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
		{
			continue;
		}
		applyHierarchyFollowBinding(host, edge.childId.toStdString(), edge.parentId.toStdString());
	}
	runBackendFollowSolveAndSync(host, osg, solveCtx);
}

} // namespace cloudsim::host
