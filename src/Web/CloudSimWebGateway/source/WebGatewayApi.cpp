/// @file WebGatewayApi.cpp
/// @brief WebGateway P1–P5 请求处理（主线程契约调用）

#include "WebGateway.h"

#include "DocumentHost.h"
#include "DocumentImportFacade.h"
#include "IDataService.h"
#include "IDocumentScope.h"
#include "IRobotService.h"
#include "ProjectPackageIo.h"
#include "StoreZipExtract.h"
#include "WebGatewaySidecars.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QVariant>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace cloudsim::web
{
namespace
{
struct SidecarStore
{
	QJsonObject processFlow;
	QJsonObject geometricModeling;
	QJsonArray annotations;
	QString workspaceMode = QStringLiteral("scene3d");
};
SidecarStore g_sidecars;

QString jsonEscapeApi(const QString& s)
{
	const QByteArray quoted = QJsonDocument(QJsonArray{s}).toJson(QJsonDocument::Compact);
	if (quoted.size() >= 4)
		return QString::fromUtf8(quoted.mid(2, quoted.size() - 4));
	return s;
}

cloudsim::core::PoseDto poseFromJson(const QJsonObject& o)
{
	cloudsim::core::PoseDto p;
	const QJsonArray pos = o.value(QStringLiteral("positionMm")).toArray();
	const QJsonArray eu = o.value(QStringLiteral("eulerDeg")).toArray();
	if (pos.size() >= 3)
	{
		p.positionMm.x = pos[0].toDouble();
		p.positionMm.y = pos[1].toDouble();
		p.positionMm.z = pos[2].toDouble();
	}
	if (eu.size() >= 3)
	{
		p.eulerDeg.x = eu[0].toDouble();
		p.eulerDeg.y = eu[1].toDouble();
		p.eulerDeg.z = eu[2].toDouble();
	}
	return p;
}
} // namespace

void webGatewayLoadSidecarsFromProject(const QJsonObject& root)
{
	g_sidecars.processFlow = root.value(QStringLiteral("processFlow")).toObject();
	g_sidecars.geometricModeling = root.value(QStringLiteral("geometricModeling")).toObject();
	g_sidecars.annotations = root.value(QStringLiteral("annotations")).toArray();
}

void webGatewayMergeSidecarsIntoProject(QJsonObject& root)
{
	if (!g_sidecars.processFlow.isEmpty())
		root.insert(QStringLiteral("processFlow"), g_sidecars.processFlow);
	if (!g_sidecars.geometricModeling.isEmpty())
		root.insert(QStringLiteral("geometricModeling"), g_sidecars.geometricModeling);
	if (!g_sidecars.annotations.isEmpty())
		root.insert(QStringLiteral("annotations"), g_sidecars.annotations);
}

bool WebGateway::newProjectOnGuiThread(cloudsim::host::DocumentHost* host, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	host->data().clear();
	host->backendSourcePath().clear();
	host->backendSourceType().clear();
	host->backendParentId().clear();
	host->setProjectFilePath(QString());
	g_sidecars = SidecarStore{};
	pushEvent(QStringLiteral("{\"type\":\"ProjectLoaded\",\"path\":\"\",\"objectCount\":0}"));
	return true;
}

bool WebGateway::saveProjectOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err)
{
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	QString savePath = host->projectFilePath();
	if (doc.isObject())
	{
		const QString p = doc.object().value(QStringLiteral("path")).toString();
		if (!p.isEmpty())
			savePath = p;
	}
	if (savePath.isEmpty())
	{
		if (err)
			*err = QStringLiteral("Save path required.");
		return false;
	}

	releaseProjectFileLock();
	const auto restoreLock = [this, host]() { reacquireProjectFileLock(host->projectFilePath()); };

	const QFileInfo fi(savePath);
	const bool packageMode = fi.suffix().compare(QStringLiteral("pcp"), Qt::CaseInsensitive) == 0;
	QTemporaryDir temp;
	const QString workRoot = packageMode ? temp.path() : fi.absolutePath();
	if (packageMode && !temp.isValid())
	{
		if (err)
			*err = QStringLiteral("Cannot create temp dir.");
		restoreLock();
		return false;
	}
	const QString jsonPath =
		packageMode ? QDir(workRoot).filePath(QStringLiteral("project.json")) : savePath;

	auto built = cloudsim::host::buildProjectSaveRoot(*host, QStringLiteral("zh"), workRoot);
	if (!built.abortMessage.isEmpty())
	{
		if (err)
			*err = built.abortMessage;
		restoreLock();
		return false;
	}
	cloudsim::host::mergeRobotProgramsIntoProjectRoot(*host, built.root);
	webGatewayMergeSidecarsIntoProject(built.root);

	QFile out(jsonPath);
	if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		if (err)
			*err = QStringLiteral("Cannot write project.json.");
		restoreLock();
		return false;
	}
	out.write(QJsonDocument(built.root).toJson(QJsonDocument::Indented));
	out.close();

	if (packageMode)
	{
		QString zipErr;
		if (!packStoreZipArchive(savePath, workRoot, &zipErr))
		{
			if (err)
				*err = zipErr;
			restoreLock();
			return false;
		}
	}
	host->setProjectFilePath(savePath);
	reacquireProjectFileLock(savePath);
	pushEvent(QStringLiteral("{\"type\":\"ProjectSaved\",\"path\":\"%1\"}").arg(jsonEscapeApi(savePath)));
	return true;
}

QByteArray WebGateway::objectDetailJsonOnGuiThread(const QString& id)
{
	if (!m_document)
		return QByteArrayLiteral("{}");
	auto& data = m_document->data();
	if (!data.isValid(id))
		return QByteArrayLiteral("{\"ok\":false,\"error\":\"unknown id\"}");
	const auto snap = data.objectSnapshot(id);
	QJsonObject o;
	o.insert(QStringLiteral("id"), snap.id);
	o.insert(QStringLiteral("name"), snap.name);
	o.insert(QStringLiteral("className"), snap.className);
	o.insert(QStringLiteral("visible"), snap.visible);
	o.insert(QStringLiteral("hasGeometry"), snap.hasGeometry);
	o.insert(QStringLiteral("geometryKind"), static_cast<int>(snap.geometryKind));
	const auto pose = data.worldPoseMm(id);
	o.insert(QStringLiteral("pose"),
			 QJsonObject{{QStringLiteral("positionMm"), QJsonArray{pose.positionMm.x, pose.positionMm.y, pose.positionMm.z}},
						 {QStringLiteral("eulerDeg"), QJsonArray{pose.eulerDeg.x, pose.eulerDeg.y, pose.eulerDeg.z}}});
	QJsonArray rows;
	for (const auto& r : data.propertyRows(id))
	{
		rows.append(QJsonObject{{QStringLiteral("key"), r.key},
								{QStringLiteral("labelEn"), r.labelEn},
								{QStringLiteral("editable"), r.editable},
								{QStringLiteral("value"), r.value}});
	}
	o.insert(QStringLiteral("properties"), rows);
	return QJsonDocument(o).toJson(QJsonDocument::Compact);
}

bool WebGateway::patchObjectOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
										QString* err)
{
	if (!host || !host->data().isValid(id))
	{
		if (err)
			*err = QStringLiteral("Unknown id.");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject())
	{
		if (err)
			*err = QStringLiteral("Invalid JSON.");
		return false;
	}
	const QJsonObject o = doc.object();
	auto& data = host->data();
	if (o.contains(QStringLiteral("visible")))
	{
		if (!data.setVisible(id, o.value(QStringLiteral("visible")).toBool(), err))
			return false;
	}
	if (o.contains(QStringLiteral("pose")))
	{
		if (!data.applyWorldPoseMm(id, poseFromJson(o.value(QStringLiteral("pose")).toObject()), err))
			return false;
		pushEvent(QStringLiteral("{\"type\":\"PoseCommitted\",\"backendId\":\"%1\"}").arg(id));
	}
	if (o.contains(QStringLiteral("properties")) && o.value(QStringLiteral("properties")).isObject())
	{
		const QJsonObject props = o.value(QStringLiteral("properties")).toObject();
		for (auto it = props.begin(); it != props.end(); ++it)
		{
			if (!data.applyPropertyChange(id, it.key(), it.value().toVariant().toString(), err))
				return false;
		}
	}
	if (o.contains(QStringLiteral("propertyKey")))
	{
		if (!data.applyPropertyChange(id, o.value(QStringLiteral("propertyKey")).toString(),
									  o.value(QStringLiteral("propertyValue")).toVariant().toString(), err))
			return false;
	}
	pushEvent(QStringLiteral("{\"type\":\"ObjectPatched\",\"backendId\":\"%1\"}").arg(id));
	return true;
}

bool WebGateway::importObjectOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err,
										 QString* outId)
{
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject() || !host)
	{
		if (err)
			*err = QStringLiteral("Invalid body.");
		return false;
	}
	const QJsonObject o = doc.object();
	const QString path = o.value(QStringLiteral("path")).toString();
	if (path.isEmpty() || !QFileInfo::exists(path))
	{
		if (err)
			*err = QStringLiteral("Import path missing.");
		return false;
	}
	cloudsim::core::ImportOptionsDto opt;
	opt.quietUi = true;
	opt.resetViewToHome = false;
	opt.isPointCloud = o.value(QStringLiteral("isPointCloud")).toBool(false);
	const auto kind =
		opt.isPointCloud ? cloudsim::host::ImportFileKind::PointCloud : cloudsim::host::ImportFileKind::Mesh;
	const auto imported = cloudsim::host::importFileIntoDocument(*host, path, kind, opt, err);
	if (!imported.ok)
		return false;
	if (outId)
		*outId = imported.rootBackendId;
	pushEvent(QStringLiteral("{\"type\":\"BackendObjectRegistered\",\"backendId\":\"%1\"}").arg(imported.rootBackendId));
	return true;
}

bool WebGateway::deleteObjectOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, QString* err)
{
	if (!host)
		return false;
	if (!host->data().unregisterSubtree(id, err))
		return false;
	pushEvent(QStringLiteral("{\"type\":\"BackendObjectRemoved\",\"backendId\":\"%1\"}").arg(id));
	return true;
}

bool WebGateway::attachChildOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err)
{
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject() || !host)
	{
		if (err)
			*err = QStringLiteral("Invalid body.");
		return false;
	}
	const QString parentId = doc.object().value(QStringLiteral("parentId")).toString();
	const QString childId = doc.object().value(QStringLiteral("childId")).toString();
	return host->data().attachChild(parentId, childId, err);
}

QByteArray WebGateway::robotProgramsJsonOnGuiThread()
{
	if (!m_document)
		return QByteArrayLiteral("[]");
	return QJsonDocument(m_document->robot().robotProgramsJson()).toJson(QJsonDocument::Compact);
}

bool WebGateway::setRobotProgramsOnGuiThread(const QByteArray& body, QString* err)
{
	if (!m_document)
		return false;
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	QJsonArray arr;
	if (doc.isArray())
		arr = doc.array();
	else if (doc.isObject())
		arr = doc.object().value(QStringLiteral("programs")).toArray();
	else
	{
		if (err)
			*err = QStringLiteral("Expect programs array.");
		return false;
	}
	return m_document->robot().setRobotProgramsJson(arr, err);
}

bool WebGateway::applyJointsOnGuiThread(const QByteArray& body, QString* err)
{
	if (!m_document)
		return false;
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject())
	{
		if (err)
			*err = QStringLiteral("Invalid JSON.");
		return false;
	}
	const QJsonObject o = doc.object();
	const QString rootId = o.value(QStringLiteral("sceneRootBackendId")).toString();
	QVector<double> joints;
	for (const auto& v : o.value(QStringLiteral("jointAnglesRad")).toArray())
		joints.push_back(v.toDouble());
	QVector<double> agg;
	if (!m_document->robot().applyJointAnglesRad(rootId, joints, &agg, err))
		return false;
	QJsonArray a;
	for (double d : agg)
		a.append(d);
	pushEvent(QStringLiteral("{\"type\":\"RobotKinematicsApplied\",\"sceneRootBackendId\":\"%1\",\"joints\":%2}")
				  .arg(rootId, QString::fromUtf8(QJsonDocument(a).toJson(QJsonDocument::Compact))));
	return true;
}

bool WebGateway::registerUrdfOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out)
{
	if (!m_document)
		return false;
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject())
	{
		if (err)
			*err = QStringLiteral("Invalid JSON.");
		return false;
	}
	const QString urdfPath = doc.object().value(QStringLiteral("urdfPath")).toString();
	cloudsim::core::ImportOptionsDto opt;
	opt.quietUi = true;
	const auto reg = m_document->robot().registerUrdfRobot(urdfPath, opt);
	if (out)
	{
		(*out)[QStringLiteral("ok")] = reg.ok;
		(*out)[QStringLiteral("sceneRootBackendId")] = reg.sceneRootBackendId;
		(*out)[QStringLiteral("error")] = reg.error;
	}
	if (!reg.ok)
	{
		if (err)
			*err = reg.error.isEmpty() ? QStringLiteral("URDF import failed.") : reg.error;
		return false;
	}
	pushEvent(QStringLiteral("{\"type\":\"BackendObjectRegistered\",\"backendId\":\"%1\"}").arg(reg.sceneRootBackendId));
	return true;
}

bool WebGateway::planInstructionOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out)
{
	if (!m_document)
		return false;
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject())
	{
		if (err)
			*err = QStringLiteral("Invalid JSON.");
		return false;
	}
	const QJsonObject o = doc.object();
	cloudsim::core::MotionInstructionDto instr;
	instr.instructionType = o.value(QStringLiteral("instructionType")).toString(QStringLiteral("PTP"));
	instr.jointRadCsv = o.value(QStringLiteral("jointRadCsv")).toString();
	instr.axisConfiguration = o.value(QStringLiteral("axisConfiguration")).toObject();
	instr.extensions = o.value(QStringLiteral("extensions")).toObject();
	if (o.contains(QStringLiteral("targetPose")))
		instr.targetPose = poseFromJson(o.value(QStringLiteral("targetPose")).toObject());

	cloudsim::core::PlanContextDto ctx;
	const QString sceneRoot = o.value(QStringLiteral("sceneRootBackendId")).toString();
	if (!sceneRoot.isEmpty())
		ctx.extensions.insert(QStringLiteral("sceneRootBackendId"), sceneRoot);
	if (o.contains(QStringLiteral("urdfPath")))
		ctx.urdfPath = o.value(QStringLiteral("urdfPath")).toString();
	cloudsim::core::PlanResultDto result;
	if (!m_document->robot().planInstruction(instr, ctx, result, err))
		return false;
	if (out)
	{
		(*out)[QStringLiteral("ok")] = result.ok;
		(*out)[QStringLiteral("error")] = result.error;
		QJsonArray joints;
		for (double d : result.jointTargetsRad)
			joints.append(d);
		(*out)[QStringLiteral("jointTargetsRad")] = joints;
	}
	return result.ok;
}

QByteArray WebGateway::sidecarGetOnGuiThread(const QString& key)
{
	if (key == QLatin1String("processFlow"))
		return QJsonDocument(g_sidecars.processFlow).toJson(QJsonDocument::Compact);
	if (key == QLatin1String("geometricModeling"))
		return QJsonDocument(g_sidecars.geometricModeling).toJson(QJsonDocument::Compact);
	if (key == QLatin1String("annotations"))
		return QJsonDocument(g_sidecars.annotations).toJson(QJsonDocument::Compact);
	if (key == QLatin1String("workspaceMode"))
		return QJsonDocument(QJsonObject{{QStringLiteral("mode"), g_sidecars.workspaceMode}})
			.toJson(QJsonDocument::Compact);
	return QByteArrayLiteral("{}");
}

bool WebGateway::sidecarPutOnGuiThread(const QString& key, const QByteArray& body, QString* err)
{
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (key == QLatin1String("processFlow") && doc.isObject())
	{
		g_sidecars.processFlow = doc.object();
		return true;
	}
	if (key == QLatin1String("geometricModeling") && doc.isObject())
	{
		g_sidecars.geometricModeling = doc.object();
		return true;
	}
	if (key == QLatin1String("annotations") && doc.isArray())
	{
		g_sidecars.annotations = doc.array();
		return true;
	}
	if (key == QLatin1String("workspaceMode") && doc.isObject())
	{
		g_sidecars.workspaceMode = doc.object().value(QStringLiteral("mode")).toString(QStringLiteral("scene3d"));
		pushEvent(QStringLiteral("{\"type\":\"WorkspaceModeChanged\",\"mode\":\"%1\"}").arg(g_sidecars.workspaceMode));
		return true;
	}
	if (err)
		*err = QStringLiteral("Unknown sidecar or bad body.");
	return false;
}

QByteArray WebGateway::modesCatalogJson() const
{
	QJsonArray modes;
	modes.append(QJsonObject{{QStringLiteral("id"), QStringLiteral("scene3d")},
							 {QStringLiteral("title"), QStringLiteral("三维场景")}});
	modes.append(QJsonObject{{QStringLiteral("id"), QStringLiteral("geomodeling")},
							 {QStringLiteral("title"), QStringLiteral("几何建模")}});
	modes.append(QJsonObject{{QStringLiteral("id"), QStringLiteral("processflow")},
							 {QStringLiteral("title"), QStringLiteral("工艺流程")}});
	modes.append(QJsonObject{{QStringLiteral("id"), QStringLiteral("drawing")},
							 {QStringLiteral("title"), QStringLiteral("工程图")}});
	modes.append(QJsonObject{{QStringLiteral("id"), QStringLiteral("labeling")},
							 {QStringLiteral("title"), QStringLiteral("标注")}});
	return QJsonDocument(QJsonObject{{QStringLiteral("modes"), modes},
									 {QStringLiteral("active"), g_sidecars.workspaceMode}})
		.toJson(QJsonDocument::Compact);
}

QByteArray WebGateway::helpIndexJson() const
{
	return QByteArrayLiteral(
		"{\"helpRoot\":\"help/\",\"entries\":[{\"id\":\"web\",\"title\":\"CloudSim Web\",\"path\":\"index.html\"}]}");
}

QByteArray WebGateway::aiStatusJson() const
{
	return QByteArrayLiteral(
		"{\"ok\":true,\"endpoint\":\"/api/ai/chat\",\"note\":\"P5 stub: configure ai_config.json for desktop LLM\"}");
}

} // namespace cloudsim::web
