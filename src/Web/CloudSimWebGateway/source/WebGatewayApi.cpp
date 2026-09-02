/// @file WebGatewayApi.cpp
/// @brief WebGateway P1–P5 请求处理（主线程契约调用）

#include "WebGateway.h"

#include "BackendFileImport.h"
#include "BackendTypeIds.h"
#include "CloudSimHost.h"
#include "io/CustomDeviceHostOps.h"
#include "DocumentHost.h"
#include "DocumentImportFacade.h"
#include "FrameBackendData.h"
#include "HeadlessRobotContext.h"
#include "IDataService.h"
#include "IDocumentScope.h"
#include "io/IoSignalNetwork.h"
#include "IRobotService.h"
#include "ProjectPackageIo.h"
#include "RobotCoordinateFrameOps.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionModel.h"
#include "RobotProgramCatalog.h"
#include "RobotProgramStore.h"
#include "StoreZipExtract.h"
#include "WebGatewaySidecars.h"
#include "headless/HeadlessDrawingBridge.h"
#include "headless/HeadlessLabelingBridge.h"
#include "headless/HeadlessProcessFlowBridge.h"
#include "NamedSignalTable.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QVariant>

#include <json.hpp>
#include <memory>
#include <vector>

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

QJsonObject namedSignalTableToQJson(const RobotIo::NamedSignalTable& table)
{
	const nlohmann::json j = table.toJson();
	return QJsonDocument::fromJson(QByteArray::fromStdString(j.dump())).object();
}

bool namedSignalTableFromQJson(RobotIo::NamedSignalTable& table, const QJsonObject& obj, QString* err)
{
	std::string e;
	const QByteArray raw = QJsonDocument(obj).toJson(QJsonDocument::Compact);
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(raw.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("ioSignals JSON invalid");
		return false;
	}
	if (!table.fromJson(j, &e))
	{
		if (err)
			*err = QString::fromStdString(e);
		return false;
	}
	return true;
}

QString jsonEscapeApi(const QString& s)
{
	const QByteArray quoted = QJsonDocument(QJsonArray{s}).toJson(QJsonDocument::Compact);
	if (quoted.size() >= 4)
		return QString::fromUtf8(quoted.mid(2, quoted.size() - 4));
	return s;
}

std::shared_ptr<RobotInstruction::Base>
findInstructionInStepsApi(const std::vector<std::shared_ptr<RobotInstruction::Base>>& steps, const std::string& idUtf8)
{
	for (const auto& step : steps)
	{
		if (!step)
			continue;
		if (step->id() == idUtf8)
			return step;
		if (auto hit = findInstructionInStepsApi(step->nestedSteps(), idUtf8))
			return hit;
		if (auto hit = findInstructionInStepsApi(step->elseSteps(), idUtf8))
			return hit;
	}
	return nullptr;
}

std::shared_ptr<RobotInstruction::Base> findSeedInstructionApi(RobotProgramStore& store, const QString& instructionId)
{
	const std::string idUtf8 = instructionId.toStdString();
	const auto& catalogs = store.allCatalogs();
	for (auto it = catalogs.constBegin(); it != catalogs.constEnd(); ++it)
	{
		for (const RobotInstruction::RobotProgram& prog : it.value().programs())
		{
			if (auto hit = findInstructionInStepsApi(prog.steps, idUtf8))
				return hit;
		}
	}
	return nullptr;
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

void webGatewaySyncSidecarsToHost(cloudsim::host::DocumentHost* host, const QJsonObject& projectRoot)
{
	if (!host)
		return;
	if (auto* b = host->headlessProcessFlowBridge())
		b->loadSidecarFromProject(projectRoot);
	if (auto* b = host->headlessDrawingBridge())
		b->loadSidecarFromProject(projectRoot);
	if (auto* b = host->headlessLabelingBridge())
		b->loadSidecarFromProject(projectRoot);
}

void webGatewayMergeHostSidecarsIntoProject(cloudsim::host::DocumentHost* host, QJsonObject& projectRoot)
{
	if (!host)
		return;
	if (auto* b = host->headlessProcessFlowBridge())
		b->mergeSidecarIntoProject(projectRoot);
	if (auto* b = host->headlessDrawingBridge())
		b->mergeSidecarIntoProject(projectRoot);
	if (auto* b = host->headlessLabelingBridge())
		b->mergeSidecarIntoProject(projectRoot);
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
	host->resetHeadlessGeomodelHistory();
	host->backendSourcePath().clear();
	host->backendSourceType().clear();
	host->backendParentId().clear();
	if (cloudsim::host::HeadlessRobotContext* hrc = host->headlessRobotContext())
		hrc->clearRobotSimulationContext();
	host->setProjectFilePath(QString());
	host->ioSignalNetwork().clear();
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
	cloudsim::host::mergeRobotKinematicsIntoProjectRoot(*host, built.root);
	webGatewayMergeHostSidecarsIntoProject(host, built.root);
	webGatewayMergeSidecarsIntoProject(built.root);
	host->ioSignalNetwork().flushDeviceTablesToDocument(*host);
	built.root.insert(QStringLiteral("ioSignalNetwork"), host->ioSignalNetwork().toProjectJson());
	built.root.remove(QStringLiteral("ioSignals"));

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
	if (o.contains(QStringLiteral("propertyKey")) || o.contains(QStringLiteral("key")))
	{
		QString propKey = o.contains(QStringLiteral("propertyKey"))
							  ? o.value(QStringLiteral("propertyKey")).toString()
							  : o.value(QStringLiteral("key")).toString();
		const QString propVal = o.contains(QStringLiteral("propertyValue"))
									? o.value(QStringLiteral("propertyValue")).toVariant().toString()
									: o.value(QStringLiteral("value")).toVariant().toString();
		if (propKey == QStringLiteral("name"))
		{
			propKey = QStringLiteral("core.name");
		}
		if (!data.applyPropertyChange(id, propKey, propVal, err))
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

namespace
{
bool readVec3Json(const QJsonValue& v, double out[3])
{
	if (!v.isArray())
		return false;
	const QJsonArray a = v.toArray();
	if (a.size() < 3)
		return false;
	out[0] = a.at(0).toDouble();
	out[1] = a.at(1).toDouble();
	out[2] = a.at(2).toDouble();
	return true;
}
} // namespace

bool WebGateway::createCoordinateFrameOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body,
												  QString* err, QString* outId)
{
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject() || !host)
	{
		if (err)
			*err = QStringLiteral("Invalid body.");
		return false;
	}
	const QJsonObject o = doc.object();
	QString name = o.value(QStringLiteral("name")).toString().trimmed();
	if (name.isEmpty())
		name = QString::fromUtf8(backend_type::kCatalogCoordinateFrame);

	double positionMm[3]{0.0, 0.0, 0.0};
	double eulerDeg[3]{0.0, 0.0, 0.0};
	if (o.contains(QStringLiteral("positionMm")) && !readVec3Json(o.value(QStringLiteral("positionMm")), positionMm))
	{
		if (err)
			*err = QStringLiteral("positionMm[3] required");
		return false;
	}
	if (o.contains(QStringLiteral("eulerDeg")) && !readVec3Json(o.value(QStringLiteral("eulerDeg")), eulerDeg))
	{
		if (err)
			*err = QStringLiteral("eulerDeg[3] required");
		return false;
	}

	auto frame = std::make_shared<FrameBackendData>();
	frame->setName(name.toStdString());
	frame->setPose(BackendVec3{positionMm[0], positionMm[1], positionMm[2]});
	frame->setRotation(BackendVec3{eulerDeg[0], eulerDeg[1], eulerDeg[2]});
	if (o.contains(QStringLiteral("axisLengthMm")))
	{
		const double axisLen = o.value(QStringLiteral("axisLengthMm")).toDouble(FrameBackendData::kDefaultAxisLengthMm);
		frame->setAxisLengthMm(static_cast<float>(axisLen));
	}

	const QString parentId = o.value(QStringLiteral("parentId")).toString().trimmed();

	if (!cloudsim::host::registerAdoptedFrameAndLoadScene(
			*host, frame, QLatin1String(backend_type::kCatalogCoordinateFrame), parentId, false, err))
	{
		return false;
	}
	const QString id = QString::fromStdString(frame->id());
	if (outId)
		*outId = id;
	pushEvent(QStringLiteral("{\"type\":\"BackendObjectRegistered\",\"backendId\":\"%1\"}").arg(id));
	return true;
}

QByteArray WebGateway::coordinateFramesJsonOnGuiThread()
{
	QJsonArray frames;
	if (m_document)
	{
		for (const auto& snap : m_document->data().listObjectSnapshots())
		{
			if (snap.className != QLatin1String(backend_type::kClassFrame))
				continue;
			frames.append(QJsonObject{{QStringLiteral("id"), snap.id}, {QStringLiteral("name"), snap.name}});
		}
	}
	return QJsonDocument(QJsonObject{{QStringLiteral("ok"), true}, {QStringLiteral("frames"), frames}})
		.toJson(QJsonDocument::Compact);
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
	const QString parentId = doc.object().value(QStringLiteral("parentId")).toString().trimmed();
	const QString childId = doc.object().value(QStringLiteral("childId")).toString().trimmed();
	if (parentId.isEmpty() || childId.isEmpty())
	{
		if (err)
		{
			*err = QStringLiteral("parentId and childId required");
		}
		return false;
	}
	return cloudsim::host::attachBackendChildToParent(*host, parentId.toStdString(), childId.toStdString(), err);
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
	if (auto* host = cloudsim::host::documentHostFromScope(m_document.get()))
	{
		if (cloudsim::host::HeadlessRobotContext* hrc = host->headlessRobotContext())
			hrc->recordJointAnglesForSceneRoot(rootId, joints);
	}
	QJsonArray a;
	for (double d : agg)
		a.append(d);
	pushEvent(QStringLiteral("{\"type\":\"RobotKinematicsApplied\",\"sceneRootBackendId\":\"%1\",\"joints\":%2}")
				  .arg(rootId, QString::fromUtf8(QJsonDocument(a).toJson(QJsonDocument::Compact))));
	return true;
}

QByteArray WebGateway::robotInstancesJsonOnGuiThread()
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), true);
	QJsonArray arr;
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	if (host)
	{
		if (cloudsim::host::HeadlessRobotContext* hrc = host->headlessRobotContext())
		{
			for (const auto& info : hrc->listInstances())
			{
				QJsonObject o;
				o.insert(QStringLiteral("sceneRootBackendId"), info.sceneRootBackendId);
				o.insert(QStringLiteral("label"), info.label);
				o.insert(QStringLiteral("jointCount"), info.jointCount);
				o.insert(QStringLiteral("urdfPath"), info.urdfPath);
				arr.append(o);
			}
		}
	}
	root.insert(QStringLiteral("instances"), arr);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

QByteArray WebGateway::robotJointsMetaJsonOnGuiThread(const QString& sceneRootBackendId)
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), false);
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	if (!host)
	{
		root.insert(QStringLiteral("error"), QStringLiteral("no document"));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	cloudsim::host::HeadlessRobotContext* hrc = host->headlessRobotContext();
	if (!hrc)
	{
		root.insert(QStringLiteral("error"), QStringLiteral("no headless robot context"));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	QString rootId = sceneRootBackendId.trimmed();
	if (rootId.isEmpty())
	{
		const auto instances = hrc->listInstances();
		if (!instances.isEmpty())
			rootId = instances.first().sceneRootBackendId;
	}
	QStringList names;
	QVector<double> lower;
	QVector<double> upper;
	QVector<double> angles;
	if (!hrc->jointMetaForSceneRoot(rootId, names, lower, upper, angles))
	{
		root.insert(QStringLiteral("error"), QStringLiteral("unknown sceneRootBackendId"));
		root.insert(QStringLiteral("sceneRootBackendId"), rootId);
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	QJsonArray joints;
	for (int i = 0; i < names.size(); ++i)
	{
		QJsonObject j;
		j.insert(QStringLiteral("name"), names[i]);
		j.insert(QStringLiteral("lowerRad"), i < lower.size() ? lower[i] : -3.141592653589793);
		j.insert(QStringLiteral("upperRad"), i < upper.size() ? upper[i] : 3.141592653589793);
		j.insert(QStringLiteral("angleRad"), i < angles.size() ? angles[i] : 0.0);
		joints.append(j);
	}
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("sceneRootBackendId"), rootId);
	root.insert(QStringLiteral("joints"), joints);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

QByteArray WebGateway::robotResolveJsonOnGuiThread(const QString& backendId)
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("isRobot"), false);
	root.insert(QStringLiteral("backendId"), backendId);
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	if (!host || !host->headlessRobotContext())
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	cloudsim::host::HeadlessRobotContext* hrc = host->headlessRobotContext();
	bool isSceneRoot = false;
	const int idx = hrc->robotInstanceIndexForBackendId(backendId, &isSceneRoot);
	if (idx < 0)
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	const auto instances = hrc->listInstances();
	if (idx >= instances.size())
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	root.insert(QStringLiteral("isRobot"), true);
	root.insert(QStringLiteral("isSceneRoot"), isSceneRoot);
	root.insert(QStringLiteral("sceneRootBackendId"), instances[idx].sceneRootBackendId);
	root.insert(QStringLiteral("anchorBackendId"), hrc->robotGizmoAnchorBackendId(backendId));
	root.insert(QStringLiteral("flangeBackendId"), hrc->robotFlangeBackendId(backendId));
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::placeRobotOnGuiThread(const QByteArray& body, QString* err)
{
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	if (!host || !host->headlessRobotContext())
	{
		if (err)
			*err = QStringLiteral("no headless robot context");
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
	const QString anchorId = o.value(QStringLiteral("anchorBackendId")).toString();
	const QJsonArray wmArr = o.value(QStringLiteral("worldMatrix")).toArray();
	if (anchorId.isEmpty() || wmArr.size() < 16)
	{
		if (err)
			*err = QStringLiteral("anchorBackendId and worldMatrix[16] required");
		return false;
	}
	QVector<double> m16;
	m16.reserve(16);
	for (int i = 0; i < 16; ++i)
		m16.append(wmArr.at(i).toDouble());
	QString placeErr;
	if (!host->headlessRobotContext()->applyFkFromGizmoAnchorThreeJsMatrix(anchorId, m16, &placeErr))
	{
		if (err)
			*err = placeErr.isEmpty() ? QStringLiteral("place failed") : placeErr;
		return false;
	}
	pushEvent(QStringLiteral("{\"type\":\"RobotKinematicsApplied\",\"sceneRootBackendId\":\"%1\"}").arg(anchorId));
	return true;
}

bool WebGateway::tcpIkRobotOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out)
{
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	if (!host || !host->headlessRobotContext())
	{
		if (err)
			*err = QStringLiteral("no headless robot context");
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
	QString flangeId = o.value(QStringLiteral("flangeBackendId")).toString();
	if (flangeId.isEmpty())
		flangeId = o.value(QStringLiteral("anchorBackendId")).toString();
	const QJsonArray wmArr = o.value(QStringLiteral("worldMatrix")).toArray();
	if (flangeId.isEmpty() || wmArr.size() < 16)
	{
		if (err)
			*err = QStringLiteral("flangeBackendId and worldMatrix[16] required");
		return false;
	}
	QVector<double> m16;
	m16.reserve(16);
	for (int i = 0; i < 16; ++i)
		m16.append(wmArr.at(i).toDouble());
	const bool translateOnly = o.value(QStringLiteral("translateOnly")).toBool(false);
	QVector<double> joints;
	QString ikErr;
	bool incomplete = false;
	if (!host->headlessRobotContext()->applyIkFromFlangeThreeJsMatrix(flangeId, m16, &joints, &ikErr, &incomplete,
																	  translateOnly))
	{
		if (err)
			*err = ikErr.isEmpty() ? QStringLiteral("tcp IK failed") : ikErr;
		return false;
	}
	const int instIdx = host->headlessRobotContext()->robotInstanceIndexForBackendId(flangeId);
	QString rootId = flangeId;
	if (instIdx >= 0)
	{
		const auto instances = host->headlessRobotContext()->listInstances();
		if (instIdx < instances.size())
			rootId = instances[instIdx].sceneRootBackendId;
	}
	if (out)
	{
		QJsonArray ja;
		for (double d : joints)
			ja.append(d);
		(*out)[QStringLiteral("sceneRootBackendId")] = rootId;
		(*out)[QStringLiteral("jointAnglesRad")] = ja;
		(*out)[QStringLiteral("incomplete")] = incomplete;
	}
	QJsonArray a;
	for (double d : joints)
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
	instr.taughtJointRadCsv = o.value(QStringLiteral("taughtJointRadCsv")).toString(); // 废弃字段，兼容旧 JSON
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

	const QString seedPolicyRaw = o.value(QStringLiteral("seedPolicy")).toString();
	const QString seedInstructionId = o.value(QStringLiteral("seedInstructionId")).toString().trimmed();
	// Chain/Current 为网页别名，对齐桌面 IkSeedPolicy
	const bool fromCurrentPose = seedPolicyRaw.compare(QStringLiteral("FromCurrentPose"), Qt::CaseInsensitive) == 0 ||
								 seedPolicyRaw.compare(QStringLiteral("Current"), Qt::CaseInsensitive) == 0;
	const bool fromInstruction = seedPolicyRaw.isEmpty() ||
								 seedPolicyRaw.compare(QStringLiteral("FromInstruction"), Qt::CaseInsensitive) == 0 ||
								 seedPolicyRaw.compare(QStringLiteral("Chain"), Qt::CaseInsensitive) == 0;
	if (!seedPolicyRaw.isEmpty() && !fromCurrentPose && !fromInstruction)
	{
		if (err)
			*err = QStringLiteral("invalid seedPolicy; expected FromInstruction|Chain|FromCurrentPose|Current");
		return false;
	}
	if (!seedPolicyRaw.isEmpty())
		ctx.extensions.insert(QStringLiteral("seedPolicy"), seedPolicyRaw);
	if (!seedInstructionId.isEmpty())
		ctx.extensions.insert(QStringLiteral("seedInstructionId"), seedInstructionId);

	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto fillLiveSeed = [&]() -> bool
	{
		if (!host || !host->headlessRobotContext())
			return false;
		QStringList names;
		QVector<double> lo, hi, ang;
		if (!host->headlessRobotContext()->jointMetaForSceneRoot(sceneRoot, names, lo, hi, ang) || ang.isEmpty())
			return false;
		ctx.seedJointRad = ang;
		return true;
	};

	if (fromCurrentPose)
	{
		// 当前位姿优先：忽略 jointRadCsv 作主种子
		if (!fillLiveSeed())
		{
			if (err)
				*err = QStringLiteral("current pose seed unavailable");
			return false;
		}
	}
	else if (!seedInstructionId.isEmpty())
	{
		if (!host)
		{
			if (err)
				*err = QStringLiteral("no document host");
			return false;
		}
		const auto seedIns = findSeedInstructionApi(host->robotProgramStore(), seedInstructionId);
		if (!seedIns)
		{
			if (err)
				*err = QStringLiteral("seed instruction not found: %1").arg(seedInstructionId);
			return false;
		}
		// 无会话 PlanResult 缓存时仅能读临时扩展；缺失则硬失败，禁止静默降级
		const auto& ext = seedIns->extensionProperties();
		const auto csvIt = ext.find("context.currentJointRadCsv");
		if (csvIt == ext.end() || csvIt->second.empty())
		{
			if (err)
				*err = QStringLiteral(
					"seed instruction has no planned joints yet; plan the referenced waypoint first");
			return false;
		}
		const QStringList parts =
			QString::fromStdString(csvIt->second).split(QLatin1Char(','), Qt::SkipEmptyParts);
		ctx.seedJointRad.clear();
		ctx.seedJointRad.reserve(parts.size());
		for (const QString& p : parts)
			ctx.seedJointRad.append(p.trimmed().toDouble());
		if (ctx.seedJointRad.isEmpty())
		{
			if (err)
				*err = QStringLiteral(
					"seed instruction has no planned joints yet; plan the referenced waypoint first");
			return false;
		}
	}
	else if (!instr.jointRadCsv.isEmpty())
	{
		// jointRadCsv = 链式种子；示教目标走 taughtJointRadCsv，勿混用
		const QStringList parts = instr.jointRadCsv.split(QLatin1Char(','), Qt::SkipEmptyParts);
		ctx.seedJointRad.reserve(parts.size());
		for (const QString& p : parts)
			ctx.seedJointRad.append(p.trimmed().toDouble());
	}
	else if (o.contains(QStringLiteral("seedJointRad")))
	{
		for (const auto& v : o.value(QStringLiteral("seedJointRad")).toArray())
			ctx.seedJointRad.append(v.toDouble());
	}
	else if (!fillLiveSeed())
	{
		// 无显式种子时回退当前关节
	}

	cloudsim::core::PlanResultDto result;
	if (!m_document->robot().planInstruction(instr, ctx, result, err))
		return false;
	if (out)
	{
		(*out)[QStringLiteral("ok")] = result.ok;
		(*out)[QStringLiteral("error")] = result.error;
		(*out)[QStringLiteral("durationSec")] = result.durationSec;
		QJsonArray joints;
		for (double d : result.jointTargetsRad)
			joints.append(d);
		(*out)[QStringLiteral("jointTargetsRad")] = joints;
		QJsonArray traj;
		for (const QVector<double>& sample : result.jointTrajectoryRad)
		{
			QJsonArray row;
			for (double d : sample)
				row.append(d);
			traj.append(row);
		}
		(*out)[QStringLiteral("jointTrajectoryRad")] = traj;
	}
	return result.ok;
}

QByteArray WebGateway::robotTcpPoseJsonOnGuiThread(const QString& sceneRootBackendId)
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), false);
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	if (!host || !host->headlessRobotContext())
	{
		root.insert(QStringLiteral("error"), QStringLiteral("no headless robot context"));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	QString rootId = sceneRootBackendId.trimmed();
	if (rootId.isEmpty())
	{
		const auto instances = host->headlessRobotContext()->listInstances();
		if (!instances.isEmpty())
			rootId = instances.first().sceneRootBackendId;
	}
	cloudsim::host::HeadlessRobotContext::TcpPoseCapture pose;
	QString err;
	if (!host->headlessRobotContext()->captureTcpPose(rootId, pose, &err))
	{
		root.insert(QStringLiteral("error"), err.isEmpty() ? QStringLiteral("capture failed") : err);
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("sceneRootBackendId"), rootId);
	root.insert(QStringLiteral("flangeLinkName"), pose.flangeLinkName);
	root.insert(QStringLiteral("jointRadCsv"), pose.jointRadCsv);
	root.insert(QStringLiteral("positionMm"),
				QJsonArray{pose.positionMm[0], pose.positionMm[1], pose.positionMm[2]});
	root.insert(QStringLiteral("eulerDeg"), QJsonArray{pose.eulerDeg[0], pose.eulerDeg[1], pose.eulerDeg[2]});
	QJsonArray mat;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
			mat.append(pose.worldMat.v[r * 4 + c]);
	}
	root.insert(QStringLiteral("worldMatrix"), mat);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

namespace
{
QString resolveSceneRootId(cloudsim::host::HeadlessRobotContext* hrc, const QString& sceneRootBackendId)
{
	QString rootId = sceneRootBackendId.trimmed();
	if (rootId.isEmpty() && hrc)
	{
		const auto instances = hrc->listInstances();
		if (!instances.isEmpty())
			rootId = instances.first().sceneRootBackendId;
	}
	return rootId;
}

QJsonObject overlayEntryToJson(const cloudsim::host::FrameOverlayEntry& e)
{
	QJsonArray mat;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
			mat.append(e.worldMat.v[r * 4 + c]);
	}
	return QJsonObject{{QStringLiteral("id"), e.id},
					   {QStringLiteral("name"), e.name},
					   {QStringLiteral("active"), e.active},
					   {QStringLiteral("positionMm"), QJsonArray{e.positionMm[0], e.positionMm[1], e.positionMm[2]}},
					   {QStringLiteral("eulerDeg"), QJsonArray{e.eulerDeg[0], e.eulerDeg[1], e.eulerDeg[2]}},
					   {QStringLiteral("worldMatrix"), mat}};
}
} // namespace

QByteArray WebGateway::robotFramesJsonOnGuiThread(const QString& sceneRootBackendId)
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), false);
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		root.insert(QStringLiteral("error"), QStringLiteral("no headless robot context"));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	const QString rootId = resolveSceneRootId(hrc, sceneRootBackendId);
	const int idx = hrc->robotInstanceIndexForSceneBackendId(rootId);
	if (idx < 0)
	{
		root.insert(QStringLiteral("error"), QStringLiteral("unknown sceneRootBackendId"));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = hrc->robotCoordinateFramesForInstance(idx);
	QJsonArray linkNames;
	cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
	if (hrc->robotPerLinkKinematicsForInstance(idx, pl))
	{
		QStringList keys = pl.linkNameToBackendId.keys();
		keys.sort();
		for (const QString& k : keys)
			linkNames.append(k);
	}
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("sceneRootBackendId"), rootId);
	root.insert(QStringLiteral("frames"), cloudsim::host::coordinateFrameSetToQJson(frames));
	root.insert(QStringLiteral("linkNames"), linkNames);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::putRobotFramesOnGuiThread(const QByteArray& body, QString* err)
{
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		if (err)
			*err = QStringLiteral("no headless robot context");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject())
	{
		if (err)
			*err = QStringLiteral("invalid json");
		return false;
	}
	const QJsonObject o = doc.object();
	const QString rootId = resolveSceneRootId(hrc, o.value(QStringLiteral("sceneRootBackendId")).toString());
	const int idx = hrc->robotInstanceIndexForSceneBackendId(rootId);
	if (idx < 0)
	{
		if (err)
			*err = QStringLiteral("unknown sceneRootBackendId");
		return false;
	}
	RobotCoordinate::RobotCoordinateFrameSet newFrames;
	const QJsonObject framesObj = o.value(QStringLiteral("frames")).toObject();
	if (framesObj.isEmpty() || !cloudsim::host::coordinateFrameSetFromQJson(framesObj, newFrames))
	{
		if (err)
			*err = QStringLiteral("invalid frames");
		return false;
	}
	RobotCoordinate::RobotCoordinateFrameSet& cur = hrc->robotCoordinateFramesForInstance(idx);
	const RobotCoordinate::RobotCoordinateFrameSet oldFrames = cur;
	cur = newFrames;
	cloudsim::host::syncProgramToolContextAfterFrameChange(host->robotProgramStore(), rootId, oldFrames, cur);
	pushEvent(QStringLiteral("{\"type\":\"RobotCoordinateFramesChanged\",\"sceneRootBackendId\":\"%1\"}")
				  .arg(jsonEscapeApi(rootId)));
	return true;
}

QByteArray WebGateway::robotExternalAxesJsonOnGuiThread(const QString& sceneRootBackendId)
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), false);
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		root.insert(QStringLiteral("error"), QStringLiteral("no headless robot context"));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	const QString rootId = resolveSceneRootId(hrc, sceneRootBackendId);
	QJsonObject ea;
	QString err;
	if (!hrc->getExternalAxesJson(rootId, ea, &err))
	{
		root.insert(QStringLiteral("error"), err.isEmpty() ? QStringLiteral("get externalAxes failed") : err);
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("sceneRootBackendId"), rootId);
	root.insert(QStringLiteral("externalAxes"), ea);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::putRobotExternalAxesOnGuiThread(const QByteArray& body, QString* err)
{
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		if (err)
			*err = QStringLiteral("no headless robot context");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	if (!doc.isObject())
	{
		if (err)
			*err = QStringLiteral("invalid json");
		return false;
	}
	const QJsonObject o = doc.object();
	const QString rootId = resolveSceneRootId(hrc, o.value(QStringLiteral("sceneRootBackendId")).toString());
	if (!hrc->setExternalAxesJson(rootId, o, err))
		return false;
	pushEvent(QStringLiteral("{\"type\":\"RobotExternalAxesChanged\",\"sceneRootBackendId\":\"%1\"}")
				  .arg(jsonEscapeApi(rootId)));
	return true;
}

bool WebGateway::mutateRobotFramesOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out)
{
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		if (err)
			*err = QStringLiteral("no headless robot context");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	const QJsonObject o = doc.isObject() ? doc.object() : QJsonObject{};
	const QString rootId = resolveSceneRootId(hrc, o.value(QStringLiteral("sceneRootBackendId")).toString());
	const int idx = hrc->robotInstanceIndexForSceneBackendId(rootId);
	if (idx < 0)
	{
		if (err)
			*err = QStringLiteral("unknown sceneRootBackendId");
		return false;
	}
	const QString action = o.value(QStringLiteral("action")).toString();
	const QString sourceId = o.value(QStringLiteral("id")).toString();
	RobotCoordinate::RobotCoordinateFrameSet& frames = hrc->robotCoordinateFramesForInstance(idx);
	const RobotCoordinate::RobotCoordinateFrameSet oldFrames = frames;
	std::string selectedId;
	if (action == QLatin1String("addTool"))
	{
		selectedId = cloudsim::host::addToolFrame(frames);
	}
	else if (action == QLatin1String("addUser"))
	{
		selectedId = cloudsim::host::addUserFrame(frames);
	}
	else if (action == QLatin1String("duplicateTool"))
	{
		selectedId = cloudsim::host::duplicateToolFrame(frames, sourceId.toStdString());
		if (selectedId.empty())
		{
			if (err)
				*err = QStringLiteral("tool frame not found");
			return false;
		}
	}
	else if (action == QLatin1String("duplicateUser"))
	{
		selectedId = cloudsim::host::duplicateUserFrame(frames, sourceId.toStdString());
		if (selectedId.empty())
		{
			if (err)
				*err = QStringLiteral("user frame not found");
			return false;
		}
	}
	else
	{
		if (err)
			*err = QStringLiteral("unknown action");
		return false;
	}
	cloudsim::host::syncProgramToolContextAfterFrameChange(host->robotProgramStore(), rootId, oldFrames, frames);
	pushEvent(QStringLiteral("{\"type\":\"RobotCoordinateFramesChanged\",\"sceneRootBackendId\":\"%1\"}")
				  .arg(jsonEscapeApi(rootId)));
	if (out)
	{
		QJsonArray linkNames;
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (hrc->robotPerLinkKinematicsForInstance(idx, pl))
		{
			QStringList keys = pl.linkNameToBackendId.keys();
			keys.sort();
			for (const QString& k : keys)
				linkNames.append(k);
		}
		out->insert(QStringLiteral("sceneRootBackendId"), rootId);
		out->insert(QStringLiteral("frames"), cloudsim::host::coordinateFrameSetToQJson(frames));
		out->insert(QStringLiteral("linkNames"), linkNames);
		out->insert(QStringLiteral("selectedId"), QString::fromStdString(selectedId));
		out->insert(QStringLiteral("kind"),
					action.startsWith(QLatin1String("addTool")) || action.startsWith(QLatin1String("duplicateTool"))
						? QStringLiteral("tool")
						: QStringLiteral("user"));
	}
	return true;
}

bool WebGateway::captureRobotToolFrameOnGuiThread(const QByteArray& body, QString* err)
{
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		if (err)
			*err = QStringLiteral("no headless robot context");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	const QJsonObject o = doc.isObject() ? doc.object() : QJsonObject{};
	const QString rootId = resolveSceneRootId(hrc, o.value(QStringLiteral("sceneRootBackendId")).toString());
	const int idx = hrc->robotInstanceIndexForSceneBackendId(rootId);
	if (idx < 0)
	{
		if (err)
			*err = QStringLiteral("unknown sceneRootBackendId");
		return false;
	}
	cloudsim::host::HeadlessRobotContext::TcpPoseCapture pose;
	if (!hrc->captureTcpPose(rootId, pose, err))
		return false;
	const BackendMat4 T_base_tcp = RobotCoordinate::tcpInBaseFromPose(
		pose.positionMm[0], pose.positionMm[1], pose.positionMm[2], pose.eulerDeg[0], pose.eulerDeg[1], pose.eulerDeg[2]);
	RobotCoordinate::RobotCoordinateFrameSet& frames = hrc->robotCoordinateFramesForInstance(idx);
	const RobotCoordinate::RobotCoordinateFrameSet oldFrames = frames;
	QString flangeLink = pose.flangeLinkName;
	if (const RobotCoordinate::RobotToolFrame* active = RobotCoordinate::activeToolFrame(frames))
	{
		const QString eff = QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *active));
		if (!eff.isEmpty())
			flangeLink = eff;
	}
	QStringList names;
	QVector<double> lo, hi, angles;
	(void)hrc->jointMetaForSceneRoot(rootId, names, lo, hi, angles);
	if (!cloudsim::host::captureToolFrameFromTcpPose(hrc->robotUrdfAbsolutePathForInstance(idx), angles, flangeLink,
													 T_base_tcp, frames, err))
		return false;
	cloudsim::host::syncProgramToolContextAfterFrameChange(host->robotProgramStore(), rootId, oldFrames, frames);
	pushEvent(QStringLiteral("{\"type\":\"RobotCoordinateFramesChanged\",\"sceneRootBackendId\":\"%1\"}")
				  .arg(jsonEscapeApi(rootId)));
	return true;
}

bool WebGateway::captureRobotUserFrameOnGuiThread(const QByteArray& body, QString* err)
{
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		if (err)
			*err = QStringLiteral("no headless robot context");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	const QJsonObject o = doc.isObject() ? doc.object() : QJsonObject{};
	const QString rootId = resolveSceneRootId(hrc, o.value(QStringLiteral("sceneRootBackendId")).toString());
	const int idx = hrc->robotInstanceIndexForSceneBackendId(rootId);
	if (idx < 0)
	{
		if (err)
			*err = QStringLiteral("unknown sceneRootBackendId");
		return false;
	}
	cloudsim::host::HeadlessRobotContext::TcpPoseCapture pose;
	if (!hrc->captureTcpPose(rootId, pose, err))
		return false;
	RobotCoordinate::RobotCoordinateFrameSet& frames = hrc->robotCoordinateFramesForInstance(idx);
	const RobotCoordinate::RobotCoordinateFrameSet oldFrames = frames;
	if (!cloudsim::host::captureUserFrameFromTcpPose(pose.positionMm[0], pose.positionMm[1], pose.positionMm[2],
													 pose.eulerDeg[0], pose.eulerDeg[1], pose.eulerDeg[2], frames, err))
		return false;
	cloudsim::host::syncProgramToolContextAfterFrameChange(host->robotProgramStore(), rootId, oldFrames, frames);
	pushEvent(QStringLiteral("{\"type\":\"RobotCoordinateFramesChanged\",\"sceneRootBackendId\":\"%1\"}")
				  .arg(jsonEscapeApi(rootId)));
	return true;
}

bool WebGateway::resetRobotToolFrameOnGuiThread(const QByteArray& body, QString* err)
{
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		if (err)
			*err = QStringLiteral("no headless robot context");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	const QJsonObject o = doc.isObject() ? doc.object() : QJsonObject{};
	const QString rootId = resolveSceneRootId(hrc, o.value(QStringLiteral("sceneRootBackendId")).toString());
	const int idx = hrc->robotInstanceIndexForSceneBackendId(rootId);
	if (idx < 0)
	{
		if (err)
			*err = QStringLiteral("unknown sceneRootBackendId");
		return false;
	}
	RobotCoordinate::RobotCoordinateFrameSet& frames = hrc->robotCoordinateFramesForInstance(idx);
	const RobotCoordinate::RobotCoordinateFrameSet oldFrames = frames;
	cloudsim::host::resetActiveToolFrame(frames);
	cloudsim::host::syncProgramToolContextAfterFrameChange(host->robotProgramStore(), rootId, oldFrames, frames);
	pushEvent(QStringLiteral("{\"type\":\"RobotCoordinateFramesChanged\",\"sceneRootBackendId\":\"%1\"}")
				  .arg(jsonEscapeApi(rootId)));
	return true;
}

QByteArray WebGateway::robotFrameOverlaysJsonOnGuiThread(const QString& sceneRootBackendId)
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), false);
	auto* host = cloudsim::host::documentHostFromScope(m_document.get());
	auto* hrc = host ? host->headlessRobotContext() : nullptr;
	if (!hrc)
	{
		root.insert(QStringLiteral("error"), QStringLiteral("no headless robot context"));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	const QString rootId = resolveSceneRootId(hrc, sceneRootBackendId);
	cloudsim::host::FrameOverlaySnapshot snap;
	QString err;
	if (!cloudsim::host::buildFrameOverlaySnapshot(*hrc, host->backend(), rootId, snap, &err))
	{
		root.insert(QStringLiteral("error"), err.isEmpty() ? QStringLiteral("overlay failed") : err);
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	QJsonArray tools;
	for (const auto& e : snap.tools)
		tools.append(overlayEntryToJson(e));
	QJsonArray users;
	for (const auto& e : snap.users)
		users.append(overlayEntryToJson(e));
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("sceneRootBackendId"), rootId);
	root.insert(QStringLiteral("tools"), tools);
	root.insert(QStringLiteral("users"), users);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

QByteArray WebGateway::instructionPropertiesJsonOnGuiThread(const QString& instructionId)
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), false);
	if (!m_document || instructionId.isEmpty())
	{
		root.insert(QStringLiteral("error"), QStringLiteral("missing instructionId"));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	const auto rows = m_document->robot().instructionPropertyRows(instructionId);
	QJsonArray arr;
	for (const auto& r : rows)
	{
		QJsonObject o;
		o.insert(QStringLiteral("key"), r.key);
		o.insert(QStringLiteral("label"), r.labelEn.isEmpty() ? r.key : r.labelEn);
		o.insert(QStringLiteral("value"), r.value);
		o.insert(QStringLiteral("editable"), r.editable);
		arr.append(o);
	}
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("instructionId"), instructionId);
	root.insert(QStringLiteral("properties"), arr);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::patchInstructionPropertyOnGuiThread(const QString& instructionId, const QByteArray& body,
													 QString* err)
{
	if (!m_document || instructionId.isEmpty())
	{
		if (err)
			*err = QStringLiteral("missing instructionId");
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
	const QString key = o.value(QStringLiteral("key")).toString();
	const QString value = o.value(QStringLiteral("value")).toVariant().toString();
	if (key.isEmpty())
	{
		if (err)
			*err = QStringLiteral("key required");
		return false;
	}
	return m_document->robot().applyInstructionPropertyChange(instructionId, key, value, err);
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
		"{\"ok\":true,\"endpoint\":\"/api/ai/chat\",\"note\":\"Configure ai_config.json beside CloudSimWeb.exe\"}");
}

QByteArray WebGateway::ioSignalsJsonOnGuiThread(cloudsim::host::DocumentHost* host)
{
	QJsonObject root;
	if (!host)
	{
		root.insert(QStringLiteral("ok"), false);
		root.insert(QStringLiteral("error"), QStringLiteral("No host."));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	const QString oid = host->ioSignalNetwork().primaryRobotOwnerId();
	root = host->ioSignalNetwork().ownerSignalsPayloadWithRuntime(oid);
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("ownerId"), oid);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::ioSignalsPutOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	QJsonObject obj = doc.object();
	if (doc.isArray())
		obj = QJsonObject{{QStringLiteral("signals"), doc.array()}};
	if (!namedSignalTableFromQJson(host->namedSignalTable(), obj, err))
		return false;
	host->ioSignalNetwork().resetRuntime(host->ioSignalNetwork().primaryRobotOwnerId(), false);
	pushEvent(QStringLiteral("{\"type\":\"IoSignalsChanged\"}"));
	pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	return true;
}

QByteArray WebGateway::ioSignalNamesJsonOnGuiThread(cloudsim::host::DocumentHost* host, const QString& kindFilter)
{
	QJsonObject root{{QStringLiteral("ok"), true}};
	QJsonArray names;
	names.append(QString());
	if (host)
	{
		RobotIo::SignalKind want = RobotIo::SignalKind::DI;
		const bool filter = RobotIo::NamedSignalTable::kindFromString(kindFilter.toStdString(), want);
		for (const RobotIo::SignalDef& s : host->namedSignalTable().entries())
		{
			if (filter && s.kind != want)
				continue;
			if (!s.name.empty())
				names.append(QString::fromStdString(s.name));
		}
	}
	root.insert(QStringLiteral("names"), names);
	root.insert(QStringLiteral("kind"), kindFilter);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::ioSignalRuntimePatchOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body,
												 QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	QString ownerId = o.value(QStringLiteral("ownerId")).toString();
	if (ownerId.isEmpty())
		ownerId = host->ioSignalNetwork().primaryRobotOwnerId();
	if (!host->ioSignalNetwork().setRuntime(ownerId, o.value(QStringLiteral("kind")).toString(),
											o.value(QStringLiteral("port")).toInt(),
											o.value(QStringLiteral("value")).toString().trimmed(),
											o.contains(QStringLiteral("forced")), o.value(QStringLiteral("forced")).toBool(),
											err))
		return false;
	pushEvent(QStringLiteral("{\"type\":\"IoSignalsChanged\",\"ownerId\":\"%1\"}").arg(ownerId));
	pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	// DO 可能经接线触发设备姿态；插值过程中也会继续 visualSceneDirty
	if (o.value(QStringLiteral("kind")).toString() == QLatin1String("DO"))
		pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
	return true;
}

bool WebGateway::ioSignalRuntimeResetOnGuiThread(cloudsim::host::DocumentHost* host, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	// 先清边沿再复位 runtime，避免复位过程中的采样被随后 clear 抹掉
	cloudsim::host::clearCustomDevicePoseEdgeMemory(host);
	host->ioSignalNetwork().resetRuntime(QString(), false);
	cloudsim::host::primeCustomDevicePoseEdgeMemory(host->ioSignalNetwork());
	pushEvent(QStringLiteral("{\"type\":\"IoSignalsChanged\"}"));
	pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	return true;
}

QByteArray WebGateway::ioNetworkJsonOnGuiThread(cloudsim::host::DocumentHost* host)
{
	QJsonObject root;
	if (!host)
	{
		root.insert(QStringLiteral("ok"), false);
		root.insert(QStringLiteral("error"), QStringLiteral("No host."));
		return QJsonDocument(root).toJson(QJsonDocument::Compact);
	}
	host->ioSignalNetwork().syncOwnersFromDocument(*host);
	root = host->ioSignalNetwork().networkPayloadWithRuntime();
	root.insert(QStringLiteral("ok"), true);
	return QJsonDocument(root).toJson(QJsonDocument::Compact);
}

bool WebGateway::ioNetworkOwnerSignalsPutOnGuiThread(cloudsim::host::DocumentHost* host, const QString& ownerId,
													 const QByteArray& body, QString* err)
{
	if (!host || ownerId.isEmpty())
	{
		if (err)
			*err = QStringLiteral("No host/owner.");
		return false;
	}
	RobotIo::NamedSignalTable* table = host->ioSignalNetwork().table(ownerId);
	if (!table)
	{
		if (err)
			*err = QStringLiteral("unknown owner");
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(body);
	QJsonObject obj = doc.object();
	if (doc.isArray())
		obj = QJsonObject{{QStringLiteral("signals"), doc.array()}};
	if (!namedSignalTableFromQJson(*table, obj, err))
		return false;
	host->ioSignalNetwork().resetRuntime(ownerId, false);
	host->ioSignalNetwork().flushDeviceTablesToDocument(*host);
	pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	pushEvent(QStringLiteral("{\"type\":\"IoSignalsChanged\",\"ownerId\":\"%1\"}").arg(ownerId));
	return true;
}

bool WebGateway::ioNetworkWirePostOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	cloudsim::host::IoSignalWire w;
	w.id = o.value(QStringLiteral("id")).toString();
	w.fromOwnerId = o.value(QStringLiteral("fromOwnerId")).toString();
	w.fromSignal = o.value(QStringLiteral("fromSignal")).toString();
	w.toOwnerId = o.value(QStringLiteral("toOwnerId")).toString();
	w.toSignal = o.value(QStringLiteral("toSignal")).toString();
	if (!host->ioSignalNetwork().addWire(w, err))
		return false;
	pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	return true;
}

bool WebGateway::ioNetworkWireDeleteOnGuiThread(cloudsim::host::DocumentHost* host, const QString& wireId, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	if (!host->ioSignalNetwork().removeWire(wireId))
	{
		if (err)
			*err = QStringLiteral("wire not found");
		return false;
	}
	pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	return true;
}

bool WebGateway::ioNetworkOwnerLayoutPatchOnGuiThread(cloudsim::host::DocumentHost* host, const QString& ownerId,
													  const QByteArray& body, QString* err)
{
	if (!host || ownerId.isEmpty())
	{
		if (err)
			*err = QStringLiteral("No host/owner.");
		return false;
	}
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	host->ioSignalNetwork().setCanvasPos(ownerId, o.value(QStringLiteral("canvasX")).toDouble(),
										 o.value(QStringLiteral("canvasY")).toDouble());
	pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	return true;
}

bool WebGateway::ioNetworkRuntimePatchOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body,
												  QString* err)
{
	return ioSignalRuntimePatchOnGuiThread(host, body, err);
}

bool WebGateway::ioNetworkRuntimeResetOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body,
												  QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	host->ioSignalNetwork().resetRuntime(o.value(QStringLiteral("ownerId")).toString(), false);
	pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	pushEvent(QStringLiteral("{\"type\":\"IoSignalsChanged\"}"));
	return true;
}

QByteArray WebGateway::customDevicesListJsonOnGuiThread(cloudsim::host::DocumentHost* host)
{
	if (!host)
		return QByteArrayLiteral("{\"ok\":false,\"error\":\"No host.\"}");
	return QJsonDocument(cloudsim::host::listCustomDevicesJson(*host)).toJson(QJsonDocument::Compact);
}

QByteArray WebGateway::customDeviceDetailJsonOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id)
{
	if (!host)
		return QByteArrayLiteral("{\"ok\":false,\"error\":\"No host.\"}");
	return QJsonDocument(cloudsim::host::customDeviceDetailJson(*host, id)).toJson(QJsonDocument::Compact);
}

bool WebGateway::customDevicePutOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
											QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	return cloudsim::host::putCustomDeviceRuntimeFields(*host, id, QJsonDocument::fromJson(body).object(), err);
}

bool WebGateway::customDeviceApplyQOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id,
											   const QByteArray& body, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const bool ok = cloudsim::host::applyCustomDeviceQ(*host, id, QJsonDocument::fromJson(body).object(), err);
	if (ok)
		pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
	return ok;
}

bool WebGateway::customDeviceGotoPoseOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id,
												 const QByteArray& body, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const bool ok = cloudsim::host::gotoCustomDevicePose(*host, id, QJsonDocument::fromJson(body).object(), err);
	if (ok)
		pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
	return ok;
}

bool WebGateway::customDeviceAssemblyPostOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body,
													 QString* err, QString* outId)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const bool ok =
		cloudsim::host::commitCustomDeviceAssembly(*host, QJsonDocument::fromJson(body).object(), err, outId);
	if (ok)
	{
		pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
		pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	}
	return ok;
}

QByteArray WebGateway::customDeviceAssemblyCandidatesJsonOnGuiThread(cloudsim::host::DocumentHost* host)
{
	if (!host)
		return QByteArrayLiteral("{\"ok\":false,\"error\":\"No host.\"}");
	return QJsonDocument(cloudsim::host::listAssemblyGeometryCandidatesJson(*host)).toJson(QJsonDocument::Compact);
}

bool WebGateway::customDeviceEnsureOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err,
											   QString* outId)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const bool ok = cloudsim::host::ensureCustomDevice(*host, QJsonDocument::fromJson(body).object(), err, outId);
	if (ok)
	{
		pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
		pushEvent(QStringLiteral("{\"type\":\"IoNetworkChanged\"}"));
	}
	return ok;
}

bool WebGateway::customDeviceAttachOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id,
											   const QByteArray& body, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	const bool ok = cloudsim::host::attachCustomDeviceChildren(*host, id, o.value(QStringLiteral("childIds")).toArray(), err);
	if (ok)
		pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
	return ok;
}

bool WebGateway::customDeviceExportUrdfOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id,
												   const QByteArray& body, QString* err, QString* outDir)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	const QString parentDir = o.value(QStringLiteral("packageParentDir")).toString();
	if (parentDir.isEmpty())
	{
		if (err)
			*err = QStringLiteral("packageParentDir required");
		return false;
	}
	return cloudsim::host::exportCustomDeviceUrdfZip(*host, id, parentDir, err, outDir);
}

QByteArray WebGateway::robotsForMountJsonOnGuiThread(cloudsim::host::DocumentHost* host)
{
	if (!host)
	{
		return QByteArray(R"({"ok":false,"error":"No host."})");
	}
	return QJsonDocument(cloudsim::host::listRobotsForMountJson(*host)).toJson(QJsonDocument::Compact);
}

bool WebGateway::customDeviceMountOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id,
											  const QByteArray& body, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const QJsonObject o = QJsonDocument::fromJson(body).object();
	const bool ok = cloudsim::host::mountCustomDeviceToRobotFlange(*host, id, o, err);
	if (ok)
		pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
	return ok;
}

bool WebGateway::customDeviceUnmountOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, QString* err)
{
	if (!host)
	{
		if (err)
			*err = QStringLiteral("No host.");
		return false;
	}
	const bool ok = cloudsim::host::unmountCustomDeviceFromRobotFlange(*host, id, err);
	if (ok)
		pushEvent(QStringLiteral("{\"type\":\"SceneChanged\"}"));
	return ok;
}

} // namespace cloudsim::web
