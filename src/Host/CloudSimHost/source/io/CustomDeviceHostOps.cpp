/// @file CustomDeviceHostOps.cpp
/// @brief Web/Headless 自定义设备操作

#include "CustomDeviceHostOps.h"

#include "BackendFileImport.h"
#include "BackendSceneDocumentFacade.h"
#include "CustomDeviceRobotMountComponent.h"
#include "CustomDeviceRobotMountOps.h"
#include "BackendTypeIds.h"
#include "CoreTypes.h"
#include "CustomDeviceAssemblyCommit.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "CustomDevicePoseMotionHost.h"
#include "DocumentHost.h"
#include "FollowAttachmentComponent.h"
#include "HeadlessRobotContext.h"
#include "RobotSceneKinematics.h"
#include "visual/VisualAspect.h"
#include "IDataService.h"
#include "IoSignalNetwork.h"
#include "NamedSignalTable.h"

#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QUuid>

#include <json.hpp>

#include <queue>
#include <unordered_set>

#include <BackendDataManager.h>

namespace cloudsim::host
{
namespace
{
QHash<QString, bool>& edgeMemory()
{
	static QHash<QString, bool> s;
	return s;
}

IRobotBackendPoseSink* poseSinkOf(DocumentHost& host)
{
	if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		return hrc->urdfImportScenePoseSink();
	}
	return host.sceneFacade().poseSink();
}

bool readDeviceDi(IoSignalNetwork& network, const QString& deviceId, const QString& signalName, bool* out)
{
	const RobotIo::NamedSignalTable* table = network.table(deviceId);
	if (!table || !out)
		return false;
	const RobotIo::SignalDef* def = table->findByName(signalName.toStdString());
	if (!def || def->kind != RobotIo::SignalKind::DI)
		return false;
	const QJsonObject payload = network.ownerSignalsPayloadWithRuntime(deviceId);
	for (const QJsonValue& v : payload.value(QStringLiteral("signals")).toArray())
	{
		const QJsonObject o = v.toObject();
		if (o.value(QStringLiteral("name")).toString() != signalName)
			continue;
		*out = o.value(QStringLiteral("value")).toString() == QLatin1String("1");
		return true;
	}
	return false;
}

QJsonArray doublesToJson(const std::vector<double>& q)
{
	QJsonArray a;
	for (double v : q)
		a.append(v);
	return a;
}

std::vector<double> doublesFromJson(const QJsonArray& a)
{
	std::vector<double> q;
	q.reserve(static_cast<size_t>(a.size()));
	for (const QJsonValue& v : a)
		q.push_back(v.toDouble());
	return q;
}

QVector<double> qvectorFromJson(const QJsonArray& a)
{
	QVector<double> q;
	q.reserve(a.size());
	for (const QJsonValue& v : a)
		q.append(v.toDouble());
	return q;
}

/// 挂载前 FK，与桌面 mountDeviceToRobot 预同步一致
bool applyRobotFkBeforeDeviceMount(DocumentHost& host, const QString& robotSceneBackendId,
									const QVector<double>* jointAnglesOverride, QVector<double>& outLocalJointAngles)
{
	HeadlessRobotContext* hrc = host.headlessRobotContext();
	if (!hrc)
	{
		return false;
	}
	const int instIdx = hrc->robotInstanceIndexForSceneBackendId(robotSceneBackendId);
	if (instIdx < 0)
	{
		return false;
	}
	if (jointAnglesOverride && !jointAnglesOverride->isEmpty())
	{
		outLocalJointAngles = *jointAnglesOverride;
	}
	else
	{
		QStringList names;
		QVector<double> lower;
		QVector<double> upper;
		if (!hrc->jointMetaForSceneRoot(robotSceneBackendId, names, lower, upper, outLocalJointAngles))
		{
			return false;
		}
	}
	IRobotBackendPoseSink* sink = poseSinkOf(host);
	QVector<double> aggregated;
	if (!RobotSceneKinematics::applyJointAnglesForInstance(hrc, sink, instIdx, outLocalJointAngles, aggregated))
	{
		return false;
	}
	hrc->recordJointAnglesForSceneRoot(robotSceneBackendId, outLocalJointAngles);
	host.noteRobotLocalJointAnglesForSceneRoot(robotSceneBackendId, aggregated);
	return true;
}

void notifyRobotSceneAfterDeviceMountChange(DocumentHost& host)
{
	if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		hrc->notifyRobotKinematicsAppliedToScene();
	}
}

void registerCustomDeviceLinkGeometryOwnership(DocumentHost& host, const CustomDeviceBackendData& device)
{
	QMap<QString, QString>& types = host.backendSourceType();
	for (const CustomDeviceLink& L : device.links())
	{
		if (L.geometryBackendId.empty())
		{
			continue;
		}
		types[QString::fromStdString(L.geometryBackendId)] = QStringLiteral("CustomDeviceLink");
	}
}

void stripCustomDeviceLinkHierarchyFollow(DocumentHost& host, const CustomDeviceBackendData& device)
{
	bool changed = false;
	BackendDataManager& mgr = host.backend();
	for (const CustomDeviceLink& L : device.links())
	{
		if (L.geometryBackendId.empty())
		{
			continue;
		}
		const auto geom = mgr.getData(L.geometryBackendId);
		if (!geom || !geom->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
		{
			continue;
		}
		geom->removeComponent(FollowAttachmentComponent::typeKeyStatic());
		changed = true;
	}
	if (changed)
	{
		host.invalidateFollowReverseIndex();
	}
}
} // namespace

void registerAllCustomDeviceLinkGeometryOwnership(DocumentHost& host)
{
	for (const auto& obj : host.listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassCustomDevice)
		{
			continue;
		}
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(obj);
		if (!device || !device->usesLinkJointGraph())
		{
			continue;
		}
		registerCustomDeviceLinkGeometryOwnership(host, *device);
	}
}

void ensureCustomDeviceLinkKinematicsOwnership(DocumentHost& host, const std::string& deviceBackendId)
{
	if (deviceBackendId.empty())
	{
		return;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceBackendId));
	if (!device || !device->usesLinkJointGraph())
	{
		return;
	}
	registerCustomDeviceLinkGeometryOwnership(host, *device);
	stripCustomDeviceLinkHierarchyFollow(host, *device);
	host.stripKinematicsOwnedFollowAttachments();
}

void finalizeCustomDeviceLinkJointGraph(DocumentHost& host, const std::string& deviceBackendId)
{
	if (deviceBackendId.empty())
	{
		return;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceBackendId));
	if (!device || !device->usesLinkJointGraph())
	{
		return;
	}
	ensureCustomDeviceLinkKinematicsOwnership(host, deviceBackendId);
	CustomDeviceAssemblyCommit::refreshLinkRestPosesFromGeometry(*device, host.backend());
	CustomDeviceKinematics::rebakeRotateJointOriginsFromFrames(*device, &host.backend());
	(void)CustomDeviceKinematics::applyQ(*device, &host.backend(), poseSinkOf(host), nullptr);
	flushCustomDeviceLinkGeometryVisual(host, deviceBackendId);
	flushCustomDeviceMotionCenterFrameVisual(host, deviceBackendId);
}

void flushCustomDeviceLinkGeometryVisual(DocumentHost& host, const std::string& deviceBackendId)
{
	if (deviceBackendId.empty())
	{
		return;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceBackendId));
	if (!device || !device->usesLinkJointGraph())
	{
		return;
	}
	BackendDataManager& mgr = host.backend();
	std::vector<std::string> linkGeomIds;
	linkGeomIds.reserve(device->links().size());
	for (const CustomDeviceLink& L : device->links())
	{
		if (L.geometryBackendId.empty())
		{
			continue;
		}
		linkGeomIds.push_back(L.geometryBackendId);
	}
	if (linkGeomIds.empty())
	{
		return;
	}

	// Backend 已由 applyToSink 用 compound Δ 写好连杆及下挂非运动学子件，此处只刷 OSG
	std::unordered_set<std::string> flushed;
	std::queue<std::string> queue;
	for (const std::string& linkGeomId : linkGeomIds)
	{
		queue.push(linkGeomId);
	}
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		if (!flushed.insert(cur).second)
		{
			continue;
		}
		(void)host.syncOuterPatFromBackendId(cur);
		for (const std::string& child : mgr.childrenOf(cur))
		{
			queue.push(child);
		}
	}
}

void flushCustomDeviceMotionCenterFrameVisual(DocumentHost& host, const std::string& deviceBackendId)
{
	if (deviceBackendId.empty())
	{
		return;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceBackendId));
	if (!device || !device->usesLinkJointGraph())
	{
		return;
	}
	BackendDataManager& mgr = host.backend();
	std::unordered_set<std::string> synced;
	for (const CustomDeviceJoint& J : device->joints())
	{
		if (J.motion.motionType != CustomDeviceMotionType::Rotate)
		{
			continue;
		}
		const std::string& frameId = J.motion.motionCenterFrameBackendId;
		if (frameId.empty() || synced.count(frameId) != 0)
		{
			continue;
		}
		const auto frame = mgr.getData(frameId);
		if (!frame || !backend_type::isCoordinateFrameClassName(frame->className()))
		{
			continue;
		}
		(void)host.syncOuterPatFromBackendId(frameId);
		synced.insert(frameId);
	}
}

void syncCustomDeviceKinematicsAfterRootPoseChange(DocumentHost& host, const std::string& deviceBackendId)
{
	if (deviceBackendId.empty())
	{
		return;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceBackendId));
	if (!device || !device->usesLinkJointGraph())
	{
		return;
	}
	BackendDataManager& mgr = host.backend();
	(void)CustomDeviceKinematics::applyQ(*device, &mgr, poseSinkOf(host), nullptr);
	(void)host.syncOuterPatFromBackendId(deviceBackendId);
	flushCustomDeviceLinkGeometryVisual(host, deviceBackendId);
	flushCustomDeviceMotionCenterFrameVisual(host, deviceBackendId);
	host.markFollowAttachmentDirtyFromBackendMove(deviceBackendId);
}

QJsonObject listCustomDevicesJson(DocumentHost& host)
{
	QJsonArray arr;
	for (const auto& obj : host.listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassCustomDevice)
			continue;
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(obj);
		if (!device)
			continue;
		QJsonObject o;
		o.insert(QStringLiteral("id"), QString::fromStdString(device->id()));
		o.insert(QStringLiteral("name"), QString::fromStdString(device->name()));
		o.insert(QStringLiteral("axisCount"), static_cast<int>(device->axes().axes.size()));
		o.insert(QStringLiteral("jointCount"), static_cast<int>(device->joints().size()));
		o.insert(QStringLiteral("linkCount"), static_cast<int>(device->links().size()));
		arr.append(o);
	}
	return QJsonObject{{QStringLiteral("ok"), true}, {QStringLiteral("devices"), arr}};
}

QJsonObject customDeviceDetailJson(DocumentHost& host, const QString& deviceId)
{
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString()));
	if (!device)
		return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), QStringLiteral("not found")}};

	nlohmann::json posesJ = nlohmann::json::array();
	writeCustomDeviceNamedPosesToJson(device->namedPoses(), posesJ);
	nlohmann::json bindsJ = nlohmann::json::array();
	writeCustomDevicePoseSignalBindingsToJson(device->poseSignalBindings(), bindsJ);
	nlohmann::json linksJ = nlohmann::json::array();
	writeCustomDeviceLinksToJson(device->links(), linksJ);
	nlohmann::json jointsJ = nlohmann::json::array();
	writeCustomDeviceJointsToJson(device->joints(), jointsJ);

	auto toQ = [](const nlohmann::json& j) {
		return QJsonDocument::fromJson(QByteArray::fromStdString(j.dump())).array();
	};

	QJsonObject root;
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("id"), deviceId);
	root.insert(QStringLiteral("name"), QString::fromStdString(device->name()));
	root.insert(QStringLiteral("q"), doublesToJson(device->qValues()));
	root.insert(QStringLiteral("namedPoses"), toQ(posesJ));
	root.insert(QStringLiteral("poseSignalBindings"), toQ(bindsJ));
	root.insert(QStringLiteral("links"), toQ(linksJ));
	root.insert(QStringLiteral("joints"), toQ(jointsJ));
	root.insert(QStringLiteral("signals"),
				QJsonDocument::fromJson(QByteArray::fromStdString(device->ioSignalsJson().dump())).array());

	if (const CustomDeviceRobotMountComponent* mount = CustomDeviceRobotMountComponent::mountOf(*device).get())
	{
		QJsonObject rm;
		rm.insert(QStringLiteral("enabled"), mount->enabled());
		rm.insert(QStringLiteral("robotSceneBackendId"), QString::fromStdString(mount->robotSceneBackendId()));
		rm.insert(QStringLiteral("flangeLinkName"), QString::fromStdString(mount->flangeLinkName()));
		rm.insert(QStringLiteral("flangeBackendId"), QString::fromStdString(mount->flangeBackendId()));
		rm.insert(QStringLiteral("mountFrameBackendId"), QString::fromStdString(mount->mountFrameBackendId()));
		root.insert(QStringLiteral("robotMount"), rm);
	}
	return root;
}

bool putCustomDeviceRuntimeFields(DocumentHost& host, const QString& deviceId, const QJsonObject& body, QString* err)
{
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString()));
	if (!device)
	{
		if (err)
			*err = QStringLiteral("device not found");
		return false;
	}
	if (body.contains(QStringLiteral("name")))
		device->setName(body.value(QStringLiteral("name")).toString().toStdString());
	if (body.contains(QStringLiteral("namedPoses")))
	{
		std::vector<CustomDeviceNamedPose> poses;
		const QByteArray raw =
			QJsonDocument(body.value(QStringLiteral("namedPoses")).toArray()).toJson(QJsonDocument::Compact);
		nlohmann::json j = nlohmann::json::parse(raw.constData(), nullptr, false);
		if (j.is_discarded() || !readCustomDeviceNamedPosesFromJson(j, poses))
		{
			if (err)
				*err = QStringLiteral("namedPoses invalid");
			return false;
		}
		device->setNamedPoses(poses);
	}
	if (body.contains(QStringLiteral("poseSignalBindings")))
	{
		std::vector<CustomDevicePoseSignalBinding> binds;
		const QByteArray raw =
			QJsonDocument(body.value(QStringLiteral("poseSignalBindings")).toArray()).toJson(QJsonDocument::Compact);
		nlohmann::json j = nlohmann::json::parse(raw.constData(), nullptr, false);
		if (j.is_discarded() || !readCustomDevicePoseSignalBindingsFromJson(j, binds))
		{
			if (err)
				*err = QStringLiteral("poseSignalBindings invalid");
			return false;
		}
		device->setPoseSignalBindings(binds);
	}
	if (body.contains(QStringLiteral("signals")))
	{
		const QByteArray raw =
			QJsonDocument(body.value(QStringLiteral("signals")).toArray()).toJson(QJsonDocument::Compact);
		nlohmann::json j = nlohmann::json::parse(raw.constData(), nullptr, false);
		if (j.is_discarded())
		{
			if (err)
				*err = QStringLiteral("signals invalid");
			return false;
		}
		device->setIoSignalsJson(j.is_array() ? nlohmann::json{{"signals", j}} : j);
		host.ioSignalNetwork().syncOwnersFromDocument(host);
	}
	if (body.contains(QStringLiteral("q")))
	{
		device->setQValues(doublesFromJson(body.value(QStringLiteral("q")).toArray()));
	}
	return true;
}

bool applyCustomDeviceQ(DocumentHost& host, const QString& deviceId, const QJsonObject& body, QString* err)
{
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString()));
	if (!device)
	{
		if (err)
			*err = QStringLiteral("device not found");
		return false;
	}
	std::vector<double> q = body.contains(QStringLiteral("q")) ? doublesFromJson(body.value(QStringLiteral("q")).toArray())
															   : device->qValues();
	device->setQValues(q);
	IRobotBackendPoseSink* sink = poseSinkOf(host);
	if (!CustomDeviceKinematics::applyQ(*device, &host.backend(), sink, &q))
	{
		if (err)
			*err = QStringLiteral("applyQ failed");
		return false;
	}
	flushCustomDeviceLinkGeometryVisual(host, deviceId.toStdString());
	flushCustomDeviceMotionCenterFrameVisual(host, deviceId.toStdString());
	emit host.visualSceneDirty();
	return true;
}

bool gotoCustomDevicePose(DocumentHost& host, const QString& deviceId, const QJsonObject& body, QString* err)
{
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString()));
	if (!device)
	{
		if (err)
			*err = QStringLiteral("device not found");
		return false;
	}
	const QString poseId = body.value(QStringLiteral("poseId")).toString();
	const CustomDeviceNamedPose* pose = device->findNamedPose(poseId.toStdString());
	if (!pose && !body.value(QStringLiteral("poseName")).toString().isEmpty())
	{
		const QString want = body.value(QStringLiteral("poseName")).toString();
		for (const auto& p : device->namedPoses())
		{
			if (QString::fromStdString(p.name) == want)
			{
				pose = &p;
				break;
			}
		}
	}
	if (!pose)
	{
		if (err)
			*err = QStringLiteral("pose not found");
		return false;
	}
	QJsonObject applyBody;
	applyBody.insert(QStringLiteral("q"), doublesToJson(pose->q));
	return applyCustomDeviceQ(host, deviceId, applyBody, err);
}

void processCustomDevicePoseRisingEdges(DocumentHost& host, IoSignalNetwork& network, const QString& deviceOwnerId)
{
	if (network.ownerKind(deviceOwnerId) != IoSignalOwnerKind::Device)
		return;
	const auto device =
		std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceOwnerId.toStdString()));
	RobotIo::NamedSignalTable* table = network.table(deviceOwnerId);
	if (!device || !table)
		return;
	for (const RobotIo::SignalDef& s : table->entries())
	{
		if (s.kind != RobotIo::SignalKind::DI || s.name.empty())
			continue;
		const QString name = QString::fromStdString(s.name);
		const QString key = deviceOwnerId + QLatin1Char('|') + name;
		bool now = false;
		if (!readDeviceDi(network, deviceOwnerId, name, &now))
			continue;
		// 无历史按低电平：网页无桌面那种持续采样，避免「首次已是高」被吞
		const bool prev = edgeMemory().value(key, false);
		edgeMemory().insert(key, now);
		if (!(!prev && now))
			continue;
		for (const auto& b : device->poseSignalBindings())
		{
			if (!b.enabled || QString::fromStdString(b.signalName) != name)
				continue;
			const CustomDeviceNamedPose* pose = device->findNamedPose(b.poseId);
			if (!pose)
				continue;
			(void)CustomDevicePoseMotionHost::forHost(host).start(deviceOwnerId, pose->q, b.durationSec);
			break;
		}
	}
}

void clearCustomDevicePoseEdgeMemory(DocumentHost* host)
{
	edgeMemory().clear();
	if (host)
		CustomDevicePoseMotionHost::forHost(*host).stopAll();
}

void primeCustomDevicePoseEdgeMemory(IoSignalNetwork& network)
{
	for (const QString& deviceOwnerId : network.ownerIds())
	{
		if (network.ownerKind(deviceOwnerId) != IoSignalOwnerKind::Device)
			continue;
		RobotIo::NamedSignalTable* table = network.table(deviceOwnerId);
		if (!table)
			continue;
		for (const RobotIo::SignalDef& s : table->entries())
		{
			if (s.kind != RobotIo::SignalKind::DI || s.name.empty())
				continue;
			const QString name = QString::fromStdString(s.name);
			bool now = false;
			if (!readDeviceDi(network, deviceOwnerId, name, &now))
				continue;
			edgeMemory().insert(deviceOwnerId + QLatin1Char('|') + name, now);
		}
	}
}

bool ensureCustomDevice(DocumentHost& host, const QJsonObject& body, QString* err, QString* outDeviceId)
{
	QString deviceId = body.value(QStringLiteral("id")).toString().trimmed();
	std::shared_ptr<CustomDeviceBackendData> device;
	if (!deviceId.isEmpty())
		device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString()));
	if (device)
	{
		if (body.contains(QStringLiteral("name")))
			device->setName(body.value(QStringLiteral("name")).toString().toStdString());
		if (outDeviceId)
			*outDeviceId = deviceId;
		return true;
	}
	device = std::make_shared<CustomDeviceBackendData>();
	deviceId = deviceId.isEmpty()
				  ? QStringLiteral("CustomDevice_%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces))
				  : deviceId;
	device->setId(deviceId.toStdString());
	device->setName(body.value(QStringLiteral("name")).toString(QStringLiteral("CustomDevice")).toStdString());
	QString regErr;
	if (!registerAdoptedCustomDeviceAndLoadScene(host, device, QString(), QString(), true, &regErr))
	{
		if (err)
			*err = regErr.isEmpty() ? QStringLiteral("register failed") : regErr;
		return false;
	}
	host.ioSignalNetwork().syncOwnersFromDocument(host);
	if (outDeviceId)
		*outDeviceId = deviceId;
	return true;
}

bool attachCustomDeviceChildren(DocumentHost& host, const QString& deviceId, const QJsonArray& childIds, QString* err)
{
	if (deviceId.isEmpty())
	{
		if (err)
			*err = QStringLiteral("deviceId empty");
		return false;
	}
	if (!std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString())))
	{
		if (err)
			*err = QStringLiteral("device not found");
		return false;
	}
	for (const QJsonValue& v : childIds)
	{
		const QString childId = v.toString().trimmed();
		if (childId.isEmpty())
			continue;
		QString attachErr;
		if (!attachBackendChildToCustomDevice(host, deviceId.toStdString(), childId.toStdString(), &attachErr))
		{
			if (err)
				*err = attachErr.isEmpty() ? QStringLiteral("attach failed") : attachErr;
			return false;
		}
	}
	cloudsim::core::FollowSolveContextDto ctx;
	(void)host.data().runFollowSolveAndSync(ctx, nullptr);
	return true;
}

QJsonObject listAssemblyGeometryCandidatesJson(DocumentHost& host)
{
	QJsonArray arr;
	for (const auto& obj : host.listObjects())
	{
		if (!obj)
			continue;
		const std::string& cn = obj->className();
		if (!backend_type::isMeshClassName(cn) && !backend_type::isBrepWorkpieceClassName(cn))
			continue;
		QJsonObject o;
		o.insert(QStringLiteral("id"), QString::fromStdString(obj->id()));
		o.insert(QStringLiteral("name"), QString::fromStdString(obj->name().empty() ? obj->id() : obj->name()));
		o.insert(QStringLiteral("className"), QString::fromStdString(cn));
		arr.append(o);
	}
	return QJsonObject{{QStringLiteral("ok"), true}, {QStringLiteral("objects"), arr}};
}

bool commitCustomDeviceAssembly(DocumentHost& host, const QJsonObject& body, QString* err, QString* outDeviceId)
{
	std::vector<CustomDeviceLink> links;
	std::vector<CustomDeviceJoint> joints;
	{
		const QByteArray raw = QJsonDocument(body.value(QStringLiteral("links")).toArray()).toJson(QJsonDocument::Compact);
		nlohmann::json j = nlohmann::json::parse(raw.constData(), nullptr, false);
		if (j.is_discarded() || !readCustomDeviceLinksFromJson(j, links))
		{
			if (err)
				*err = QStringLiteral("links invalid");
			return false;
		}
	}
	{
		const QByteArray raw =
			QJsonDocument(body.value(QStringLiteral("joints")).toArray()).toJson(QJsonDocument::Compact);
		nlohmann::json j = nlohmann::json::parse(raw.constData(), nullptr, false);
		if (j.is_discarded() || !readCustomDeviceJointsFromJson(j, joints))
		{
			if (err)
				*err = QStringLiteral("joints invalid");
			return false;
		}
	}
	if (links.empty() || joints.empty())
	{
		if (err)
			*err = QStringLiteral("links and joints required");
		return false;
	}

	QString deviceId = body.value(QStringLiteral("id")).toString();
	std::shared_ptr<CustomDeviceBackendData> device;
	if (!deviceId.isEmpty())
		device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString()));
	if (!device)
	{
		device = std::make_shared<CustomDeviceBackendData>();
		deviceId = QStringLiteral("CustomDevice_%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces));
		device->setId(deviceId.toStdString());
		const QString name = body.value(QStringLiteral("name")).toString(QStringLiteral("CustomDevice"));
		device->setName(name.toStdString());
		QString regErr;
		if (!registerAdoptedCustomDeviceAndLoadScene(host, device, QString(), QString(), true, &regErr))
		{
			if (err)
				*err = regErr.isEmpty() ? QStringLiteral("register failed") : regErr;
			return false;
		}
	}
	else if (body.contains(QStringLiteral("name")))
	{
		device->setName(body.value(QStringLiteral("name")).toString().toStdString());
	}

	// 提交前挂父子，保证 commitGraph 能读到世界矩阵
	for (const CustomDeviceLink& L : links)
	{
		if (L.geometryBackendId.empty())
			continue;
		QString attachErr;
		(void)attachBackendChildToCustomDevice(host, deviceId.toStdString(), L.geometryBackendId, &attachErr);
	}

	IRobotBackendPoseSink* sink = poseSinkOf(host);
	if (!CustomDeviceAssemblyCommit::commitGraph(*device, links, joints, host.backend(), sink))
	{
		if (err)
			*err = QStringLiteral("commitGraph failed");
		return false;
	}
	finalizeCustomDeviceLinkJointGraph(host, deviceId.toStdString());
	cloudsim::core::FollowSolveContextDto ctx;
	(void)host.data().runFollowSolveAndSync(ctx, nullptr);
	host.ioSignalNetwork().syncOwnersFromDocument(host);
	if (outDeviceId)
		*outDeviceId = deviceId;
	return true;
}

bool exportCustomDeviceUrdfZip(DocumentHost& host, const QString& deviceId, const QString& packageParentDir,
							   QString* err, QString* outPackageDir)
{
	QString urdfPath;
	QString packageRoot;
	if (!exportCustomDeviceUrdfPackage(host, deviceId.toStdString(), packageParentDir, &urdfPath, &packageRoot, err))
		return false;
	if (outPackageDir)
		*outPackageDir = packageRoot;
	return true;
}

QJsonObject listRobotsForMountJson(DocumentHost& host)
{
	QJsonArray arr;
	if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		const QVector<HeadlessRobotContext::InstanceInfo> instances = hrc->listInstances();
		for (int i = 0; i < instances.size(); ++i)
		{
			const HeadlessRobotContext::InstanceInfo& info = instances[i];
			QJsonObject o;
			o.insert(QStringLiteral("sceneBackendId"), info.sceneRootBackendId);
			o.insert(QStringLiteral("label"), info.label.isEmpty() ? info.sceneRootBackendId : info.label);
			QString flangeLink = QString::fromStdString(hrc->robotCoordinateFramesForInstance(i).flangeLinkName);
			o.insert(QStringLiteral("flangeLinkName"), flangeLink);
			o.insert(QStringLiteral("flangeBackendId"), hrc->robotFlangeBackendId(info.sceneRootBackendId));
			arr.append(o);
		}
	}
	return QJsonObject{{QStringLiteral("ok"), true}, {QStringLiteral("robots"), arr}};
}

bool mountCustomDeviceToRobotFlange(DocumentHost& host, const QString& deviceId, const QJsonObject& body, QString* err)
{
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString()));
	if (!device)
	{
		if (err)
		{
			*err = QStringLiteral("device not found");
		}
		return false;
	}
	const QString robotSceneBackendId = body.value(QStringLiteral("robotSceneBackendId")).toString().trimmed();
	if (robotSceneBackendId.isEmpty())
	{
		if (err)
		{
			*err = QStringLiteral("robotSceneBackendId required");
		}
		return false;
	}
	const QString flangeLinkName = body.value(QStringLiteral("flangeLinkName")).toString().trimmed();
	const QString mountFrameBackendId = body.value(QStringLiteral("mountFrameBackendId")).toString().trimmed();
	const QString flangeBackendId = body.value(QStringLiteral("flangeBackendId")).toString().trimmed();
	BackendMat4 toolMat = BackendMat4::identity();
	if (body.contains(QStringLiteral("toolFrameInFlange")) && body.value(QStringLiteral("toolFrameInFlange")).isArray())
	{
		const QJsonArray a = body.value(QStringLiteral("toolFrameInFlange")).toArray();
		if (a.size() >= 16)
		{
			for (int i = 0; i < 16; ++i)
			{
				toolMat.v[i] = a[i].toDouble();
			}
		}
	}

	(void)host.flushVisualSync();

	QVector<double> bodyJointAngles;
	if (body.contains(QStringLiteral("jointAnglesRad")) && body.value(QStringLiteral("jointAnglesRad")).isArray())
	{
		bodyJointAngles = qvectorFromJson(body.value(QStringLiteral("jointAnglesRad")).toArray());
	}
	const QVector<double>* jointOverride = bodyJointAngles.isEmpty() ? nullptr : &bodyJointAngles;

	QVector<double> localJointAngles;
	const QVector<double>* jointAnglesForMount = nullptr;
	BackendMat4 mountTcpWorld{};
	const BackendMat4* mountTcpWorldForAlign = nullptr;
	HeadlessRobotContext* hrc = host.headlessRobotContext();
	if (hrc && hrc->robotInstanceIndexForSceneBackendId(robotSceneBackendId) >= 0)
	{
		if (applyRobotFkBeforeDeviceMount(host, robotSceneBackendId, jointOverride, localJointAngles))
		{
			jointAnglesForMount = &localJointAngles;
			notifyRobotSceneAfterDeviceMountChange(host);

			HeadlessRobotContext::TcpPoseCapture tcpCapture;
			if (hrc->captureTcpPose(robotSceneBackendId, tcpCapture, nullptr))
			{
				mountTcpWorld = tcpCapture.worldMat;
				mountTcpWorldForAlign = &mountTcpWorld;
			}
		}
	}

	const bool mounted = mountCustomDeviceToFlange(*device, host, robotSceneBackendId, flangeLinkName, flangeBackendId,
												   mountFrameBackendId, toolMat, jointAnglesForMount,
												   mountTcpWorldForAlign, err);
	if (mounted)
	{
		notifyRobotSceneAfterDeviceMountChange(host);
	}
	return mounted;
}

bool unmountCustomDeviceFromRobotFlange(DocumentHost& host, const QString& deviceId, QString* err)
{
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(deviceId.toStdString()));
	if (!device)
	{
		if (err)
		{
			*err = QStringLiteral("device not found");
		}
		return false;
	}
	const bool ok = unmountCustomDeviceFromRobot(*device, host, err);
	if (ok)
	{
		notifyRobotSceneAfterDeviceMountChange(host);
	}
	return ok;
}

} // namespace cloudsim::host
