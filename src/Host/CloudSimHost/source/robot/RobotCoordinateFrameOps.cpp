/// @file RobotCoordinateFrameOps.cpp
/// @brief 机器人坐标系运算

#include "RobotCoordinateFrameOps.h"

#include "DocumentHost.h"
#include "HeadlessRobotContext.h"
#include "RobotExternalAxes.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotPerLinkKinematicsSliceOsg.h"
#include "RobotProgramCatalog.h"
#include "RobotProgramStore.h"
#include "UrdfRobotLoader.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "CoreTypes.h"
#include "MeshBackendData.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

#include <cmath>
#include <unordered_set>

namespace cloudsim::host
{
namespace
{
bool toolGeometryMatches(const RobotCoordinate::RobotToolFrame& a, const RobotCoordinate::RobotToolFrame& b)
{
	return RobotCoordinate::encodeMat4Csv(RobotCoordinate::frameToMat4(a.T_flange_tool)) ==
			   RobotCoordinate::encodeMat4Csv(RobotCoordinate::frameToMat4(b.T_flange_tool)) &&
		   a.flangeLinkName == b.flangeLinkName;
}

bool motionFollowsActiveToolFrame(const RobotInstruction::Base& ins)
{
	const auto& ext = ins.extensionProperties();
	const auto it = ext.find(RobotCoordinate::kExtMotionToolFrameId);
	return it == ext.end() || it->second.empty() || it->second == "active";
}

void syncInstructionToolContext(RobotInstruction::Base& ins, const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	const RobotCoordinate::RobotToolFrame* tool = nullptr;
	if (motionFollowsActiveToolFrame(ins))
	{
		tool = RobotCoordinate::activeToolFrame(frames);
	}
	else if (const RobotCoordinate::RobotToolFrame* resolved =
				 RobotCoordinate::resolveToolFrameForExtension(frames, ins.extensionProperties()))
	{
		tool = resolved;
	}
	if (!tool)
	{
		return;
	}
	const BackendMat4 toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
	ins.setExtensionProperty(RobotCoordinate::kExtContextToolFrameMat4, RobotCoordinate::encodeMat4Csv(toolMat));
	ins.setExtensionProperty("context.activeToolFrameId", tool->id);
	const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(frames, *tool);
	if (!flangeLink.empty())
	{
		ins.setExtensionProperty("context.flangeLinkName", flangeLink);
	}
}

void fillOverlayPoseFromMat4(const BackendMat4& m, FrameOverlayEntry& e)
{
	RobotCoordinate::poseEulerFromTargetInBase(m, e.positionMm, e.eulerDeg);
	e.worldMat = m;
}

/// FK 基系 TCP × 基座放置 P（OSG 后乘，与 per-link M=…*Tq*P 一致）
BackendMat4 sceneWorldFromTcpInBase(const BackendMat4& tcpInBase, const cloudsim::core::Mat4& basePlacementCore)
{
	const osg::Matrixd tcpOsg = RobotMatrixOsg::matrixFromBackendColMajor(tcpInBase);
	const osg::Matrixd P = RobotSceneKinematics::osgMatrixFromCoreMat4(basePlacementCore);
	return RobotMatrixOsg::backendColMajorFromMatrix(tcpOsg * P);
}

bool backendWorldOf(BackendDataManager& backend, const QString& backendId, BackendMat4& outWorld)
{
	if (backendId.isEmpty())
	{
		return false;
	}
	const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(backend.getData(backendId.toStdString()));
	if (!mesh)
	{
		return false;
	}
	outWorld = mesh->worldMatrix();
	return true;
}

QString resolveFlangeLinkName(const QString& urdfPath, const RobotCoordinate::RobotCoordinateFrameSet& frames,
							  const RobotCoordinate::RobotToolFrame& tool)
{
	const std::string named = RobotCoordinate::effectiveFlangeLinkName(frames, tool);
	if (!named.empty())
	{
		return QString::fromStdString(named);
	}
	QStringList revoluteChildren;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildren, nullptr);
	return revoluteChildren.isEmpty() ? QString() : revoluteChildren.back();
}

bool toolTcpInBase(const QString& urdfPath, const QVector<double>& jointQ,
				   const RobotCoordinate::RobotCoordinateFrameSet& frames, const RobotCoordinate::RobotToolFrame& tool,
				   BackendMat4& outTcpInBase)
{
	const QString flangeQ = resolveFlangeLinkName(urdfPath, frames, tool);
	if (flangeQ.isEmpty())
	{
		return false;
	}
	QHash<QString, osg::Matrixd> linkWorld;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, jointQ, linkWorld, nullptr))
	{
		return false;
	}
	if (!linkWorld.contains(flangeQ))
	{
		return false;
	}
	const BackendMat4 T_tool = RobotCoordinate::frameToMat4(tool.T_flange_tool);
	outTcpInBase = RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(linkWorld.value(flangeQ), T_tool);
	return true;
}

/// 场景连杆世界矩阵 × T_flange_tool（FK 失败时的兜底，与可见 mesh 对齐）
bool toolTcpFromFlangeBackend(BackendDataManager& backend, const QHash<QString, QString>& linkToBackend,
							  const QString& flangeLink, const RobotCoordinate::RobotToolFrame& tool,
							  BackendMat4& outWorld)
{
	const QString flangeBid = linkToBackend.value(flangeLink);
	BackendMat4 flangeWorld = BackendMat4::identity();
	if (!backendWorldOf(backend, flangeBid, flangeWorld))
	{
		return false;
	}
	const BackendMat4 T_tool = RobotCoordinate::frameToMat4(tool.T_flange_tool);
	// 禁止 flangeWorld*tool 直乘：会与 Eigen 工具偏移约定不一致
	outWorld = RobotCoordinate::targetInBaseFromFlange(flangeWorld, T_tool);
	return true;
}
} // namespace

bool captureToolFrameFromTcpPose(const QString& urdfPath, const QVector<double>& jointAnglesRad,
								 const QString& flangeLinkName, const BackendMat4& T_base_tcp,
								 RobotCoordinate::RobotCoordinateFrameSet& frames, QString* outError)
{
	std::string flangeLink = flangeLinkName.toStdString();
	if (flangeLink.empty())
	{
		if (outError)
		{
			*outError = QStringLiteral("no flange link name");
		}
		return false;
	}
	QHash<QString, osg::Matrixd> linkWorld;
	QString err;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, jointAnglesRad, linkWorld, &err))
	{
		if (outError)
		{
			*outError = err;
		}
		return false;
	}
	const QString flangeQ = QString::fromStdString(flangeLink);
	if (!linkWorld.contains(flangeQ))
	{
		if (outError)
		{
			*outError = QStringLiteral("flange link not found: %1").arg(flangeQ);
		}
		return false;
	}
	const BackendMat4 T_base_flange = RobotMatrixOsg::backendColMajorFromMatrix(linkWorld.value(flangeQ));
	BackendMat4 invFlange{};
	if (!backend_mat4_invert_rigid(T_base_flange, invFlange))
	{
		if (outError)
		{
			*outError = QStringLiteral("cannot invert flange matrix");
		}
		return false;
	}
	BackendMat4 T_flange_tool{};
	backend_mat4_multiply(invFlange, T_base_tcp, T_flange_tool);
	RobotCoordinate::RobotToolFrame* target = nullptr;
	for (RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
	{
		if (tf.id == frames.activeToolFrameId)
		{
			target = &tf;
			break;
		}
	}
	if (!target && !frames.toolFrames.empty())
	{
		target = &frames.toolFrames.front();
	}
	if (!target)
	{
		if (outError)
		{
			*outError = QStringLiteral("no tool frame available");
		}
		return false;
	}
	target->T_flange_tool = RobotCoordinate::mat4ToFrame(T_flange_tool);
	return true;
}

bool captureUserFrameFromTcpPose(double posXmm, double posYmm, double posZmm, double eulerXdeg, double eulerYdeg,
								 double eulerZdeg, RobotCoordinate::RobotCoordinateFrameSet& frames, QString* outError)
{
	RobotCoordinate::RobotUserFrame* target = nullptr;
	for (RobotCoordinate::RobotUserFrame& uf : frames.userFrames)
	{
		if (uf.id == frames.activeUserFrameId)
		{
			target = &uf;
			break;
		}
	}
	if (!target && !frames.userFrames.empty())
	{
		target = &frames.userFrames.front();
	}
	if (!target)
	{
		if (outError)
		{
			*outError = QStringLiteral("no user frame available");
		}
		return false;
	}
	target->T_base_user.positionMm[0] = posXmm;
	target->T_base_user.positionMm[1] = posYmm;
	target->T_base_user.positionMm[2] = posZmm;
	target->T_base_user.eulerDeg[0] = eulerXdeg;
	target->T_base_user.eulerDeg[1] = eulerYdeg;
	target->T_base_user.eulerDeg[2] = eulerZdeg;
	return true;
}

void resetActiveToolFrame(RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	for (RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
	{
		if (tf.id == frames.activeToolFrameId)
		{
			tf.T_flange_tool = RobotCoordinate::identityRigidFrame();
			break;
		}
	}
}

bool buildFrameOverlaySnapshot(HeadlessRobotContext& hrc, BackendDataManager& backend, const QString& sceneRootBackendId,
							   FrameOverlaySnapshot& out, QString* outError)
{
	out = {};
	const int idx = hrc.robotInstanceIndexForSceneBackendId(sceneRootBackendId);
	if (idx < 0)
	{
		if (outError)
		{
			*outError = QStringLiteral("unknown sceneRootBackendId");
		}
		return false;
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = hrc.robotCoordinateFramesForInstance(idx);
	const QString urdfPath = hrc.robotUrdfAbsolutePathForInstance(idx);
	QStringList names;
	QVector<double> lo, hi, angles;
	if (!hrc.jointMetaForSceneRoot(sceneRootBackendId, names, lo, hi, angles))
	{
		angles = QVector<double>(hrc.robotRevoluteJointCountForInstance(idx), 0.0);
	}

	// 世界烘焙默认下法兰 mesh 外阵不是完整连杆系；叠加与桌面一致走 FK 基系 TCP 再 ×P
	cloudsim::core::Mat4 basePlacementCore = cloudsim::core::PlanContextDto::identityMat4();
	QHash<QString, QString> linkToBackend;
	cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
	if (hrc.robotPerLinkKinematicsForInstance(idx, pl))
	{
		basePlacementCore = pl.robotBasePlacementWorld;
		linkToBackend = pl.linkNameToBackendId;
	}

	// 显示开关只影响「是否画叠加轴」；拖拽罗盘始终需要激活工具 TCP
	for (const RobotCoordinate::RobotToolFrame& tool : frames.toolFrames)
	{
		const bool draw = frames.showToolFrameInScene && tool.showInScene;
		const bool isActive = (tool.id == frames.activeToolFrameId);
		if (!draw && !isActive)
		{
			continue;
		}
		BackendMat4 world = BackendMat4::identity();
		bool ok = false;
		BackendMat4 tcpInBase = BackendMat4::identity();
		if (toolTcpInBase(urdfPath, angles, frames, tool, tcpInBase))
		{
			world = sceneWorldFromTcpInBase(tcpInBase, basePlacementCore);
			ok = true;
		}
		else
		{
			const QString flangeQ = resolveFlangeLinkName(urdfPath, frames, tool);
			ok = toolTcpFromFlangeBackend(backend, linkToBackend, flangeQ, tool, world);
		}
		if (!ok)
		{
			continue;
		}
		FrameOverlayEntry e;
		e.id = QString::fromStdString(tool.id);
		e.name = QString::fromStdString(tool.name);
		e.active = isActive;
		fillOverlayPoseFromMat4(world, e);
		out.tools.push_back(e);
	}

	if (frames.showUserFramesInScene)
	{
		for (const RobotCoordinate::RobotUserFrame& uf : frames.userFrames)
		{
			if (!uf.showInScene)
			{
				continue;
			}
			const BackendMat4 userInBase = RobotCoordinate::frameToMat4(uf.T_base_user);
			const BackendMat4 world = sceneWorldFromTcpInBase(userInBase, basePlacementCore);
			FrameOverlayEntry e;
			e.id = QString::fromStdString(uf.id);
			e.name = QString::fromStdString(uf.name);
			e.active = (uf.id == frames.activeUserFrameId);
			fillOverlayPoseFromMat4(world, e);
			out.users.push_back(e);
		}
	}
	return true;
}

void syncProgramToolContextAfterFrameChange(RobotProgramStore& store, const QString& sceneRootBackendId,
											const RobotCoordinate::RobotCoordinateFrameSet& oldFrames,
											const RobotCoordinate::RobotCoordinateFrameSet& newFrames)
{
	if (coordinateFrameSetPlanningEquals(oldFrames, newFrames))
	{
		return;
	}
	const bool activeChanged = oldFrames.activeToolFrameId != newFrames.activeToolFrameId;
	std::unordered_set<std::string> changedToolIds;
	for (const RobotCoordinate::RobotToolFrame& nt : newFrames.toolFrames)
	{
		const RobotCoordinate::RobotToolFrame* ot = RobotCoordinate::findToolFrameById(oldFrames, nt.id);
		if (!ot || !toolGeometryMatches(*ot, nt))
		{
			changedToolIds.insert(nt.id);
		}
	}
	if (!activeChanged && changedToolIds.empty())
	{
		return;
	}

	RobotInstruction::RobotProgramCatalog& catalog = store.catalogFor(sceneRootBackendId);
	for (RobotInstruction::RobotProgram& prog : catalog.programs())
	{
		for (const std::shared_ptr<RobotInstruction::Base>& step : prog.steps)
		{
			if (!step || !step->hasPoseProperty())
			{
				continue;
			}
			bool affected = false;
			if (activeChanged && motionFollowsActiveToolFrame(*step))
			{
				affected = true;
			}
			else
			{
				const auto& ext = step->extensionProperties();
				const auto it = ext.find(RobotCoordinate::kExtMotionToolFrameId);
				if (it != ext.end() && changedToolIds.count(it->second) > 0)
				{
					affected = true;
				}
			}
			if (!affected)
			{
				continue;
			}
			step->eraseExtensionProperty("context.currentJointRadCsv");
			syncInstructionToolContext(*step, newFrames);
		}
	}
}

QJsonObject coordinateFrameSetToQJson(const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	nlohmann::json j;
	RobotCoordinate::writeCoordinateFrameSetToJson(frames, j);
	const QByteArray raw = QByteArray::fromStdString(j.dump());
	const QJsonDocument doc = QJsonDocument::fromJson(raw);
	return doc.isObject() ? doc.object() : QJsonObject{};
}

namespace
{
void readRigidFromQJson(const QJsonObject& o, RobotCoordinate::RobotRigidFrame& out)
{
	out = RobotCoordinate::identityRigidFrame();
	const QJsonArray pos = o.value(QStringLiteral("positionMm")).toArray();
	const QJsonArray eu = o.value(QStringLiteral("eulerDeg")).toArray();
	for (int i = 0; i < 3; ++i)
	{
		if (i < pos.size())
		{
			out.positionMm[i] = pos.at(i).toDouble();
		}
		if (i < eu.size())
		{
			out.eulerDeg[i] = eu.at(i).toDouble();
		}
	}
}
} // namespace

bool coordinateFrameSetFromQJson(const QJsonObject& obj, RobotCoordinate::RobotCoordinateFrameSet& out)
{
	// 走 QJson 直读，避免 nlohmann get<T> 对整型/布尔边界抛异常导致 PUT 整单失败
	out = RobotCoordinate::RobotCoordinateFrameSet{};
	out.flangeLinkName = obj.value(QStringLiteral("flangeLinkName")).toString().toStdString();
	out.activeToolFrameId = obj.value(QStringLiteral("activeToolFrameId")).toString().toStdString();
	out.activeUserFrameId = obj.value(QStringLiteral("activeUserFrameId")).toString().toStdString();
	out.showToolFrameInScene = obj.value(QStringLiteral("showToolFrame")).toBool(true);
	out.showUserFramesInScene = obj.value(QStringLiteral("showUserFrames")).toBool(true);

	const QJsonArray tools = obj.value(QStringLiteral("toolFrames")).toArray();
	for (const QJsonValue& v : tools)
	{
		if (!v.isObject())
		{
			continue;
		}
		const QJsonObject item = v.toObject();
		RobotCoordinate::RobotToolFrame tf;
		tf.id = item.value(QStringLiteral("id")).toString().toStdString();
		tf.name = item.value(QStringLiteral("name")).toString().toStdString();
		tf.flangeLinkName = item.value(QStringLiteral("flangeLinkName")).toString().toStdString();
		tf.showInScene = item.value(QStringLiteral("showInScene")).toBool(true);
		if (item.contains(QStringLiteral("T_flange_tool")) && item.value(QStringLiteral("T_flange_tool")).isObject())
		{
			readRigidFromQJson(item.value(QStringLiteral("T_flange_tool")).toObject(), tf.T_flange_tool);
		}
		else if (item.contains(QStringLiteral("toolInFlange")) && item.value(QStringLiteral("toolInFlange")).isObject())
		{
			readRigidFromQJson(item.value(QStringLiteral("toolInFlange")).toObject(), tf.T_flange_tool);
		}
		if (tf.name.empty())
		{
			tf.name = "TFrame";
		}
		out.toolFrames.push_back(std::move(tf));
	}

	const QJsonArray users = obj.value(QStringLiteral("userFrames")).toArray();
	for (const QJsonValue& v : users)
	{
		if (!v.isObject())
		{
			continue;
		}
		const QJsonObject item = v.toObject();
		RobotCoordinate::RobotUserFrame uf;
		uf.id = item.value(QStringLiteral("id")).toString().toStdString();
		uf.name = item.value(QStringLiteral("name")).toString().toStdString();
		uf.showInScene = item.value(QStringLiteral("showInScene")).toBool(true);
		if (item.contains(QStringLiteral("T_base_user")) && item.value(QStringLiteral("T_base_user")).isObject())
		{
			readRigidFromQJson(item.value(QStringLiteral("T_base_user")).toObject(), uf.T_base_user);
		}
		else if (item.contains(QStringLiteral("userInBase")) && item.value(QStringLiteral("userInBase")).isObject())
		{
			readRigidFromQJson(item.value(QStringLiteral("userInBase")).toObject(), uf.T_base_user);
		}
		if (uf.name.empty())
		{
			uf.name = "UFrame";
		}
		out.userFrames.push_back(std::move(uf));
	}

	RobotCoordinate::ensureUniqueToolFrameIds(out);
	if (out.activeToolFrameId.empty() && !out.toolFrames.empty())
	{
		out.activeToolFrameId = out.toolFrames.front().id;
	}
	if (out.activeUserFrameId.empty() && !out.userFrames.empty())
	{
		out.activeUserFrameId = out.userFrames.front().id;
	}
	return true;
}

bool coordinateFrameSetPlanningEquals(const RobotCoordinate::RobotCoordinateFrameSet& a,
									  const RobotCoordinate::RobotCoordinateFrameSet& b)
{
	RobotCoordinate::RobotCoordinateFrameSet aa = a;
	RobotCoordinate::RobotCoordinateFrameSet bb = b;
	aa.showToolFrameInScene = bb.showToolFrameInScene;
	aa.showUserFramesInScene = bb.showUserFramesInScene;
	for (size_t i = 0; i < aa.toolFrames.size() && i < bb.toolFrames.size(); ++i)
	{
		aa.toolFrames[i].showInScene = bb.toolFrames[i].showInScene;
	}
	for (size_t i = 0; i < aa.userFrames.size() && i < bb.userFrames.size(); ++i)
	{
		aa.userFrames[i].showInScene = bb.userFrames[i].showInScene;
	}
	const nlohmann::json ja = [&] {
		nlohmann::json j;
		RobotCoordinate::writeCoordinateFrameSetToJson(aa, j);
		return j;
	}();
	const nlohmann::json jb = [&] {
		nlohmann::json j;
		RobotCoordinate::writeCoordinateFrameSetToJson(bb, j);
		return j;
	}();
	return ja == jb;
}

std::string addToolFrame(RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	RobotCoordinate::RobotToolFrame tf;
	tf.id = RobotCoordinate::allocateUniqueToolFrameId(frames);
	tf.name = std::string("TFrame") + std::to_string(frames.toolFrames.size() + 1);
	tf.T_flange_tool = RobotCoordinate::identityRigidFrame();
	tf.flangeLinkName = frames.flangeLinkName;
	tf.showInScene = true;
	frames.toolFrames.push_back(tf);
	return frames.toolFrames.back().id;
}

namespace
{
std::string allocateUniqueUserFrameId(const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	// makeUserFrameId 进程计数器不感知已有 id，加载工程后易与 UFR_1 撞车
	for (int guard = 0; guard < 10000; ++guard)
	{
		const std::string id = RobotCoordinate::makeUserFrameId();
		if (!RobotCoordinate::findUserFrameById(frames, id))
		{
			return id;
		}
	}
	return std::string("UFR_") + std::to_string(frames.userFrames.size() + 1000ULL);
}
} // namespace

std::string addUserFrame(RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	RobotCoordinate::RobotUserFrame uf;
	uf.id = allocateUniqueUserFrameId(frames);
	uf.name = std::string("UFrame") + std::to_string(frames.userFrames.size() + 1);
	uf.T_base_user = RobotCoordinate::identityRigidFrame();
	uf.showInScene = true;
	frames.userFrames.push_back(uf);
	if (frames.activeUserFrameId.empty())
	{
		frames.activeUserFrameId = frames.userFrames.back().id;
	}
	return frames.userFrames.back().id;
}

std::string duplicateToolFrame(RobotCoordinate::RobotCoordinateFrameSet& frames, const std::string& sourceId)
{
	const RobotCoordinate::RobotToolFrame* src = RobotCoordinate::findToolFrameById(frames, sourceId);
	if (!src)
	{
		return {};
	}
	RobotCoordinate::RobotToolFrame copy = *src;
	copy.id = RobotCoordinate::allocateUniqueToolFrameId(frames);
	copy.name += "_copy";
	copy.showInScene = true;
	frames.toolFrames.push_back(copy);
	return frames.toolFrames.back().id;
}

std::string duplicateUserFrame(RobotCoordinate::RobotCoordinateFrameSet& frames, const std::string& sourceId)
{
	const RobotCoordinate::RobotUserFrame* src = RobotCoordinate::findUserFrameById(frames, sourceId);
	if (!src)
	{
		return {};
	}
	RobotCoordinate::RobotUserFrame copy = *src;
	copy.id = allocateUniqueUserFrameId(frames);
	copy.name += "_copy";
	copy.showInScene = true;
	frames.userFrames.push_back(copy);
	return frames.userFrames.back().id;
}

void mergeRobotKinematicsIntoProjectRoot(DocumentHost& host, QJsonObject& root)
{
	HeadlessRobotContext* hrc = host.headlessRobotContext();
	if (!hrc || hrc->robotKinematicInstanceCount() <= 0)
	{
		root.remove(QStringLiteral("robotKinematicsInstances"));
		return;
	}
	QJsonArray robotsArr;
	const QVector<HeadlessRobotContext::InstanceInfo> instances = hrc->listInstances();
	for (int ri = 0; ri < hrc->robotKinematicInstanceCount(); ++ri)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (!hrc->robotPerLinkKinematicsForInstance(ri, pl))
		{
			continue;
		}
		QJsonObject rk;
		rk.insert(QStringLiteral("mode"), QStringLiteral("perLink"));
		rk.insert(QStringLiteral("urdf"), pl.urdfAbsolutePath);
		rk.insert(QStringLiteral("sceneRootBackendId"), pl.sceneRootBackendId);
		const QString prefix = hrc->robotJointKeyPrefixForInstance(ri);
		QString jointRoot = prefix;
		if (jointRoot.endsWith(QStringLiteral("::")))
		{
			jointRoot.chop(2);
		}
		rk.insert(QStringLiteral("jointPrefixRoot"), jointRoot);
		rk.insert(QStringLiteral("importKey"), jointRoot + QStringLiteral("_ctx"));
		QJsonObject linksObj;
		for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
		{
			linksObj.insert(it.key(), it.value());
		}
		rk.insert(QStringLiteral("links"), linksObj);
		rk.insert(QStringLiteral("coordinateFrames"),
				  coordinateFrameSetToQJson(hrc->robotCoordinateFramesForInstance(ri)));

		nlohmann::json eaJ;
		RobotExternal::writeExternalAxisConfigSetToJson(hrc->robotExternalAxesForInstance(ri), eaJ);
		const QByteArray eaRaw = QByteArray::fromStdString(eaJ.dump());
		const QJsonDocument eaDoc = QJsonDocument::fromJson(eaRaw);
		if (eaDoc.isObject())
		{
			rk.insert(QStringLiteral("externalAxes"), eaDoc.object());
		}

		QJsonArray baseArr;
		for (int c = 0; c < 4; ++c)
		{
			for (int r = 0; r < 4; ++r)
			{
				baseArr.append(pl.robotBasePlacementWorld[static_cast<size_t>(c * 4 + r)]);
			}
		}
		rk.insert(QStringLiteral("basePlacementWorld"), baseArr);

		if (ri < instances.size())
		{
			QStringList jNames;
			QVector<double> jLo, jHi, jAng;
			if (hrc->jointMetaForSceneRoot(instances[ri].sceneRootBackendId, jNames, jLo, jHi, jAng) && !jAng.isEmpty())
			{
				QJsonArray ja;
				for (double v : jAng)
				{
					ja.append(v);
				}
				rk.insert(QStringLiteral("jointAnglesRad"), ja);
			}
		}
		robotsArr.push_back(rk);
	}
	if (robotsArr.isEmpty())
	{
		root.remove(QStringLiteral("robotKinematicsInstances"));
	}
	else
	{
		root.insert(QStringLiteral("robotKinematicsInstances"), robotsArr);
	}
}

} // namespace cloudsim::host
