/// @file RobotPlanInstruction.cpp
/// @brief RobotPlanInstruction 实现

#include "RobotPlanInstruction.h"

#include "IRobotSimulationDocument.h"
#include "IRobotUrdfImportContext.h"
#include "RobotCoordinateFrames.h"
#include "RobotExternalAxes.h"
#include "RobotInstructionController.h"
#include "RobotInstructionFactory.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>

#include <json.hpp>

namespace cloudsim::host
{
namespace
{
/// Host 侧规划准备：不链 RobotWidget（与 prepareMotionInstructionForPlanning 同语义，doc/osg 可空）
void prepareMotionInstructionForHostPlanning(RobotInstruction::Base& ins, const QVector<double>& rollingQ,
											 const QString& urdfPath, const std::string& defaultTcpLinkName,
											 const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames)
{
	QStringList parts;
	parts.reserve(rollingQ.size());
	for (double v : rollingQ)
	{
		parts.push_back(QString::number(v, 'g', 12));
	}
	ins.setExtensionProperty("context.currentJointRadCsv", parts.join(QLatin1Char(',')).toStdString());
	ins.setExtensionProperty("context.urdfPath", urdfPath.toStdString());
	ins.setExtensionProperty("context.tcpLinkName", defaultTcpLinkName);
	if (!coordinateFrames)
	{
		return;
	}
	const BackendMat4 T_tool = RobotCoordinate::toolMat4ForExtension(*coordinateFrames, ins.extensionProperties());
	ins.setExtensionProperty("context.toolFrameMat4", RobotCoordinate::encodeMat4Csv(T_tool));
	if (const RobotCoordinate::RobotToolFrame* tool =
			RobotCoordinate::resolveToolFrameForExtension(*coordinateFrames, ins.extensionProperties()))
	{
		const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(*coordinateFrames, *tool);
		if (!flangeLink.empty())
		{
			ins.setExtensionProperty("context.flangeLinkName", flangeLink);
		}
	}
	else if (!coordinateFrames->flangeLinkName.empty())
	{
		ins.setExtensionProperty("context.flangeLinkName", coordinateFrames->flangeLinkName);
	}
}

nlohmann::json motionDtoToJson(const core::MotionInstructionDto& instruction)
{
	nlohmann::json j;
	j["type"] = instruction.instructionType.toStdString();
	nlohmann::json pose;
	pose["x"] = instruction.targetPose.positionMm.x;
	pose["y"] = instruction.targetPose.positionMm.y;
	pose["z"] = instruction.targetPose.positionMm.z;
	j["pose"] = pose;
	nlohmann::json euler;
	euler["x"] = instruction.targetPose.eulerDeg.x;
	euler["y"] = instruction.targetPose.eulerDeg.y;
	euler["z"] = instruction.targetPose.eulerDeg.z;
	j["eulerDeg"] = euler;
	if (!instruction.axisConfiguration.isEmpty())
	{
		const QByteArray raw = QJsonDocument(instruction.axisConfiguration).toJson(QJsonDocument::Compact);
		j["axisConfiguration"] =
			nlohmann::json::parse(std::string(raw.constData(), static_cast<size_t>(raw.size())), nullptr, false);
	}
	if (!instruction.jointRadCsv.isEmpty())
	{
		j["context"] = nlohmann::json::object();
		j["context"]["currentJointRadCsv"] = instruction.jointRadCsv.toStdString();
	}
	if (!instruction.extensions.isEmpty())
	{
		const QByteArray raw = QJsonDocument(instruction.extensions).toJson(QJsonDocument::Compact);
		const nlohmann::json ext =
			nlohmann::json::parse(std::string(raw.constData(), static_cast<size_t>(raw.size())), nullptr, false);
		if (ext.is_object())
		{
			for (auto it = ext.begin(); it != ext.end(); ++it)
			{
				j[it.key()] = it.value();
			}
		}
	}
	return j;
}

core::MotionInstructionDto motionDtoFromInstructionJson(const nlohmann::json& j)
{
	core::MotionInstructionDto dto;
	dto.instructionType = QString::fromStdString(j.value("type", std::string()));
	if (j.contains("pose") && j["pose"].is_object())
	{
		const nlohmann::json& p = j["pose"];
		dto.targetPose.positionMm.x = p.value("x", 0.0);
		dto.targetPose.positionMm.y = p.value("y", 0.0);
		dto.targetPose.positionMm.z = p.value("z", 0.0);
	}
	if (j.contains("eulerDeg") && j["eulerDeg"].is_object())
	{
		const nlohmann::json& e = j["eulerDeg"];
		dto.targetPose.eulerDeg.x = e.value("x", 0.0);
		dto.targetPose.eulerDeg.y = e.value("y", 0.0);
		dto.targetPose.eulerDeg.z = e.value("z", 0.0);
	}
	if (j.contains("axisConfiguration") && j["axisConfiguration"].is_object())
	{
		const QByteArray raw = QByteArray::fromStdString(j["axisConfiguration"].dump());
		const QJsonDocument axisDoc = QJsonDocument::fromJson(raw);
		if (axisDoc.isObject())
		{
			dto.axisConfiguration = axisDoc.object();
		}
	}
	if (j.contains("context") && j["context"].is_object() && j["context"].contains("currentJointRadCsv"))
	{
		dto.jointRadCsv = QString::fromStdString(j["context"]["currentJointRadCsv"].get<std::string>());
	}
	QJsonObject extObj;
	for (auto it = j.begin(); it != j.end(); ++it)
	{
		const std::string key = it.key();
		if (key == "type" || key == "pose" || key == "eulerDeg" || key == "axisConfiguration" || key == "context")
		{
			continue;
		}
		if (it.value().is_object() || it.value().is_array())
		{
			const QByteArray raw = QByteArray::fromStdString(it.value().dump());
			const QJsonDocument extDoc = QJsonDocument::fromJson(raw);
			if (extDoc.isObject())
			{
				extObj.insert(QString::fromStdString(key), extDoc.object());
			}
			else if (extDoc.isArray())
			{
				extObj.insert(QString::fromStdString(key), extDoc.array());
			}
		}
		else if (it.value().is_string())
		{
			extObj.insert(QString::fromStdString(key), QString::fromStdString(it.value().get<std::string>()));
		}
		else if (it.value().is_number())
		{
			extObj.insert(QString::fromStdString(key), it.value().get<double>());
		}
	}
	if (!extObj.isEmpty())
	{
		dto.extensions = extObj;
	}
	return dto;
}

int resolveInstanceIndex(const core::PlanContextDto& context, IRobotUrdfImportContext& ctx)
{
	if (context.extensions.contains(QStringLiteral("instanceIndex")))
	{
		const int idx = context.extensions.value(QStringLiteral("instanceIndex")).toInt(-1);
		if (idx >= 0 && idx < ctx.robotKinematicInstanceCount())
		{
			return idx;
		}
	}
	const QString sceneRoot = context.extensions.value(QStringLiteral("sceneRootBackendId")).toString();
	if (sceneRoot.isEmpty())
	{
		return 0;
	}
	const int instIdx = ctx.robotInstanceIndexForSceneBackendId(sceneRoot);
	return instIdx < 0 ? 0 : instIdx;
}

} // namespace

bool planMotionInstruction(IRobotUrdfImportContext& ctx, const core::MotionInstructionDto& instruction,
						   const core::PlanContextDto& context, core::PlanResultDto& out, QString* outError)
{
	out = {};
	IRobotSimulationDocument* doc = ctx.urdfImportRobotSimulationDocument();
	if (!doc || !doc->hasRobotSimulationContext())
	{
		if (outError)
		{
			*outError = QStringLiteral("no robot simulation context");
		}
		return false;
	}
	if (instruction.instructionType.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("instructionType is empty");
		}
		return false;
	}
	const nlohmann::json j = motionDtoToJson(instruction);
	std::string parseErr;
	std::shared_ptr<RobotInstruction::Base> ins = RobotInstruction::createFromJson(j, &parseErr);
	if (!ins)
	{
		if (outError)
		{
			*outError = parseErr.empty() ? QStringLiteral("createFromJson failed") : QString::fromStdString(parseErr);
		}
		return false;
	}
	const int instIdx = resolveInstanceIndex(context, ctx);
	IRobotSimulationDocument* simDoc = ctx.urdfImportRobotSimulationDocument();
	QString urdfPath = context.urdfPath;
	if (urdfPath.isEmpty() && simDoc)
	{
		urdfPath = simDoc->robotUrdfAbsolutePathForInstance(instIdx);
	}
	std::string tcpLink = context.tcpLinkName.toStdString();
	if (tcpLink.empty())
	{
		tcpLink = "tool0";
	}
	RobotCoordinate::RobotCoordinateFrameSet& frames = ctx.robotCoordinateFramesForInstance(instIdx);
	prepareMotionInstructionForHostPlanning(*ins, context.seedJointRad, urdfPath, tcpLink, &frames);
	RobotInstruction::Controller controller;
	controller.buildDefaultPlanners();
	// PTP/LINE 主路径在此建 Controller；未注入则外轴联立永远不生效
	controller.setExternalAxes(ctx.robotExternalAxesForInstance(instIdx));
	RobotInstruction::PlanResult plan{};
	std::string planErr;
	if (!controller.plan(*ins, plan, &planErr))
	{
		if (outError)
		{
			*outError = planErr.empty() ? QStringLiteral("plan failed") : QString::fromStdString(planErr);
		}
		return false;
	}
	out.ok = plan.ok;
	out.error = QString::fromStdString(plan.summary);
	out.hasExternalAxisQ = plan.hasExternalAxisQ;
	out.externalAxisQ = plan.externalAxisQ;
	out.jointTargetsRad.clear();
	out.jointTargetsRad.reserve(static_cast<int>(plan.jointTargetsRad.size()));
	for (double v : plan.jointTargetsRad)
	{
		out.jointTargetsRad.append(v);
	}
	return out.ok;
}

bool planRobotInstruction(IRobotUrdfImportContext& ctx, RobotInstruction::Base& instruction,
						  const QVector<double>& seedJointRad, const int instanceIndex, const QString& urdfPath,
						  const std::string& defaultTcpLinkName, const QString& sceneRootBackendId,
						  RobotInstruction::PlanResult& out, QString* outError)
{
	out = {};
	const nlohmann::json j = RobotInstruction::toJson(instruction);
	const core::MotionInstructionDto motionDto = motionDtoFromInstructionJson(j);
	core::PlanContextDto planCtx;
	planCtx.seedJointRad = seedJointRad;
	planCtx.urdfPath = urdfPath;
	planCtx.tcpLinkName = QString::fromStdString(defaultTcpLinkName);
	planCtx.extensions.insert(QStringLiteral("instanceIndex"), instanceIndex);
	if (!sceneRootBackendId.isEmpty())
	{
		planCtx.extensions.insert(QStringLiteral("sceneRootBackendId"), sceneRootBackendId);
	}
	core::PlanResultDto hostResult;
	if (!planMotionInstruction(ctx, motionDto, planCtx, hostResult, outError))
	{
		return false;
	}
	out.ok = hostResult.ok;
	out.summary = hostResult.error.toStdString();
	out.hasExternalAxisQ = hostResult.hasExternalAxisQ;
	out.externalAxisQ = hostResult.externalAxisQ;
	out.jointTargetsRad.clear();
	out.jointTargetsRad.reserve(static_cast<size_t>(hostResult.jointTargetsRad.size()));
	for (double v : hostResult.jointTargetsRad)
	{
		out.jointTargetsRad.push_back(v);
	}
	return out.ok;
}

} // namespace cloudsim::host
