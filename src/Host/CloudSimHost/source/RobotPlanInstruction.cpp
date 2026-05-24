#include "RobotPlanInstruction.h"

#include "IRobotUrdfImportContext.h"
#include "IRobotSimulationDocument.h"

#include "RobotCoordinateFrames.h"
#include "RobotInstructionController.h"
#include "RobotInstructionFactory.h"
#include "RobotInstructionPlanningHelpers.h"

#include <QJsonDocument>

#include <json.hpp>

namespace cloudsim::host {

namespace {

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
		j["axisConfiguration"] = nlohmann::json::parse(
			std::string(raw.constData(), static_cast<size_t>(raw.size())), nullptr, false);
	}
	if (!instruction.jointRadCsv.isEmpty())
	{
		j["context"] = nlohmann::json::object();
		j["context"]["currentJointRadCsv"] = instruction.jointRadCsv.toStdString();
	}
	if (!instruction.extensions.isEmpty())
	{
		const QByteArray raw = QJsonDocument(instruction.extensions).toJson(QJsonDocument::Compact);
		const nlohmann::json ext = nlohmann::json::parse(
			std::string(raw.constData(), static_cast<size_t>(raw.size())), nullptr, false);
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
	if (j.contains("context") && j["context"].is_object()
		&& j["context"].contains("currentJointRadCsv"))
	{
		dto.jointRadCsv =
			QString::fromStdString(j["context"]["currentJointRadCsv"].get<std::string>());
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
	RobotInstructionPlanning::prepareMotionInstructionForPlanning(
		*ins, context.seedJointRad, nullptr, nullptr, instIdx, urdfPath, tcpLink, &frames);
	RobotInstruction::Controller controller;
	controller.buildDefaultPlanners();
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
	const std::string& defaultTcpLinkName, const QString& sceneRootBackendId, RobotInstruction::PlanResult& out,
	QString* outError)
{
	out = {};
	const nlohmann::json j = RobotInstruction::toJson(instruction);
	const core::MotionInstructionDto motionDto = motionDtoFromInstructionJson(j);
	core::PlanContextDto planCtx;
	planCtx.seedJointRad = seedJointRad;
	planCtx.urdfPath = urdfPath;
	planCtx.tcpLinkName = QString::fromStdString(defaultTcpLinkName);
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
	out.jointTargetsRad.clear();
	out.jointTargetsRad.reserve(static_cast<size_t>(hostResult.jointTargetsRad.size()));
	for (double v : hostResult.jointTargetsRad)
	{
		out.jointTargetsRad.push_back(v);
	}
	return out.ok;
}

} // namespace cloudsim::host
