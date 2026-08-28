/// @file RobotPlanInstruction.cpp
/// @brief 规划指令 Host 路径

#include "RobotPlanInstruction.h"

#include "IRobotSimulationDocument.h"
#include "IRobotUrdfImportContext.h"
#include "RobotCoordinateFrames.h"
#include "RobotExternalAxes.h"
#include "RobotInstructionController.h"
#include "RobotInstructionFactory.h"
#include "RobotInstructionIkContext.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>

#include <json.hpp>

namespace cloudsim::host
{
namespace
{
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
	// 指令契约：不落盘关节 CSV（jointRadCsv / taughtJointRadCsv 忽略）
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
	if (j.contains("context") && j["context"].is_object())
	{
		j["context"].erase("currentJointRadCsv");
		j["context"].erase("taughtJointRadCsv");
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

QVector<double> parseJointCsv(const QString& csv)
{
	QVector<double> out;
	const QStringList parts = csv.split(QLatin1Char(','), Qt::SkipEmptyParts);
	out.reserve(parts.size());
	for (const QString& p : parts)
	{
		bool ok = false;
		const double v = p.trimmed().toDouble(&ok);
		if (!ok)
		{
			return {};
		}
		out.append(v);
	}
	return out;
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
	std::vector<double> seedStd(context.seedJointRad.begin(), context.seedJointRad.end());
	RobotInstruction::prepareInstructionIkContext(*ins, seedStd, urdfPath.toStdString(), tcpLink, &frames);

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
	out.durationSec = plan.durationSec;
	out.jointTargetsRad.clear();
	out.jointTargetsRad.reserve(static_cast<int>(plan.jointTargetsRad.size()));
	for (double v : plan.jointTargetsRad)
	{
		out.jointTargetsRad.append(v);
	}
	out.jointTrajectoryRad.clear();
	out.jointTrajectoryRad.reserve(static_cast<int>(plan.jointTrajectoryRad.size()));
	for (const std::vector<double>& sample : plan.jointTrajectoryRad)
	{
		QVector<double> row;
		row.reserve(static_cast<int>(sample.size()));
		for (double v : sample)
		{
			row.append(v);
		}
		out.jointTrajectoryRad.append(std::move(row));
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
	out.durationSec = hostResult.durationSec;
	out.jointTargetsRad.clear();
	out.jointTargetsRad.reserve(static_cast<size_t>(hostResult.jointTargetsRad.size()));
	for (double v : hostResult.jointTargetsRad)
	{
		out.jointTargetsRad.push_back(v);
	}
	out.jointTrajectoryRad.clear();
	out.jointTrajectoryRad.reserve(static_cast<size_t>(hostResult.jointTrajectoryRad.size()));
	for (const QVector<double>& row : hostResult.jointTrajectoryRad)
	{
		std::vector<double> sample(row.begin(), row.end());
		out.jointTrajectoryRad.push_back(std::move(sample));
	}
	return out.ok;
}

} // namespace cloudsim::host
