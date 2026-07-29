/// @file AiActionPlanExecutor.cpp
/// @brief AiActionPlanExecutor 实现

#include "Ai/AiActionPlanExecutor.h"

#include "Ai/AiFeatureComposeSteps.h"
#include "Ai/AiHostButtonApiDispatch.h"
#include "Ai/AiMeshDefaults.h"
#include "AiCommandSchema.h"
#include "PluginHostContext.h"
#include "PluginPrimitiveTypes.h"

#include <QHash>
#include <QString>
#include <QStringList>
#include <functional>

#include <json.hpp>

namespace
{
PluginPrimitiveKind toPluginKind(BackendPrimitiveGeometry::PrimitiveKind k)
{
	switch (k)
	{
	case BackendPrimitiveGeometry::PrimitiveKind::Box:
		return PluginPrimitiveKind::Box;

	case BackendPrimitiveGeometry::PrimitiveKind::Cylinder:
		return PluginPrimitiveKind::Cylinder;

	case BackendPrimitiveGeometry::PrimitiveKind::Cone:
		return PluginPrimitiveKind::Cone;

	case BackendPrimitiveGeometry::PrimitiveKind::Sphere:
		return PluginPrimitiveKind::Sphere;
	}

	return PluginPrimitiveKind::Box;
}

struct AiPlanStepContext
{
	bool ephemeralCompose = false;

	QHash<QString, QString> stepIdToBackendId;

	QHash<QString, std::vector<float>> stepIdToWorldSoup;

	QStringList clarifyNotes;

	QString resolveBackendRef(const std::string& ref) const
	{
		if (ref.empty())
			return {};

		if (ref.front() == '$')
			return stepIdToBackendId.value(QString::fromStdString(ref.substr(1)));

		return QString::fromStdString(ref);
	}

	const std::vector<float>* resolveWorldSoupRef(const std::string& ref) const
	{
		if (ref.empty())
			return nullptr;

		QString stepId;

		if (ref.front() == '$')
			stepId = QString::fromStdString(ref.substr(1));

		else
			stepId = QString::fromStdString(ref);

		const auto it = stepIdToWorldSoup.constFind(stepId);

		if (it == stepIdToWorldSoup.constEnd() || it->empty())
			return nullptr;

		return &(*it);
	}
};

PluginMeshBooleanOp parseBooleanOp(const std::string& op)
{
	if (op == "union")
		return PluginMeshBooleanOp::Union;

	if (op == "intersection")
		return PluginMeshBooleanOp::Intersection;

	return PluginMeshBooleanOp::Difference;
}

bool parseCreateMeshCommandToPlugin(const nlohmann::json& cmdIn, PluginPrimitiveMeshParams& pp,
									PluginPrimitiveMeshQuality& pq, PluginMeshCreateOptions& opt, QString* outError,
									QString* outSummaryExtra = nullptr)
{
	nlohmann::json cmd = cmdIn;

	bool usedDefaults = false;

	AiMeshDefaults::applyMissingDimensions(cmd, &usedDefaults);

	if (outSummaryExtra && usedDefaults)
		*outSummaryExtra = AiMeshDefaults::defaultsAppliedNote(cmd, true);

	BackendPrimitiveGeometry::PrimitiveMeshParams params;

	BackendPrimitiveGeometry::PrimitiveMeshQuality quality;

	std::string displayName;

	std::string sourcePath;

	std::string errStd;

	if (!AiCommandSchema::parseCreateMeshCommand(cmd, params, quality, displayName, sourcePath, errStd))
	{
		if (outError)
			*outError = QString::fromStdString(errStd);

		return false;
	}

	pp.kind = toPluginKind(params.kind);

	pp.lengthMm = params.lengthMm;

	pp.widthMm = params.widthMm;

	pp.heightMm = params.heightMm;

	pp.radiusMm = params.radiusMm;

	pp.radiusTopMm = params.radiusTopMm;

	pq.segments = quality.segments;

	pq.rings = quality.rings;

	opt.displayName = QString::fromStdString(displayName);

	opt.sourcePath = QString::fromStdString(sourcePath);

	opt.selectInTree = false;

	opt.resetViewToHome = false;

	if (cmd.contains("pose_mm") && cmd["pose_mm"].is_object())
	{
		const auto& p = cmd["pose_mm"];

		opt.poseMm.x = p.value("x", 0.0);

		opt.poseMm.y = p.value("y", 0.0);

		opt.poseMm.z = p.value("z", 0.0);
	}

	if (cmd.contains("rotation_deg") && cmd["rotation_deg"].is_object())
	{
		const auto& r = cmd["rotation_deg"];

		opt.rotationDeg.x = r.value("x", 0.0);

		opt.rotationDeg.y = r.value("y", 0.0);

		opt.rotationDeg.z = r.value("z", 0.0);
	}

	return true;
}

bool executeCreateMeshStep(PluginHostContext& host, const nlohmann::json& cmdIn, QString* outError,
						   QString* outBackendId, QString* outSummaryExtra = nullptr)
{
	PluginPrimitiveMeshParams pp;

	PluginPrimitiveMeshQuality pq;

	PluginMeshCreateOptions opt;

	if (!parseCreateMeshCommandToPlugin(cmdIn, pp, pq, opt, outError, outSummaryExtra))
		return false;

	return host.createPrimitiveMesh(pp, pq, opt, outError, outBackendId);
}

bool planUsesEphemeralCompose(const nlohmann::json& root)
{
	if (root.value("version", 0) != 2 || !root.contains("steps") || !root["steps"].is_array())
		return false;

	for (const auto& step : root["steps"])
	{
		if (step.is_object() && step.value("api", "") == "booleanMesh")
			return true;
	}

	return false;
}

bool executeStep(PluginHostContext& host, const nlohmann::json& step, AiPlanStepContext& ctx, QString* outError)
{
	const std::string api = step.value("api", "");

	const nlohmann::json args =
		step.contains("args") && step["args"].is_object() ? step["args"] : nlohmann::json::object();

	const std::string stepId = step.value("id", "");

	if (api == "askClarify")
	{
		QStringList qs;

		if (args.contains("questions") && args["questions"].is_array())
		{
			for (const auto& q : args["questions"])
			{
				if (q.is_string())
					qs.append(QString::fromStdString(q.get<std::string>()));
			}
		}

		else if (args.contains("question") && args["question"].is_string())
		{
			qs.append(QString::fromStdString(args["question"].get<std::string>()));
		}

		if (qs.isEmpty())
			qs.append(QStringLiteral("请补充关键尺寸或约束后再生成。"));

		ctx.clarifyNotes.append(qs);

		return true;
	}

	if (api == "createPrimitiveMesh")
	{
		nlohmann::json cmd;

		cmd["version"] = 1;

		cmd["action"] = "create_mesh";

		cmd["primitive"] = args.value("primitive", "box");

		if (args.contains("dimensions_mm"))
			cmd["dimensions_mm"] = args["dimensions_mm"];

		if (args.contains("name"))
			cmd["name"] = args["name"];

		if (args.contains("pose_mm"))
			cmd["pose_mm"] = args["pose_mm"];

		if (args.contains("rotation_deg"))
			cmd["rotation_deg"] = args["rotation_deg"];

		if (args.contains("mesh_quality"))
			cmd["mesh_quality"] = args["mesh_quality"];

		AiMeshDefaults::applyMissingDimensions(cmd);

		if (ctx.ephemeralCompose)
		{
			PluginPrimitiveMeshParams pp;

			PluginPrimitiveMeshQuality pq;

			PluginMeshCreateOptions placement;

			if (!parseCreateMeshCommandToPlugin(cmd, pp, pq, placement, outError))
				return false;

			std::vector<float> worldSoup;

			if (!host.buildPrimitiveMeshSoup(pp, pq, placement, worldSoup, outError))
				return false;

			if (!stepId.empty())
				ctx.stepIdToWorldSoup[QString::fromStdString(stepId)] = std::move(worldSoup);

			return true;
		}

		QString backendId;

		if (!executeCreateMeshStep(host, cmd, outError, &backendId))
			return false;

		if (!stepId.empty() && !backendId.isEmpty())
			ctx.stepIdToBackendId[QString::fromStdString(stepId)] = backendId;

		return true;
	}

	if (api == "booleanMesh")
	{
		PluginBooleanMeshOptions opt;

		opt.op = parseBooleanOp(args.value("op", "difference"));

		if (args.contains("result_name"))
			opt.resultName = QString::fromStdString(args.value("result_name", ""));

		opt.hideOperands = args.value("hide_operands", true);

		opt.selectInTree = args.value("select_in_tree", true);

		opt.resetViewToHome = args.value("reset_view", false);

		if (ctx.ephemeralCompose)
		{
			const std::vector<float>* targetSoup = ctx.resolveWorldSoupRef(args.value("target", ""));

			const std::vector<float>* toolSoup = ctx.resolveWorldSoupRef(args.value("tool", ""));

			if (!targetSoup || !toolSoup)
			{
				if (outError)
					*outError = QStringLiteral("booleanMesh: invalid target/tool step reference.");

				return false;
			}

			std::string resultBackendId;

			if (!host.booleanMeshSoups(opt.op, *targetSoup, *toolSoup, opt, &resultBackendId, outError))
				return false;

			if (!stepId.empty() && !resultBackendId.empty())
				ctx.stepIdToBackendId[QString::fromStdString(stepId)] = QString::fromStdString(resultBackendId);

			return true;
		}

		const QString targetId = ctx.resolveBackendRef(args.value("target", ""));

		const QString toolId = ctx.resolveBackendRef(args.value("tool", ""));

		if (targetId.isEmpty() || toolId.isEmpty())
		{
			if (outError)
				*outError = QStringLiteral("booleanMesh: invalid target/tool reference.");

			return false;
		}

		std::string resultBackendId;

		if (!host.booleanMesh(opt.op, targetId.toStdString(), toolId.toStdString(), opt, &resultBackendId, outError))
			return false;

		if (!stepId.empty() && !resultBackendId.empty())
			ctx.stepIdToBackendId[QString::fromStdString(stepId)] = QString::fromStdString(resultBackendId);

		return true;
	}

	if (api == "importFileIntoActiveDocument")
	{
		const std::string path = args.value("path", "");

		if (!path.empty())
		{
			const bool isPc = args.value("is_point_cloud", false);

			std::string err;

			const std::string id = host.importFileIntoActiveDocument(path, isPc, &err);

			if (id.empty())
			{
				if (outError)
					*outError = QString::fromStdString(err);

				return false;
			}

			if (!stepId.empty())
				ctx.stepIdToBackendId[QString::fromStdString(stepId)] = QString::fromStdString(id);

			return true;
		}
		// 无 path：走 Agent 文件对话框
	}

	{
		QString ferr;
		if (AiFeatureComposeSteps::tryExecute(host, api, args, stepId, ctx.stepIdToBackendId, &ferr))
		{
			if (!ferr.isEmpty())
			{
				if (outError)
					*outError = ferr;
				return false;
			}
			return true;
		}
	}

	{
		QString err;
		if (AiHostButtonApiDispatch::tryExecute(host, api, args, &err))
		{
			if (!err.isEmpty())
			{
				if (outError)
					*outError = err;
				return false;
			}
			return true;
		}
	}

	if (outError)
		*outError = QStringLiteral("Unknown API: %1").arg(QString::fromStdString(api));

	return false;
}

} // namespace
namespace AiActionPlanExecutor
{
bool execute(const PluginHostContext& host, const QByteArray& planJsonUtf8, QString* outSummary, QString* outError)
{
	if (outSummary)
		outSummary->clear();

	if (outError)
		outError->clear();

	nlohmann::json root;

	try
	{
		root = nlohmann::json::parse(planJsonUtf8.constData(), nullptr, true);
	}

	catch (...)
	{
		if (outError)
			*outError = QStringLiteral("Invalid action plan JSON.");

		return false;
	}

	PluginHostContext& mutableHost = const_cast<PluginHostContext&>(host);

	if (root.contains("action") && root.value("action", "") == "create_mesh")
	{
		QString err;

		QString extra;

		QString backendId;

		if (!executeCreateMeshStep(mutableHost, root, &err, &backendId, &extra))
		{
			if (outError)
				*outError = err;

			return false;
		}

		if (outSummary)
		{
			nlohmann::json summaryCmd = root;

			AiMeshDefaults::applyMissingDimensions(summaryCmd);

			*outSummary = QStringLiteral("已根据 AI 指令创建网格。");

			const QString dims = AiMeshDefaults::summarizeDimensionsMm(summaryCmd);

			if (!dims.isEmpty())
				*outSummary += QStringLiteral("\n尺寸：%1").arg(dims);

			if (!extra.isEmpty())
				*outSummary += QStringLiteral("\n") + extra;
		}

		return true;
	}

	const int ver = root.value("version", 0);

	if (ver != 2 || !root.contains("steps") || !root["steps"].is_array())
	{
		if (outError)
			*outError = QStringLiteral("Expected action plan version 2 with steps[].");

		return false;
	}

	AiPlanStepContext ctx;

	ctx.ephemeralCompose = planUsesEphemeralCompose(root);

	int okCount = 0;

	for (const auto& step : root["steps"])
	{
		QString err;

		if (!executeStep(mutableHost, step, ctx, &err))
		{
			if (outError)
				*outError = QStringLiteral("Step %1 failed: %2").arg(okCount).arg(err);

			return false;
		}

		++okCount;
	}

	if (outSummary)
	{
		if (!ctx.clarifyNotes.isEmpty())
		{
			*outSummary = QStringLiteral("生成前需澄清：\n- %1").arg(ctx.clarifyNotes.join(QStringLiteral("\n- ")));
		}
		else if (ctx.ephemeralCompose)
			*outSummary = QStringLiteral("已执行 %1 个步骤（布尔多步编排，仅注册最终结果）。").arg(okCount);
		else if (root.value("domain", "") == "feature.compose")
			*outSummary = QStringLiteral("已执行 %1 个参数化特征步骤。").arg(okCount);
		else
			*outSummary = QStringLiteral("已执行 %1 个步骤（布尔多步编排）。").arg(okCount);
	}

	return true;
}

} // namespace AiActionPlanExecutor
