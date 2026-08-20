/// @file AiActionPlanExecutor.cpp
/// @brief AiActionPlanExecutor 实现

#include "Ai/AiActionPlanExecutor.h"

#include "Ai/AiFeatureComposeSteps.h"
#include "Ai/AiHostButtonApiDispatch.h"
#include "Ai/AiMeshDefaults.h"
#include "Ai/FeatureComposeDomainHandler.h"
#include "Ai/MeshComposeDomainHandler.h"
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

double argNumber(const nlohmann::json& args, const char* key, double fallback = 0.0)
{
	if (!args.contains(key) || !args[key].is_number())
		return fallback;
	return args[key].get<double>();
}

QString describeProfileZh(const nlohmann::json& args)
{
	const std::string profile = args.value("profile", "rectangle");
	if (profile == "rectangle" || profile == "rect" || profile == "box")
	{
		double L = argNumber(args, "length_mm");
		double W = argNumber(args, "width_mm");
		if (args.contains("dimensions_mm") && args["dimensions_mm"].is_object())
		{
			const auto& d = args["dimensions_mm"];
			L = d.value("length", L);
			W = d.value("width", W);
		}
		if (L > 0.0 && W > 0.0)
			return QStringLiteral("矩形轮廓 %1×%2 mm").arg(L).arg(W);
		return QStringLiteral("矩形轮廓");
	}
	if (profile == "circle" || profile == "disk")
	{
		double R = argNumber(args, "radius_mm");
		double D = argNumber(args, "diameter_mm");
		if (R <= 0.0 && D > 0.0)
			R = D * 0.5;
		if (args.contains("dimensions_mm") && args["dimensions_mm"].is_object())
			R = args["dimensions_mm"].value("radius", R);
		if (R > 0.0)
			return QStringLiteral("圆形轮廓 Ø%1 mm").arg(R * 2.0);
		return QStringLiteral("圆形轮廓");
	}
	if (profile == "polygon")
	{
		const int sides = static_cast<int>(argNumber(args, "sides", 6));
		const double R = argNumber(args, "radius_mm");
		if (R > 0.0)
			return QStringLiteral("%1 边多边形 R%2 mm").arg(sides).arg(R);
		return QStringLiteral("%1 边多边形").arg(sides);
	}
	return QStringLiteral("自定义轮廓");
}

double extrudeLengthMm(const nlohmann::json& args)
{
	if (args.contains("extrude_mm") && args["extrude_mm"].is_number())
		return args["extrude_mm"].get<double>();
	if (args.contains("height_mm") && args["height_mm"].is_number())
		return args["height_mm"].get<double>();
	if (args.contains("dimensions_mm") && args["dimensions_mm"].is_object())
		return args["dimensions_mm"].value("height", 0.0);
	return 0.0;
}

/// 单步 extrude → 草图 + 拉伸/切除 文案（供确认面板与执行摘要）
QStringList describeExtrudeFeatureLines(const nlohmann::json& args, int startIndex)
{
	QStringList lines;
	int n = startIndex;
	const std::string mode = args.value("mode", "pad");
	lines << QStringLiteral("%1. 草图绘制：%2（基准面 XY）").arg(n++).arg(describeProfileZh(args));
	const double H = extrudeLengthMm(args);
	const std::string endCond = args.value("end_condition", args.value("endCondition", "blind"));
	if (mode == "pocket")
	{
		if (endCond == "through_all" || endCond == "ThroughAll" || endCond == "throughAll")
			lines << QStringLiteral("%1. 拉伸切除（Pocket）：贯穿全部").arg(n++);
		else if (H > 0.0)
			lines << QStringLiteral("%1. 拉伸切除（Pocket）：深度 %2 mm").arg(n++).arg(H);
		else
			lines << QStringLiteral("%1. 拉伸切除（Pocket）").arg(n++);
	}
	else
	{
		if (H > 0.0)
			lines << QStringLiteral("%1. 拉伸凸台（Pad）：高度 %2 mm").arg(n++).arg(H);
		else
			lines << QStringLiteral("%1. 拉伸凸台（Pad）").arg(n++);
	}
	return lines;
}

QStringList describeFeatureComposePlanLines(const nlohmann::json& root)
{
	QStringList lines;
	if (!root.contains("steps") || !root["steps"].is_array())
		return lines;
	int n = 1;
	for (const auto& step : root["steps"])
	{
		if (!step.is_object())
			continue;
		const std::string api = step.value("api", "");
		const nlohmann::json args =
			step.contains("args") && step["args"].is_object() ? step["args"] : nlohmann::json::object();
		if (api == "extrudeSketchProfileToBrep")
		{
			const QStringList sub = describeExtrudeFeatureLines(args, n);
			lines += sub;
			n += sub.size();
			continue;
		}
		if (api == "revolveSketchProfileToBrep")
		{
			lines << QStringLiteral("%1. 草图绘制：%2").arg(n++).arg(describeProfileZh(args));
			const double ang = argNumber(args, "angle_deg", 360.0);
			const std::string mode = args.value("mode", "boss");
			if (mode == "cut")
				lines << QStringLiteral("%1. 旋转切除：%2°").arg(n++).arg(ang);
			else
				lines << QStringLiteral("%1. 旋转凸台：%2°").arg(n++).arg(ang);
			continue;
		}
		if (api == "filletEdgesToBrep")
		{
			const double r = argNumber(args, "radius_mm", 1.0);
			lines << QStringLiteral("%1. 圆角：R%2 mm").arg(n++).arg(r);
			continue;
		}
		if (api == "chamferEdgesToBrep")
		{
			const double d = argNumber(args, "distance_mm", argNumber(args, "dist_mm", 1.0));
			lines << QStringLiteral("%1. 倒角：%2 mm").arg(n++).arg(d);
			continue;
		}
		if (api == "linearPatternBodyToBrep")
		{
			const int count = static_cast<int>(argNumber(args, "count", 2));
			lines << QStringLiteral("%1. 线性阵列：%2 个").arg(n++).arg(count);
			continue;
		}
		if (api == "circularPatternBodyToBrep")
		{
			const int count = static_cast<int>(argNumber(args, "count", 2));
			lines << QStringLiteral("%1. 圆周阵列：%2 个").arg(n++).arg(count);
			continue;
		}
		if (api == "sweepSketchProfileToBrep")
		{
			const std::string mode = args.value("mode", "boss");
			lines << QStringLiteral("%1. 草图绘制：%2").arg(n++).arg(describeProfileZh(args));
			lines << (mode == "cut" ? QStringLiteral("%1. 扫描切除").arg(n++) : QStringLiteral("%1. 扫描凸台").arg(n++));
			continue;
		}
		if (api == "loftSketchProfilesToBrep")
		{
			const std::string mode = args.value("mode", "boss");
			lines << (mode == "cut" ? QStringLiteral("%1. 放样切除").arg(n++) : QStringLiteral("%1. 放样凸台").arg(n++));
			continue;
		}
		if (api == "shellFacesToBrep")
		{
			const double t = argNumber(args, "thickness_mm", 1.0);
			lines << QStringLiteral("%1. 抽壳：厚度 %2 mm").arg(n++).arg(t);
			continue;
		}
		if (api == "draftFacesToBrep")
		{
			const double a = argNumber(args, "angle_deg", 1.0);
			lines << QStringLiteral("%1. 拔模：%2°").arg(n++).arg(a);
			continue;
		}
		if (api == "askClarify")
		{
			lines << QStringLiteral("%1. 澄清提问").arg(n++);
			continue;
		}
		if (!api.empty())
			lines << QStringLiteral("%1. %2").arg(n++).arg(QString::fromStdString(api));
	}
	return lines;
}

QString formatCreateStepsBlock(const QStringList& lines)
{
	if (lines.isEmpty())
		return {};
	return QStringLiteral("创建步骤：\n") + lines.join(QLatin1Char('\n'));
}

/// create_mesh box/cylinder → 拟执行的特征步骤（确认前预览）
QString previewStepsFromCreateMeshCmd(const nlohmann::json& cmd)
{
	nlohmann::json c = cmd;
	AiMeshDefaults::applyMissingDimensions(c);
	const std::string prim = c.value("primitive", "box");
	if (!c.contains("dimensions_mm") || !c["dimensions_mm"].is_object())
		return {};
	const auto& dims = c["dimensions_mm"];
	nlohmann::json args = nlohmann::json::object();
	args["mode"] = "pad";
	if (prim == "box")
	{
		args["profile"] = "rectangle";
		args["length_mm"] = dims.value("length", 0.0);
		args["width_mm"] = dims.value("width", 0.0);
		args["extrude_mm"] = dims.value("height", 0.0);
	}
	else if (prim == "cylinder")
	{
		args["profile"] = "circle";
		args["radius_mm"] = dims.value("radius", 0.0);
		args["extrude_mm"] = dims.value("height", 0.0);
	}
	else
		return {};
	return formatCreateStepsBlock(describeExtrudeFeatureLines(args, 1));
}

nlohmann::json createMeshCmdFromAgentArgs(const nlohmann::json& args)
{
	nlohmann::json cmd;
	cmd["version"] = 1;
	cmd["action"] = "create_mesh";
	cmd["primitive"] = args.value("primitive", "box");
	nlohmann::json dim = nlohmann::json::object();
	if (args.contains("dimensions_mm") && args["dimensions_mm"].is_object())
		dim = args["dimensions_mm"];
	else
	{
		if (args.contains("length_mm"))
			dim["length"] = args["length_mm"];
		if (args.contains("width_mm"))
			dim["width"] = args["width_mm"];
		if (args.contains("height_mm"))
			dim["height"] = args["height_mm"];
		if (args.contains("radius_mm"))
			dim["radius"] = args["radius_mm"];
	}
	if (!dim.empty())
		cmd["dimensions_mm"] = dim;
	return cmd;
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

/// box/cylinder → Sketch+Pad（Parametric Body，特征树可见）；cone/sphere 仍走裸 B-rep
bool tryExecuteCreateMeshAsFeatureCompose(PluginHostContext& host, nlohmann::json cmd, QString* outError,
										  QString* outBackendId, QString* outSummaryExtra)
{
	AiMeshDefaults::applyMissingDimensions(cmd);
	if (!cmd.contains("dimensions_mm") || !cmd["dimensions_mm"].is_object())
		return false;

	const std::string prim = cmd.value("primitive", "box");
	const auto& dims = cmd["dimensions_mm"];
	nlohmann::json args = nlohmann::json::object();
	args["mode"] = "pad";
	if (cmd.contains("name") && cmd["name"].is_string())
		args["name"] = cmd["name"];
	else if (prim == "cylinder")
		args["name"] = "Cylinder";
	else
		args["name"] = "Body";

	if (prim == "box")
	{
		const double L = dims.value("length", 0.0);
		const double W = dims.value("width", 0.0);
		const double H = dims.value("height", 0.0);
		if (L <= 0.0 || W <= 0.0 || H <= 0.0)
			return false;
		args["profile"] = "rectangle";
		args["length_mm"] = L;
		args["width_mm"] = W;
		args["extrude_mm"] = H;
	}
	else if (prim == "cylinder")
	{
		const double R = dims.value("radius", 0.0);
		const double H = dims.value("height", 0.0);
		if (R <= 0.0 || H <= 0.0)
			return false;
		args["profile"] = "circle";
		args["radius_mm"] = R;
		args["extrude_mm"] = H;
	}
	else
		return false;

	QHash<QString, QString> stepIds;
	QString ferr;
	if (!AiFeatureComposeSteps::tryExecute(host, "extrudeSketchProfileToBrep", args, "body", stepIds, &ferr) ||
		!ferr.isEmpty())
	{
		if (outError)
			*outError = ferr.isEmpty() ? QStringLiteral("参数化拉伸失败。") : ferr;
		return false;
	}
	if (outBackendId)
		*outBackendId = stepIds.value(QStringLiteral("body"));
	if (outSummaryExtra)
	{
		*outSummaryExtra = formatCreateStepsBlock(describeExtrudeFeatureLines(args, 1));
		if (!outSummaryExtra->isEmpty())
			*outSummaryExtra += QStringLiteral("\n（已写入特征树，可继续圆角/切除等）");
	}
	return true;
}

bool executeCreateMeshStep(PluginHostContext& host, const nlohmann::json& cmdIn, QString* outError,
						   QString* outBackendId, QString* outSummaryExtra = nullptr)
{
	nlohmann::json cmd = cmdIn;
	AiMeshDefaults::applyMissingDimensions(cmd);
	if (tryExecuteCreateMeshAsFeatureCompose(host, cmd, outError, outBackendId, outSummaryExtra))
		return true;
	// cone/sphere 或 Pad 失败：回退裸 B-rep（轨迹工件等仍可用）
	if (outError)
		outError->clear();
	if (outSummaryExtra)
		outSummaryExtra->clear();

	PluginPrimitiveMeshParams pp;

	PluginPrimitiveMeshQuality pq;

	PluginMeshCreateOptions opt;

	if (!parseCreateMeshCommandToPlugin(cmd, pp, pq, opt, outError, outSummaryExtra))
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

			const QString dims = AiMeshDefaults::summarizeDimensionsMm(summaryCmd);
			if (!extra.isEmpty() && extra.startsWith(QStringLiteral("创建步骤")))
			{
				*outSummary = QStringLiteral("已创建参数化实体。");
				if (!dims.isEmpty())
					*outSummary += QStringLiteral("\n尺寸：%1").arg(dims);
				*outSummary += QStringLiteral("\n") + extra;
			}
			else
			{
				*outSummary = QStringLiteral("已根据 AI 指令创建网格。");
				if (!dims.isEmpty())
					*outSummary += QStringLiteral("\n尺寸：%1").arg(dims);
				if (!extra.isEmpty())
					*outSummary += QStringLiteral("\n") + extra;
			}
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

	if (root.value("domain", "") == "feature.compose")
	{
		QString planErr;
		if (!FeatureComposeDomainHandler::validatePlanJson(root, &planErr))
		{
			if (outError)
				*outError = planErr.isEmpty() ? QStringLiteral("feature.compose 计划校验失败。") : planErr;
			return false;
		}
	}
	if (root.value("domain", "") == "mesh.compose")
	{
		QString planErr;
		if (!MeshComposeDomainHandler::validatePlanJson(root, &planErr))
		{
			if (outError)
				*outError = planErr.isEmpty() ? QStringLiteral("mesh.compose 计划校验失败。") : planErr;
			return false;
		}
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
		{
			const QString stepsText = formatCreateStepsBlock(describeFeatureComposePlanLines(root));
			*outSummary = QStringLiteral("已执行 %1 个参数化特征步骤。").arg(okCount);
			if (!stepsText.isEmpty())
				*outSummary += QStringLiteral("\n") + stepsText;
		}
		else
			*outSummary = QStringLiteral("已执行 %1 个步骤（布尔多步编排）。").arg(okCount);
	}

	return true;
}

QString previewCreateMeshFeatureSteps(const QByteArray& createMeshOrArgsJsonUtf8)
{
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(createMeshOrArgsJsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		return {};
	}
	if (!j.is_object())
		return {};
	if (j.value("action", "") == "create_mesh")
		return previewStepsFromCreateMeshCmd(j);
	// Agent args：flat length_mm / 或已是 create_mesh
	return previewStepsFromCreateMeshCmd(createMeshCmdFromAgentArgs(j));
}

} // namespace AiActionPlanExecutor
