/// @file AiIntentParser.cpp
/// @brief AiIntentParser 实现

#include "AiIntentParser.h"

#include "Ai/AiCommandSchema.h"
#include "Ai/AiMeshDefaults.h"

#include <QRegularExpression>
#include <QString>
#include <vector>

namespace AiIntentParser

{
namespace

{
QString norm(const QString& t)

{
	return t.trimmed();
}

std::vector<double> extractNumbersMm(const QString& text)

{
	std::vector<double> out;

	QRegularExpression re(QStringLiteral("(?:直径|半径|长|宽|高|底面)?\\s*(\\d+(?:\\.\\d+)?)\\s*(?:mm|毫米)?"),

						  QRegularExpression::CaseInsensitiveOption);

	QRegularExpression reMul(
		QStringLiteral("(\\d+(?:\\.\\d+)?)\\s*[xX×*]\\s*(\\d+(?:\\.\\d+)?)\\s*[xX×*]\\s*(\\d+(?:\\.\\d+)?)"));

	auto mMul = reMul.match(text);

	if (mMul.hasMatch())

	{
		out.push_back(mMul.captured(1).toDouble());

		out.push_back(mMul.captured(2).toDouble());

		out.push_back(mMul.captured(3).toDouble());

		return out;
	}

	QRegularExpression reLwh(QStringLiteral(

		"长\\s*宽\\s*高\\s*(?:为|是)?\\s*(\\d+(?:\\.\\d+)?)\\s*[,，、]\\s*(\\d+(?:\\.\\d+)?)\\s*[,，、]\\s*(\\d+(?:\\."
		"\\d+)?)"));

	auto mLwh = reLwh.match(text);

	if (mLwh.hasMatch())

	{
		out.push_back(mLwh.captured(1).toDouble());

		out.push_back(mLwh.captured(2).toDouble());

		out.push_back(mLwh.captured(3).toDouble());

		return out;
	}

	QRegularExpression rePlain(QStringLiteral("(\\d+(?:\\.\\d+)?)"));

	auto it = rePlain.globalMatch(text);

	while (it.hasNext())

	{
		const auto m = it.next();

		out.push_back(m.captured(1).toDouble());
	}

	return out;
}

enum class Detected
{
	None,
	Box,
	Cylinder,
	Cone,
	Sphere
};

Detected detectKind(const QString& t)

{
	const QString s = t.toLower();

	if (s.contains(QStringLiteral("长方体")) || s.contains(QStringLiteral("盒子")) ||
		s.contains(QStringLiteral("立方体"))

		|| s.contains(QStringLiteral("box")))

		return Detected::Box;

	if (s.contains(QStringLiteral("圆柱")) || s.contains(QStringLiteral("圆筒")) ||
		s.contains(QStringLiteral("cylinder")))

		return Detected::Cylinder;

	if (s.contains(QStringLiteral("圆锥")) || s.contains(QStringLiteral("cone")))

		return Detected::Cone;

	if (s.contains(QStringLiteral("球")) || s.contains(QStringLiteral("sphere")))

		return Detected::Sphere;

	return Detected::None;
}

bool hasCreateVerb(const QString& t)

{
	return t.contains(QStringLiteral("生成")) || t.contains(QStringLiteral("创建")) ||
		   t.contains(QStringLiteral("建立"))

		   || t.contains(QStringLiteral("做一个")) || t.contains(QStringLiteral("create"), Qt::CaseInsensitive)

		   || t.contains(QStringLiteral("make"), Qt::CaseInsensitive);
}

bool hasHoleIntent(const QString& t)
{
	return t.contains(QStringLiteral("通孔")) || t.contains(QStringLiteral("穿孔")) ||
		   t.contains(QStringLiteral("钻孔")) ||
		   (t.contains(QStringLiteral("挖")) && (t.contains(QStringLiteral("孔")) || t.contains(QStringLiteral("洞"))));
}

bool isBoxStockPhrase(const QString& t)
{
	return t.contains(QStringLiteral("长方体")) || t.contains(QStringLiteral("立方体")) ||
		   t.contains(QStringLiteral("盒子"));
}

double findDiameterMm(const QString& t, const std::vector<double>& nums)
{
	QRegularExpression re(QStringLiteral("(?:直径|φ|Φ)\\s*(?:为|是)?\\s*(\\d+(?:\\.\\d+)?)"));
	const auto m = re.match(t);
	if (m.hasMatch())
		return m.captured(1).toDouble();
	QRegularExpression reD(QStringLiteral("\\bd\\s*(\\d+(?:\\.\\d+)?)"), QRegularExpression::CaseInsensitiveOption);
	const auto md = reD.match(t);
	if (md.hasMatch())
		return md.captured(1).toDouble();
	if (t.contains(QStringLiteral("直径")) && nums.size() >= 4)
		return nums[3];
	if (t.contains(QStringLiteral("直径")) && !nums.empty())
		return nums.back();
	return -1.0;
}

double findLabeled(const QString& t, const QStringList& labels, const std::vector<double>& nums, int fallbackIndex)

{
	for (const QString& lab : labels)

	{
		QRegularExpression re(QStringLiteral("%1\\s*(\\d+(?:\\.\\d+)?)").arg(QRegularExpression::escape(lab)));

		auto m = re.match(t);

		if (m.hasMatch())

			return m.captured(1).toDouble();
	}

	if (fallbackIndex >= 0 && static_cast<std::size_t>(fallbackIndex) < nums.size())

		return nums[static_cast<std::size_t>(fallbackIndex)];

	return -1.0;
}

void putDimIfPositive(nlohmann::json& dims, const char* key, double v)

{
	if (v > 0.0)

		dims[key] = v;
}

void finalizeMeshCmd(nlohmann::json& cmd, ParseResult& r)

{
	bool usedDefaults = false;

	AiMeshDefaults::applyMissingDimensions(cmd, &usedDefaults);

	if (usedDefaults)

		r.hintMessage = AiMeshDefaults::defaultsAppliedNote(cmd, true);

	const auto& dims = cmd["dimensions_mm"];

	const std::string prim = cmd.value("primitive", "");

	if (prim == "box")

	{
		cmd["name"] = QString("Box_%1x%2x%3")

						  .arg(dims.value("length", 0.0))

						  .arg(dims.value("width", 0.0))

						  .arg(dims.value("height", 0.0))

						  .toStdString();
	}

	else if (prim == "cylinder")

	{
		cmd["name"] = QString("Cylinder_R%1_H%2")

						  .arg(dims.value("radius", 0.0))

						  .arg(dims.value("height", 0.0))

						  .toStdString();
	}

	else if (prim == "cone")

	{
		cmd["name"] = QString("Cone_R%1_H%2")

						  .arg(dims.value("radius", 0.0))

						  .arg(dims.value("height", 0.0))

						  .toStdString();
	}

	else if (prim == "sphere")

		cmd["name"] = QStringLiteral("Sphere").toStdString();

	r.ok = true;

	r.command = std::move(cmd);
}

} // namespace

ParseResult tryParseUserText(const QString& textIn)

{
	ParseResult r;

	const QString t = norm(textIn);

	if (t.isEmpty())

	{
		r.errorMessage = QStringLiteral("请输入描述，例如：生成长方体，或：生成长方体，长100mm，宽50mm，高100mm");

		return r;
	}

	if (!hasCreateVerb(t))

	{
		r.errorMessage = QStringLiteral("请使用「生成/创建」等动词描述要创建的基本体。");

		r.hintMessage = QStringLiteral("支持：长方体、圆柱、圆锥、球体；尺寸可省略（将使用默认值）。");

		return r;
	}

	const Detected kind = detectKind(t);

	if (kind == Detected::None)

	{
		r.errorMessage = QStringLiteral("未识别基本体类型。");

		r.hintMessage = QStringLiteral("支持：长方体、圆柱、圆锥、球体。");

		return r;
	}

	const std::vector<double> nums = extractNumbersMm(t);

	nlohmann::json cmd;

	cmd["version"] = 1;

	cmd["action"] = "create_mesh";

	cmd["dimensions_mm"] = nlohmann::json::object();

	switch (kind)

	{
	case Detected::Box:

	{
		cmd["primitive"] = "box";

		double L = findLabeled(t, {QStringLiteral("长")}, nums, 0);

		double W = findLabeled(t, {QStringLiteral("宽")}, nums, 1);

		double H = findLabeled(t, {QStringLiteral("高")}, nums, 2);

		if (L < 0 && nums.size() >= 3)

		{
			L = nums[0];

			W = nums[1];

			H = nums[2];
		}

		putDimIfPositive(cmd["dimensions_mm"], "length", L);

		putDimIfPositive(cmd["dimensions_mm"], "width", W);

		putDimIfPositive(cmd["dimensions_mm"], "height", H);

		finalizeMeshCmd(cmd, r);

		break;
	}

	case Detected::Cylinder:

	{
		cmd["primitive"] = "cylinder";

		double R = findLabeled(t, {QStringLiteral("半径")}, nums, 0);

		double H = findLabeled(t, {QStringLiteral("高")}, nums, 1);

		if (t.contains(QStringLiteral("直径")) && nums.size() >= 1)

			R = nums[0] * 0.5;

		if (R < 0 && nums.size() >= 2)

		{
			R = nums[0];

			H = nums[1];
		}

		putDimIfPositive(cmd["dimensions_mm"], "radius", R);

		putDimIfPositive(cmd["dimensions_mm"], "height", H);

		finalizeMeshCmd(cmd, r);

		break;
	}

	case Detected::Cone:

	{
		cmd["primitive"] = "cone";

		double R = findLabeled(t, {QStringLiteral("半径"), QStringLiteral("底面")}, nums, 0);

		double H = findLabeled(t, {QStringLiteral("高")}, nums, 1);

		if (R < 0 && nums.size() >= 2)

		{
			R = nums[0];

			H = nums[1];
		}

		putDimIfPositive(cmd["dimensions_mm"], "radius", R);

		putDimIfPositive(cmd["dimensions_mm"], "height", H);

		finalizeMeshCmd(cmd, r);

		break;
	}

	case Detected::Sphere:

	{
		cmd["primitive"] = "sphere";

		double R = findLabeled(t, {QStringLiteral("半径")}, nums, 0);

		if (t.contains(QStringLiteral("直径")) && nums.size() >= 1)

			cmd["dimensions_mm"]["diameter"] = nums[0];

		else

			putDimIfPositive(cmd["dimensions_mm"], "radius", R);

		if (nums.size() >= 1 && !cmd["dimensions_mm"].contains("radius") && !cmd["dimensions_mm"].contains("diameter"))

			putDimIfPositive(cmd["dimensions_mm"], "radius", nums[0]);

		finalizeMeshCmd(cmd, r);

		break;
	}

	default:

		break;
	}

	return r;
}

ParseResult tryParseComposeUserText(const QString& textIn)
{
	ParseResult r;
	const QString t = norm(textIn);
	if (t.isEmpty())
	{
		r.errorMessage = QStringLiteral("请输入描述，例如：生成长方体 100×100×200，顶部挖直径 50 通孔。");
		return r;
	}
	if (!hasCreateVerb(t))
	{
		r.errorMessage = QStringLiteral("请使用「生成/创建」等动词。");
		return r;
	}
	if (!hasHoleIntent(t))
	{
		r.errorMessage = QStringLiteral("未识别通孔/挖孔意图。");
		return r;
	}
	if (!isBoxStockPhrase(t))
	{
		r.errorMessage = QStringLiteral("通孔规则解析暂仅支持长方体坯料（长方体/立方体/盒子）。");
		return r;
	}

	const std::vector<double> nums = extractNumbersMm(t);
	double L = findLabeled(t, {QStringLiteral("长")}, nums, 0);
	double W = findLabeled(t, {QStringLiteral("宽")}, nums, 1);
	double H = findLabeled(t, {QStringLiteral("高")}, nums, 2);
	if (L < 0 && nums.size() >= 3)
	{
		L = nums[0];
		W = nums[1];
		H = nums[2];
	}
	if (L <= 0.0 || W <= 0.0 || H <= 0.0)
	{
		r.errorMessage = QStringLiteral("请给出长方体长宽高（mm）。");
		return r;
	}

	const double diam = findDiameterMm(t, nums);
	if (diam <= 0.0)
	{
		r.errorMessage = QStringLiteral("请给出通孔直径（mm）。");
		return r;
	}
	const double R = diam * 0.5;
	const double cylH = H * 1.2 + 20.0;

	nlohmann::json plan;
	plan["version"] = 2;
	plan["domain"] = "mesh.compose";
	plan["steps"] = nlohmann::json::array();
	plan["steps"].push_back({
		{"id", "body"},
		{"api", "createPrimitiveMesh"},
		{"args",
		 {
			 {"primitive", "box"},
			 {"dimensions_mm", {{"length", L}, {"width", W}, {"height", H}}},
			 {"name", "Body"},
			 {"mesh_quality", {{"segments", 32}}},
		 }},
	});
	plan["steps"].push_back({
		{"id", "hole_tool"},
		{"api", "createPrimitiveMesh"},
		{"args",
		 {
			 {"primitive", "cylinder"},
			 {"dimensions_mm", {{"radius", R}, {"height", cylH}}},
			 {"name", "HoleTool"},
			 {"mesh_quality", {{"segments", 32}}},
		 }},
	});
	plan["steps"].push_back({
		{"id", "result"},
		{"api", "booleanMesh"},
		{"args",
		 {
			 {"op", "difference"},
			 {"target", "$body"},
			 {"tool", "$hole_tool"},
			 {"result_name", "BoxWithHole"},
			 {"hide_operands", true},
		 }},
	});

	AiCommandSchema::normalizeComposePlanJson(plan);
	r.ok = true;
	r.command = std::move(plan);
	r.hintMessage = QStringLiteral("已用规则生成 box+cylinder 通孔编排。");
	return r;
}

ParseResult tryParseFeatureComposeUserText(const QString& textIn)
{
	ParseResult r;
	const QString t = norm(textIn);
	if (t.isEmpty())
	{
		r.errorMessage = QStringLiteral("请输入描述，例如：拉伸长方体 100×80×40。");
		return r;
	}

	// 旋转圆柱：矩形截面绕 +Y 360°
	if (t.contains(QStringLiteral("旋转")) &&
		(t.contains(QStringLiteral("圆柱")) || t.contains(QStringLiteral("回转"))))
	{
		const std::vector<double> nums = extractNumbersMm(t);
		double R = findLabeled(t, {QStringLiteral("半径")}, nums, 0);
		if (R <= 0.0)
		{
			const double D = findDiameterMm(t, nums);
			if (D > 0.0)
				R = D * 0.5;
		}
		double H = findLabeled(t, {QStringLiteral("高")}, nums, 1);
		if (R <= 0.0 || H <= 0.0)
		{
			r.errorMessage = QStringLiteral("旋转圆柱请给出半径/直径与高度（mm）。");
			r.hintMessage = QStringLiteral("例如：旋转圆柱 半径50 高100。");
			return r;
		}
		nlohmann::json plan;
		plan["version"] = 2;
		plan["domain"] = "feature.compose";
		plan["steps"] = nlohmann::json::array();
		plan["steps"].push_back({
			{"id", "body"},
			{"api", "revolveSketchProfileToBrep"},
			{"args",
			 {
				 {"mode", "boss"},
				 {"profile", "rectangle"},
				 {"length_mm", R},
				 {"width_mm", H},
				 {"angle_deg", 360.0},
				 {"axis_dx", 0.0},
				 {"axis_dy", 1.0},
				 {"axis_dz", 0.0},
				 {"name", "RevolveBody"},
			 }},
		});
		r.ok = true;
		r.command = std::move(plan);
		r.hintMessage = QStringLiteral("已用规则生成 Revolve 圆柱特征计划。");
		return r;
	}

	const std::vector<double> nums = extractNumbersMm(t);
	double L = findLabeled(t, {QStringLiteral("长")}, nums, 0);
	double W = findLabeled(t, {QStringLiteral("宽")}, nums, 1);
	double H = findLabeled(t, {QStringLiteral("高")}, nums, 2);
	if (L < 0 && nums.size() >= 3)
	{
		L = nums[0];
		W = nums[1];
		H = nums[2];
	}
	if (L <= 0.0 || W <= 0.0 || H <= 0.0)
	{
		r.errorMessage = QStringLiteral("请给出长×宽×高（mm），例如：建模 100x80x40 板。");
		r.hintMessage = QStringLiteral("也可交给本地/远程 LLM（feature.compose）。");
		return r;
	}

	nlohmann::json plan;
	plan["version"] = 2;
	plan["domain"] = "feature.compose";
	plan["steps"] = nlohmann::json::array();
	plan["steps"].push_back({
		{"id", "body"},
		{"api", "extrudeSketchProfileToBrep"},
		{"args",
		 {
			 {"mode", "pad"},
			 {"profile", "rectangle"},
			 {"length_mm", L},
			 {"width_mm", W},
			 {"extrude_mm", H},
			 {"name", "Body"},
		 }},
	});

	if (hasHoleIntent(t))
	{
		double dia = findDiameterMm(t, nums);
		if (dia <= 0.0)
		{
			QRegularExpression reHole(QStringLiteral("孔\\s*(?:直径|d|D)?\\s*(\\d+(?:\\.\\d+)?)"));
			const auto mh = reHole.match(t);
			if (mh.hasMatch())
				dia = mh.captured(1).toDouble();
		}
		if (dia <= 0.0)
		{
			r.errorMessage = QStringLiteral("通孔请给出直径（mm），例如：建模 100x80x40 中心通孔 d10。");
			return r;
		}
		plan["steps"].push_back({
			{"id", "hole"},
			{"api", "extrudeSketchProfileToBrep"},
			{"args",
			 {
				 {"mode", "pocket"},
				 {"profile", "circle"},
				 {"diameter_mm", dia},
				 {"center_u_mm", L * 0.5},
				 {"center_v_mm", W * 0.5},
				 {"end_condition", "through_all"},
				 {"extrude_mm", H},
				 {"target", "$body"},
				 {"name", "ThroughHole"},
			 }},
		});
		r.ok = true;
		r.command = std::move(plan);
		r.hintMessage = QStringLiteral("已用规则生成 Pad+Pocket 通孔特征计划。");
		return r;
	}

	r.ok = true;
	r.command = std::move(plan);
	r.hintMessage = QStringLiteral("已用规则生成 Pad 特征计划。");
	return r;
}

} // namespace AiIntentParser
