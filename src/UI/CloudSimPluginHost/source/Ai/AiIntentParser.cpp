#include "AiIntentParser.h"



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

	QRegularExpression reMul(QStringLiteral("(\\d+(?:\\.\\d+)?)\\s*[xX×*]\\s*(\\d+(?:\\.\\d+)?)\\s*[xX×*]\\s*(\\d+(?:\\.\\d+)?)"));

	auto mMul = reMul.match(text);

	if (mMul.hasMatch())

	{

		out.push_back(mMul.captured(1).toDouble());

		out.push_back(mMul.captured(2).toDouble());

		out.push_back(mMul.captured(3).toDouble());

		return out;

	}

	QRegularExpression reLwh(QStringLiteral(

		"长\\s*宽\\s*高\\s*(?:为|是)?\\s*(\\d+(?:\\.\\d+)?)\\s*[,，、]\\s*(\\d+(?:\\.\\d+)?)\\s*[,，、]\\s*(\\d+(?:\\.\\d+)?)"));

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



enum class Detected { None, Box, Cylinder, Cone, Sphere };



Detected detectKind(const QString& t)

{

	const QString s = t.toLower();

	if (s.contains(QStringLiteral("长方体")) || s.contains(QStringLiteral("盒子")) || s.contains(QStringLiteral("立方体"))

		|| s.contains(QStringLiteral("box")))

		return Detected::Box;

	if (s.contains(QStringLiteral("圆柱")) || s.contains(QStringLiteral("圆筒")) || s.contains(QStringLiteral("cylinder")))

		return Detected::Cylinder;

	if (s.contains(QStringLiteral("圆锥")) || s.contains(QStringLiteral("cone")))

		return Detected::Cone;

	if (s.contains(QStringLiteral("球")) || s.contains(QStringLiteral("sphere")))

		return Detected::Sphere;

	return Detected::None;

}



bool hasCreateVerb(const QString& t)

{

	return t.contains(QStringLiteral("生成")) || t.contains(QStringLiteral("创建")) || t.contains(QStringLiteral("建立"))

		|| t.contains(QStringLiteral("做一个")) || t.contains(QStringLiteral("create"), Qt::CaseInsensitive)

		|| t.contains(QStringLiteral("make"), Qt::CaseInsensitive);

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

}



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

		double L = findLabeled(t, { QStringLiteral("长") }, nums, 0);

		double W = findLabeled(t, { QStringLiteral("宽") }, nums, 1);

		double H = findLabeled(t, { QStringLiteral("高") }, nums, 2);

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

		double R = findLabeled(t, { QStringLiteral("半径") }, nums, 0);

		double H = findLabeled(t, { QStringLiteral("高") }, nums, 1);

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

		double R = findLabeled(t, { QStringLiteral("半径"), QStringLiteral("底面") }, nums, 0);

		double H = findLabeled(t, { QStringLiteral("高") }, nums, 1);

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

		double R = findLabeled(t, { QStringLiteral("半径") }, nums, 0);

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



}

