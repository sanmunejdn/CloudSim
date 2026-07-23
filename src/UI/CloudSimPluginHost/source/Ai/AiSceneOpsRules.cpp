/// @file AiSceneOpsRules.cpp
/// @brief 多段增量/删除口语拆成可重复同 api 的步骤

#include "Ai/AiSceneOpsRules.h"

#include <QRegularExpression>

#include <algorithm>
#include <json.hpp>
#include <vector>

namespace AiSceneOpsRules
{
namespace
{
QString selectedBackendId(const QByteArray& snapUtf8)
{
	try
	{
		const auto j = nlohmann::json::parse(snapUtf8.constData(), nullptr, true);
		if (j.contains("selected_backend_id") && j["selected_backend_id"].is_string())
			return QString::fromStdString(j["selected_backend_id"].get<std::string>());
	}
	catch (...)
	{
	}
	return {};
}

double toNumber(const QString& s)
{
	bool ok = false;
	const double v = s.trimmed().toDouble(&ok);
	return ok ? v : 0.0;
}

AiAgentPlanStep makeStep(const QString& apiId, const nlohmann::json& args, const QString& rationale)
{
	AiAgentPlanStep s;
	s.apiId = apiId;
	s.argsJson = QByteArray::fromStdString(args.dump());
	s.rationale = rationale;
	return s;
}

struct OrderedStep
{
	int pos = 0;
	AiAgentPlanStep step;
};

bool looksLikeSceneOps(const QString& t)
{
	return t.contains(QStringLiteral("删除")) || t.contains(QStringLiteral("清空")) ||
		   t.contains(QStringLiteral("删掉")) || t.contains(QStringLiteral("平移")) ||
		   t.contains(QStringLiteral("移动")) || t.contains(QStringLiteral("旋转")) ||
		   t.contains(QStringLiteral("Delete"), Qt::CaseInsensitive) ||
		   t.contains(QStringLiteral("Move"), Qt::CaseInsensitive) ||
		   t.contains(QStringLiteral("Translate"), Qt::CaseInsensitive) ||
		   t.contains(QStringLiteral("Rotate"), Qt::CaseInsensitive) ||
		   t.contains(QStringLiteral("Clear"), Qt::CaseInsensitive);
}
} // namespace

AiAgentPlan tryBuildPlan(const QString& userText, const QByteArray& sceneSnapshotUtf8)
{
	AiAgentPlan plan;
	const QString t = userText.trimmed();
	if (t.isEmpty() || !looksLikeSceneOps(t))
		return plan;

	const QString backendId = selectedBackendId(sceneSnapshotUtf8);

	if (t.contains(QStringLiteral("删除全部")) || t.contains(QStringLiteral("清空场景")) ||
		t.contains(QStringLiteral("删除所有")) || t.contains(QStringLiteral("Delete all"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Clear scene"), Qt::CaseInsensitive))
	{
		plan.steps.append(makeStep(QStringLiteral("removeAllSceneObjects"), nlohmann::json::object(),
								   QStringLiteral("清空场景")));
		plan.summary = QStringLiteral("删除全部对象");
		return plan;
	}

	std::vector<OrderedStep> ordered;

	const QRegularExpression translateRe(
		QStringLiteral(
			R"((?:沿\s*([XYZxyz])\s*(?:轴)?\s*(?:平移|移动|移)|往?\s*(上|下|左|右|前|后)\s*(?:移|移动|平移)?|(?:Translate|Move)\s*([XYZxyz]))\s*(-?\d+(?:\.\d+)?)\s*(?:mm|毫米)?)"),
		QRegularExpression::CaseInsensitiveOption);
	auto tit = translateRe.globalMatch(t);
	while (tit.hasNext())
	{
		const QRegularExpressionMatch m = tit.next();
		nlohmann::json args = nlohmann::json::object();
		if (!backendId.isEmpty())
			args["backend_id"] = backendId.toStdString();
		const double v = toNumber(m.captured(4));
		QString axis = m.captured(1);
		if (axis.isEmpty())
			axis = m.captured(3);
		const QString dir = m.captured(2);
		double dx = 0, dy = 0, dz = 0;
		if (!axis.isEmpty())
		{
			const QChar c = axis.at(0).toUpper();
			if (c == QLatin1Char('X'))
				dx = v;
			else if (c == QLatin1Char('Y'))
				dy = v;
			else
				dz = v;
		}
		else if (dir == QStringLiteral("上") || dir == QStringLiteral("前"))
			dz = v;
		else if (dir == QStringLiteral("下") || dir == QStringLiteral("后"))
			dz = -v;
		else if (dir == QStringLiteral("右"))
			dx = v;
		else if (dir == QStringLiteral("左"))
			dx = -v;
		args["dx_mm"] = dx;
		args["dy_mm"] = dy;
		args["dz_mm"] = dz;
		OrderedStep os;
		os.pos = m.capturedStart(0);
		os.step = makeStep(QStringLiteral("translateSceneObject"), args,
						   QStringLiteral("平移 Δ(%1,%2,%3)").arg(dx).arg(dy).arg(dz));
		ordered.push_back(std::move(os));
	}

	const QRegularExpression rotateRe(
		QStringLiteral(
			R"((?:绕\s*([XYZxyz])\s*(?:轴)?\s*旋转|(?:Rotate)\s*(?:about\s*)?([XYZxyz]))\s*(-?\d+(?:\.\d+)?)\s*(?:度|°|deg)?)"),
		QRegularExpression::CaseInsensitiveOption);
	auto rit = rotateRe.globalMatch(t);
	while (rit.hasNext())
	{
		const QRegularExpressionMatch m = rit.next();
		nlohmann::json args = nlohmann::json::object();
		if (!backendId.isEmpty())
			args["backend_id"] = backendId.toStdString();
		QString axis = m.captured(1);
		if (axis.isEmpty())
			axis = m.captured(2);
		const double v = toNumber(m.captured(3));
		double rx = 0, ry = 0, rz = 0;
		if (!axis.isEmpty())
		{
			const QChar c = axis.at(0).toUpper();
			if (c == QLatin1Char('X'))
				rx = v;
			else if (c == QLatin1Char('Y'))
				ry = v;
			else
				rz = v;
		}
		args["rx_deg"] = rx;
		args["ry_deg"] = ry;
		args["rz_deg"] = rz;
		OrderedStep os;
		os.pos = m.capturedStart(0);
		os.step = makeStep(QStringLiteral("rotateSceneObject"), args,
						   QStringLiteral("旋转 Δ(%1,%2,%3)°").arg(rx).arg(ry).arg(rz));
		ordered.push_back(std::move(os));
	}

	const bool deleteOne = t.contains(QStringLiteral("删除选中")) || t.contains(QStringLiteral("删除对象")) ||
						   t.contains(QStringLiteral("删掉")) ||
						   t.contains(QStringLiteral("Delete object"), Qt::CaseInsensitive) ||
						   t.contains(QStringLiteral("Remove object"), Qt::CaseInsensitive) ||
						   (t.contains(QStringLiteral("删除")) && !t.contains(QStringLiteral("删除全部")));
	if (deleteOne)
	{
		nlohmann::json args = nlohmann::json::object();
		if (!backendId.isEmpty())
			args["backend_id"] = backendId.toStdString();
		int pos = t.indexOf(QStringLiteral("删除"));
		if (pos < 0)
			pos = t.indexOf(QStringLiteral("删掉"));
		if (pos < 0)
			pos = t.size();
		OrderedStep os;
		os.pos = pos;
		os.step = makeStep(QStringLiteral("removeSceneObject"), args, QStringLiteral("删除对象"));
		ordered.push_back(std::move(os));
	}

	std::sort(ordered.begin(), ordered.end(),
			  [](const OrderedStep& a, const OrderedStep& b) { return a.pos < b.pos; });
	for (auto& os : ordered)
		plan.steps.append(std::move(os.step));

	if (plan.steps.isEmpty())
	{
		if (t.contains(QStringLiteral("旋转")) || t.contains(QStringLiteral("Rotate"), Qt::CaseInsensitive))
		{
			nlohmann::json args = nlohmann::json::object();
			if (!backendId.isEmpty())
				args["backend_id"] = backendId.toStdString();
			args["rx_deg"] = 0;
			args["ry_deg"] = 0;
			args["rz_deg"] = 0;
			plan.steps.append(makeStep(QStringLiteral("rotateSceneObject"), args, QStringLiteral("旋转")));
		}
		else if (t.contains(QStringLiteral("平移")) || t.contains(QStringLiteral("移动")) ||
				 t.contains(QStringLiteral("Move"), Qt::CaseInsensitive) ||
				 t.contains(QStringLiteral("Translate"), Qt::CaseInsensitive))
		{
			nlohmann::json args = nlohmann::json::object();
			if (!backendId.isEmpty())
				args["backend_id"] = backendId.toStdString();
			args["dx_mm"] = 0;
			args["dy_mm"] = 0;
			args["dz_mm"] = 0;
			plan.steps.append(makeStep(QStringLiteral("translateSceneObject"), args, QStringLiteral("平移")));
		}
	}

	if (plan.steps.size() == 1)
		plan.summary = plan.steps.front().rationale;
	else if (plan.steps.size() > 1)
	{
		QStringList bits;
		for (const auto& s : plan.steps)
			bits << s.rationale;
		plan.summary = bits.join(QStringLiteral(" → "));
	}
	return plan;
}
} // namespace AiSceneOpsRules
