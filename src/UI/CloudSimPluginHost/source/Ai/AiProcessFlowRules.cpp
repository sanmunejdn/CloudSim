/// @file AiProcessFlowRules.cpp
/// @brief 常见产线口语映射为 apply + run 两步

#include "Ai/AiProcessFlowRules.h"

#include <QRegularExpression>

#include <json.hpp>

namespace AiProcessFlowRules
{
namespace
{
AiAgentPlanStep makeStep(const QString& apiId, const nlohmann::json& args, const QString& rationale)
{
	AiAgentPlanStep s;
	s.apiId = apiId;
	s.argsJson = QByteArray::fromStdString(args.dump());
	s.rationale = rationale;
	return s;
}

nlohmann::json nodeProps(const char* kind, double cycle = 0, double capacity = 0, double scrap = 0,
						 double inventory = 0)
{
	nlohmann::json p = nlohmann::json::object();
	p["kind"] = kind;
	p["cycleTimeSec"] = cycle;
	p["inventoryQty"] = inventory;
	p["capacityQty"] = capacity;
	p["setupTimeSec"] = 0;
	p["priority"] = 0;
	p["batchSize"] = 1;
	p["scrapRate"] = scrap;
	p["mtbfSec"] = 0;
	p["mttrSec"] = 0;
	p["requiredInputs"] = 2;
	p["binding"] = {{"backendId", ""}, {"programId", ""}};
	return p;
}

nlohmann::json makeNode(int id, const QString& title, const QString& subtitle, const char* color, double x, double y,
						const nlohmann::json& props)
{
	return {{"id", id},
			{"title", title.toStdString()},
			{"subtitle", subtitle.toStdString()},
			{"color", color},
			{"x", x},
			{"y", y},
			{"width", 160},
			{"height", 72},
			{"props", props}};
}

QString detectPolicy(const QString& t)
{
	if (t.contains(QStringLiteral("LPT"), Qt::CaseInsensitive))
		return QStringLiteral("lpt");
	if (t.contains(QStringLiteral("SPT"), Qt::CaseInsensitive))
		return QStringLiteral("spt");
	if (t.contains(QStringLiteral("EDD"), Qt::CaseInsensitive))
		return QStringLiteral("edd");
	if (t.contains(QStringLiteral("CR"), Qt::CaseInsensitive) && !t.contains(QStringLiteral("FIFO")))
		return QStringLiteral("cr");
	return QStringLiteral("fifo");
}

double detectHorizon(const QString& t)
{
	const QRegularExpression re(QStringLiteral(R"((\d+(?:\.\d+)?)\s*(?:小时|h|H))"));
	const QRegularExpressionMatch m = re.match(t);
	if (m.hasMatch())
		return m.captured(1).toDouble() * 3600.0;
	const QRegularExpression reMin(QStringLiteral(R"((\d+(?:\.\d+)?)\s*(?:分钟|min))"));
	const QRegularExpressionMatch mm = reMin.match(t);
	if (mm.hasMatch())
		return mm.captured(1).toDouble() * 60.0;
	return 3600.0;
}

int detectStationCount(const QString& t)
{
	const QRegularExpression re(QStringLiteral(R"(([一二三四五六七八九十两\d]+)\s*(?:条|个)?\s*工位)"));
	const QRegularExpressionMatch m = re.match(t);
	if (!m.hasMatch())
		return 3;
	const QString c = m.captured(1);
	if (c == QStringLiteral("一") || c == QStringLiteral("1"))
		return 1;
	if (c == QStringLiteral("二") || c == QStringLiteral("两") || c == QStringLiteral("2"))
		return 2;
	if (c == QStringLiteral("三") || c == QStringLiteral("3"))
		return 3;
	if (c == QStringLiteral("四") || c == QStringLiteral("4"))
		return 4;
	if (c == QStringLiteral("五") || c == QStringLiteral("5"))
		return 5;
	bool ok = false;
	const int n = c.toInt(&ok);
	return (ok && n >= 1 && n <= 8) ? n : 3;
}

bool looksLikeProcessFlow(const QString& t)
{
	return t.contains(QStringLiteral("工艺流程")) || t.contains(QStringLiteral("产线")) ||
		   t.contains(QStringLiteral("工位")) || t.contains(QStringLiteral("流水线")) ||
		   t.contains(QStringLiteral("process flow"), Qt::CaseInsensitive) ||
		   (t.contains(QStringLiteral("仿真")) &&
			(t.contains(QStringLiteral("缓冲")) || t.contains(QStringLiteral("装配")) ||
			 t.contains(QStringLiteral("调度"))));
}

nlohmann::json buildLinearLine(int stationCount, double cycleSec, double bufferCap, bool withInspect, double scrap)
{
	nlohmann::json nodes = nlohmann::json::array();
	nlohmann::json edges = nlohmann::json::array();
	int id = 1;
	const int startId = id++;
	nodes.push_back(makeNode(startId, QStringLiteral("开始"), QStringLiteral("流程入口"), "#2E7DD1", 40, 120,
							 nodeProps("start", 30)));

	int prev = startId;
	double x = 240;
	for (int i = 0; i < stationCount; ++i)
	{
		if (bufferCap > 0)
		{
			const int bufId = id++;
			nodes.push_back(makeNode(bufId, QStringLiteral("缓冲"), QStringLiteral("在制品"), "#F0A202", x, 120,
									 nodeProps("buffer", 0, bufferCap, 0, 0)));
			edges.push_back({{"from", prev}, {"to", bufId}, {"label", ""}});
			prev = bufId;
			x += 200;
		}
		const int stId = id++;
		nodes.push_back(makeNode(stId, QStringLiteral("工位%1").arg(i + 1), QStringLiteral("加工"), "#2E7D32", x, 120,
								 nodeProps("station", cycleSec, 1)));
		edges.push_back({{"from", prev}, {"to", stId}, {"label", ""}});
		prev = stId;
		x += 200;
	}
	if (withInspect)
	{
		const int inspId = id++;
		nodes.push_back(makeNode(inspId, QStringLiteral("检测"), QStringLiteral("质量检验"), "#8E24AA", x, 120,
								 nodeProps("inspect", cycleSec * 0.5, 1, scrap)));
		edges.push_back({{"from", prev}, {"to", inspId}, {"label", ""}});
		prev = inspId;
		x += 200;
	}
	const int endId = id++;
	nodes.push_back(makeNode(endId, QStringLiteral("结束"), QStringLiteral("流程出口"), "#546E7A", x, 120, nodeProps("end")));
	edges.push_back({{"from", prev}, {"to", endId}, {"label", ""}});

	return {{"version", 1}, {"nodes", nodes}, {"edges", edges}, {"jobSet", nlohmann::json::object()}};
}
} // namespace

AiAgentPlan tryBuildPlan(const QString& userText)
{
	AiAgentPlan plan;
	const QString t = userText.trimmed();
	if (t.isEmpty() || !looksLikeProcessFlow(t))
		return plan;

	// 短句增量改属性，避免整图替换
	const QRegularExpression setCycleRe(
		QStringLiteral(R"(工位\s*(\d+)\s*(?:节拍|加工|周期)\s*(?:改|设|为|到)?\s*(\d+(?:\.\d+)?))"));
	const QRegularExpressionMatch setCycle = setCycleRe.match(t);
	if (setCycle.hasMatch())
	{
		const int nodeId = setCycle.captured(1).toInt();
		const double cycle = setCycle.captured(2).toDouble();
		nlohmann::json ops = nlohmann::json::array();
		ops.push_back({{"op", "setNodeProp"},
					   {"nodeId", nodeId},
					   {"props", {{"cycleTimeSec", cycle}}}});
		plan.steps.append(makeStep(QStringLiteral("patchProcessFlowGraph"), {{"ops_json", ops.dump()}},
								   QStringLiteral("增量改节拍")));
		if (t.contains(QStringLiteral("仿真")) || t.contains(QStringLiteral("运行")))
		{
			plan.steps.append(makeStep(QStringLiteral("runProcessFlowSimulation"),
									   {{"horizonSec", detectHorizon(t)}, {"policy", detectPolicy(t).toStdString()}},
									   QStringLiteral("运行 DES 仿真")));
		}
		plan.summary = QStringLiteral("改工位%1节拍=%2").arg(nodeId).arg(cycle);
		return plan;
	}
	const QRegularExpression setBufRe(QStringLiteral(R"(缓冲(?:容量)?\s*(?:改|设|为|到)?\s*(\d+(?:\.\d+)?))"));
	const QRegularExpressionMatch setBuf = setBufRe.match(t);
	if (setBuf.hasMatch() && (t.contains(QStringLiteral("改")) || t.contains(QStringLiteral("设"))))
	{
		const double cap = setBuf.captured(1).toDouble();
		nlohmann::json ops = nlohmann::json::array();
		ops.push_back({{"op", "setNodeProp"},
					   {"nodeId", 2},
					   {"props", {{"capacityQty", cap}, {"kind", "buffer"}}}});
		plan.steps.append(makeStep(QStringLiteral("patchProcessFlowGraph"), {{"ops_json", ops.dump()}},
								   QStringLiteral("增量改缓冲")));
		plan.summary = QStringLiteral("改缓冲容量=%1").arg(cap);
		return plan;
	}

	const int stations = detectStationCount(t);
	const bool withInspect = t.contains(QStringLiteral("检测")) || t.contains(QStringLiteral("报废"));
	double scrap = 0.0;
	const QRegularExpression scrapRe(QStringLiteral(R"(报废[率]?\s*(\d+(?:\.\d+)?)\s*%?)"));
	const QRegularExpressionMatch sm = scrapRe.match(t);
	if (sm.hasMatch())
		scrap = sm.captured(1).toDouble() / 100.0;

	double cycle = 30.0;
	const QRegularExpression cycleRe(QStringLiteral(R"(加工\s*(\d+(?:\.\d+)?)\s*s)"));
	const QRegularExpressionMatch cm = cycleRe.match(t);
	if (cm.hasMatch())
		cycle = cm.captured(1).toDouble();

	double bufferCap = 20.0;
	const QRegularExpression bufRe(QStringLiteral(R"(缓冲(?:容量)?\s*(\d+(?:\.\d+)?))"));
	const QRegularExpressionMatch bm = bufRe.match(t);
	if (bm.hasMatch())
		bufferCap = bm.captured(1).toDouble();
	if (t.contains(QStringLiteral("无缓冲")))
		bufferCap = 0;

	const nlohmann::json flow = buildLinearLine(stations, cycle, bufferCap, withInspect, scrap);
	nlohmann::json applyArgs = {{"flow_json", flow.dump()}, {"auto_layout", true}};
	plan.steps.append(makeStep(QStringLiteral("applyProcessFlowGraph"), applyArgs, QStringLiteral("生成工艺流程图")));

	if (t.contains(QStringLiteral("仿真")) || t.contains(QStringLiteral("运行")) ||
		t.contains(QStringLiteral("simulate"), Qt::CaseInsensitive) || t.contains(QStringLiteral("LPT")) ||
		t.contains(QStringLiteral("FIFO")) || t.contains(QStringLiteral("SPT")))
	{
		nlohmann::json runArgs = {{"horizonSec", detectHorizon(t)}, {"policy", detectPolicy(t).toStdString()}};
		plan.steps.append(makeStep(QStringLiteral("runProcessFlowSimulation"), runArgs, QStringLiteral("运行 DES 仿真")));
	}

	plan.summary = QStringLiteral("生成%1工位产线并仿真").arg(stations);
	return plan;
}
} // namespace AiProcessFlowRules
