/// @file AiIntentClassifier.cpp
/// @brief 规则打分意图分类 + 可选本地小模型选域

#include "Ai/AiIntentClassifier.h"

#include "AiDomainTypes.h"
#include "AiLlmClient.h"

#include <QMap>
#include <utility>
#include <vector>

namespace AiIntentClassifier
{
namespace
{
void addScore(QMap<QString, int>& scores, const QString& domain, int delta)
{
	if (delta == 0 || domain.isEmpty())
		return;
	scores[domain] += delta;
}

bool hasAny(const QString& t, std::initializer_list<const char*> keys, Qt::CaseSensitivity cs = Qt::CaseSensitive)
{
	for (const char* k : keys)
	{
		if (t.contains(QString::fromUtf8(k), cs))
			return true;
	}
	return false;
}
} // namespace

Result classifyByRules(const QString& userText, const int minScore)
{
	Result out;
	const QString t = userText.trimmed();
	if (t.isEmpty())
		return out;

	QMap<QString, int> scores;

	if (hasAny(t, {"轨迹特征", "识别焊缝", "识别边", "焊缝边", "涂胶轨迹", "打磨面", "打磨轨迹", "线特征", "面特征",
				   "线特征识别", "面特征识别", "特征识别"},
			   Qt::CaseInsensitive) ||
		(hasAny(t, {"轨迹"}) && hasAny(t, {"识别", "特征", "焊缝", "涂胶", "打磨", "离散"})) ||
		(hasAny(t, {"焊缝", "涂胶"}) && hasAny(t, {"识别", "边", "特征", "轨迹"})))
		addScore(scores, AiDomainIds::trajectoryFeature(), 3);
	// 「重新识别」 alone：无轴信息，交给会话层复用上次指令，此处不抢域

	if (hasAny(t, {"长方体", "正方体", "立方体", "圆柱", "圆锥", "球体", "基本体"}) ||
		(hasAny(t, {"球"}) && hasAny(t, {"生成", "创建", "来一个", "做一个"})))
		addScore(scores, AiDomainIds::meshCreate(), 3);
	if (hasAny(t, {"生成网格", "创建网格", "create mesh"}, Qt::CaseInsensitive))
		addScore(scores, AiDomainIds::meshCreate(), 2);

	if ((hasAny(t, {"识别形状", "是什么形状", "recognize"}, Qt::CaseInsensitive) ||
		 (hasAny(t, {"识别"}) && hasAny(t, {"基本体", "形体", "视口", "截图"}))) &&
		scores.value(AiDomainIds::trajectoryFeature()) == 0)
		addScore(scores, AiDomainIds::geometryRecognize(), 3);

	const bool holeCue = hasAny(t, {"挖孔", "通孔", "盲孔", "穿孔", "钻孔"});
	const bool booleanCue = hasAny(t, {"布尔", "差集", "并集", "相交", "boolean"}, Qt::CaseInsensitive);
	const bool featureStrong =
		hasAny(t, {"拉伸", "凸台", "切除", "草图", "圆角", "倒角", "阵列", "放样", "抽壳", "拔模", "建模",
				   "生成模型", "text-to-cad", "特征链"},
			   Qt::CaseInsensitive);
	const bool padStockCue = hasAny(t, {"长方体", "正方体", "立方体", "盒子", "板"});
	// 「建模 … 通孔 d50」/「长方体 … 通孔」优先参数化 Pad+Pocket，压过纯 mesh.create
	if (holeCue && (featureStrong || padStockCue))
		addScore(scores, AiDomainIds::featureCompose(), 4);
	else if (featureStrong)
		addScore(scores, AiDomainIds::featureCompose(), 2);
	if (booleanCue || (holeCue && hasAny(t, {"网格", "mesh", "实体", "布尔"}, Qt::CaseInsensitive)))
		addScore(scores, AiDomainIds::meshCompose(), 3);

	if (hasAny(t, {"中心线", "模板点位", "区域划分", "特征构建", "centerline"}, Qt::CaseInsensitive))
		addScore(scores, AiDomainIds::featureBuild(), 3);

	if (hasAny(t, {"离散生成网格", "线面求交", "面面求交", "管状网格", "带状网格", "点选边", "点选面", "Pick Edge",
				   "Pick Face"},
			   Qt::CaseInsensitive))
		addScore(scores, AiDomainIds::geometryOps(), 3);

	if (hasAny(t, {"标注", "刷选", "套索", "PointNet", "labeling"}, Qt::CaseInsensitive))
		addScore(scores, AiDomainIds::labelingAnnot(), 2);

	if (hasAny(t, {"导入模型", "导入文件", "打开模型", "import", "Open Model"}, Qt::CaseInsensitive) ||
		(hasAny(t, {"导入"}) && hasAny(t, {"step", "stp", "ply", "点云", "网格", "文件"}, Qt::CaseInsensitive)))
		addScore(scores, AiDomainIds::documentImport(), 3);

	if (hasAny(t, {"点云", "下采样", "体素", "配准", "点云匹配", "Poisson", "网格简化", "曲面重构", "point cloud",
				   "ICP", "SPARE"},
			   Qt::CaseInsensitive))
		addScore(scores, AiDomainIds::pointCloudOps(), 3);

	if (hasAny(t, {"工艺流程", "产线", "工位", "节拍", "JobSet", "process flow", "processflow"}, Qt::CaseInsensitive))
		addScore(scores, AiDomainIds::processFlow(), 3);

	if (hasAny(t, {"标准件", "六角螺栓", "六角螺母", "平垫", "垫圈", "圆柱销", "齿轮毛坯", "hex bolt", "hex nut",
				   "washer", "dowel"},
			   Qt::CaseInsensitive) ||
		(hasAny(t, {"螺栓", "螺母", "垫圈", "销"}) && hasAny(t, {"创建", "生成", "来一个", "做一个", "M"})) ||
		(hasAny(t, {"齿轮"}) && hasAny(t, {"模数", "齿数", "毛坯"})))
		addScore(scores, AiDomainIds::designParts(), 4);

	const bool sceneVerb = hasAny(t, {"删除全部", "清空场景", "删除对象", "删除选中", "删掉选中"}) ||
						   hasAny(t, {"Delete all", "Clear scene"}, Qt::CaseInsensitive);
	const bool sceneMove = (hasAny(t, {"平移", "移动", "旋转", "位姿"}) ||
							hasAny(t, {"Translate", "Rotate"}, Qt::CaseInsensitive)) &&
						   hasAny(t, {"选中", "对象", "场景", "沿", "绕", "mm", "毫米", "度"});
	if (sceneVerb || sceneMove)
		addScore(scores, AiDomainIds::sceneOps(), 3);

	QString bestId;
	int bestScore = 0;
	for (auto it = scores.begin(); it != scores.end(); ++it)
	{
		if (it.value() > bestScore)
		{
			bestScore = it.value();
			bestId = it.key();
		}
	}
	if (bestScore < minScore)
		return out;
	out.domainId = bestId;
	out.score = bestScore;
	out.via = QStringLiteral("rules_score");
	return out;
}

Result classifyByLocalLlm(const QString& userText, const AiConfigDto& config)
{
	Result out;
	const QString t = userText.trimmed();
	if (t.isEmpty())
		return out;

	AiLlmConfig llm;
	llm.enabled = true;
	llm.baseUrl = config.router.baseUrl.isEmpty() ? QStringLiteral("http://127.0.0.1:11434/v1") : config.router.baseUrl;
	llm.model = config.router.localModel.isEmpty() ? QStringLiteral("qwen2.5:3b") : config.router.localModel;
	llm.timeoutMs = 30000;
	llm.temperature = 0.0;

	const QString system = QStringLiteral(
		"你是 CloudSim 意图分类器。只输出一个 domain id，不要解释。可选：\n"
		"mesh.create, mesh.compose, feature.compose, design.parts, geometry.recognize, trajectory.feature,\n"
		"pointcloud.ops, document.import, geometry.ops, feature.build, labeling.annot,\n"
		"scene.ops, process.flow\n"
		"若无法判断，输出 unknown。");
	const auto lr = AiLlmClient::chatText(system, t, llm, {});
	if (!lr.ok)
		return out;
	QString id = lr.assistantText.trimmed();
	id.remove(QLatin1Char('"'));
	id.remove(QLatin1Char('`'));
	if (id.contains(QLatin1Char('\n')))
		id = id.split(QLatin1Char('\n')).first().trimmed();
	if (id.isEmpty() || id.compare(QStringLiteral("unknown"), Qt::CaseInsensitive) == 0)
		return out;

	static const QStringList kAllowed = {AiDomainIds::meshCreate(),		 AiDomainIds::meshCompose(),
										 AiDomainIds::featureCompose(),	 AiDomainIds::geometryRecognize(),
										 AiDomainIds::trajectoryFeature(), AiDomainIds::pointCloudOps(),
										 AiDomainIds::documentImport(),	 AiDomainIds::geometryOps(),
										 AiDomainIds::featureBuild(),	 AiDomainIds::labelingAnnot(),
										 AiDomainIds::sceneOps(),		 AiDomainIds::processFlow()};
	if (!kAllowed.contains(id))
		return out;
	out.domainId = id;
	out.score = config.router.minScore;
	out.via = QStringLiteral("local_classify");
	return out;
}
} // namespace AiIntentClassifier
