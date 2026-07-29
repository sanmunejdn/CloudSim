/// @file AiDomainRouter.cpp
/// @brief AiDomainRouter 实现

#include "Ai/AiDomainRouter.h"

#include "AiDomainTypes.h"

AiDomainRouter::AiDomainRouter(const IAiDomainRegistry* registry) : m_registry(registry) {}

QString AiDomainRouter::resolve(const QString& requestedDomainId, const QString& userText) const
{
	const QString req = requestedDomainId.trimmed();
	if (!req.isEmpty() && req != AiDomainIds::autoDomain())
		return req;

	const QString t = userText;
	const bool trajectoryCue = t.contains(QStringLiteral("轨迹")) || t.contains(QStringLiteral("焊缝")) ||
							   t.contains(QStringLiteral("涂胶")) || t.contains(QStringLiteral("打磨")) ||
							   t.contains(QStringLiteral("trajectory"), Qt::CaseInsensitive) ||
							   t.contains(QStringLiteral("边")) || t.contains(QStringLiteral("面特征")) ||
							   t.contains(QStringLiteral("线特征"));
	const bool primitiveCue = t.contains(QStringLiteral("长方体")) || t.contains(QStringLiteral("圆柱")) ||
							  t.contains(QStringLiteral("圆锥")) || t.contains(QStringLiteral("球")) ||
							  t.contains(QStringLiteral("基本体"));

	if (trajectoryCue && !primitiveCue)
		return AiDomainIds::trajectoryFeature();

	if ((t.contains(QStringLiteral("识别")) || t.contains(QStringLiteral("是什么形状")) ||
		 t.contains(QStringLiteral("recognize"), Qt::CaseInsensitive)) &&
		!trajectoryCue)
	{
		return AiDomainIds::geometryRecognize();
	}
	if (t.contains(QStringLiteral("挖")) || t.contains(QStringLiteral("通孔")) || t.contains(QStringLiteral("盲孔")) ||
		t.contains(QStringLiteral("布尔")) || t.contains(QStringLiteral("差集")) ||
		t.contains(QStringLiteral("并集")) || t.contains(QStringLiteral("相交")) ||
		t.contains(QStringLiteral("boolean"), Qt::CaseInsensitive))
	{
		return AiDomainIds::meshCompose();
	}
	if (t.contains(QStringLiteral("拉伸")) || t.contains(QStringLiteral("法兰")) ||
		t.contains(QStringLiteral("支架")) || t.contains(QStringLiteral("零件")) ||
		t.contains(QStringLiteral("凸台")) || t.contains(QStringLiteral("阵列")) ||
		t.contains(QStringLiteral("圆角")) || t.contains(QStringLiteral("倒角")) ||
		t.contains(QStringLiteral("切除")) || t.contains(QStringLiteral("草图")) ||
		t.contains(QStringLiteral("text-to-cad"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("生成模型")) || t.contains(QStringLiteral("建模")) ||
		t.contains(QStringLiteral("特征")))
	{
		return AiDomainIds::featureCompose();
	}

	if (t.contains(QStringLiteral("中心线")) || t.contains(QStringLiteral("模板点位")) ||
		t.contains(QStringLiteral("区域划分")) || t.contains(QStringLiteral("特征构建")) ||
		t.contains(QStringLiteral("centerline"), Qt::CaseInsensitive))
		return AiDomainIds::featureBuild();

	// 几何「点选边/面」须先于标注泛化「点选」
	if (t.contains(QStringLiteral("离散生成网格")) || t.contains(QStringLiteral("线面求交")) ||
		t.contains(QStringLiteral("面面求交")) || t.contains(QStringLiteral("管状网格")) ||
		t.contains(QStringLiteral("带状网格")) || t.contains(QStringLiteral("点选边")) ||
		t.contains(QStringLiteral("点选面")) || t.contains(QStringLiteral("点选 F")) ||
		t.contains(QStringLiteral("Pick Edge"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Pick Face"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Discretize"), Qt::CaseInsensitive))
		return AiDomainIds::geometryOps();

	if (t.contains(QStringLiteral("标注")) || t.contains(QStringLiteral("刷选")) || t.contains(QStringLiteral("套索")) ||
		t.contains(QStringLiteral("PointNet")) || t.contains(QStringLiteral("labeling"), Qt::CaseInsensitive) ||
		(t.contains(QStringLiteral("点选")) && !t.contains(QStringLiteral("点选边")) &&
		 !t.contains(QStringLiteral("点选面"))))
		return AiDomainIds::labelingAnnot();

	if (t.contains(QStringLiteral("导入")) || t.contains(QStringLiteral("打开模型")) ||
		t.contains(QStringLiteral("import"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Open Model"), Qt::CaseInsensitive))
		return AiDomainIds::documentImport();

	if (t.contains(QStringLiteral("点云")) || t.contains(QStringLiteral("下采样")) ||
		t.contains(QStringLiteral("配准")) || t.contains(QStringLiteral("匹配")) ||
		t.contains(QStringLiteral("Poisson")) || t.contains(QStringLiteral("网格简化")) ||
		t.contains(QStringLiteral("曲面重构")) || t.contains(QStringLiteral("point cloud"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("ICP"), Qt::CaseInsensitive) || t.contains(QStringLiteral("SPARE"), Qt::CaseInsensitive))
		return AiDomainIds::pointCloudOps();

	// 工艺流程 / 产线仿真须先于默认 mesh.create
	if (t.contains(QStringLiteral("工艺流程")) || t.contains(QStringLiteral("产线")) ||
		t.contains(QStringLiteral("工位")) || t.contains(QStringLiteral("节拍")) ||
		t.contains(QStringLiteral("流水线")) || t.contains(QStringLiteral("仿真产线")) ||
		t.contains(QStringLiteral("DES")) || t.contains(QStringLiteral("JobSet"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("process flow"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("processflow"), Qt::CaseInsensitive) ||
		(t.contains(QStringLiteral("仿真")) &&
		 (t.contains(QStringLiteral("缓冲")) || t.contains(QStringLiteral("装配")) ||
		  t.contains(QStringLiteral("调度")) || t.contains(QStringLiteral("LPT")) ||
		  t.contains(QStringLiteral("FIFO")) || t.contains(QStringLiteral("SPT")))))
		return AiDomainIds::processFlow();

	// 场景删/移/转须先于默认 mesh.create
	if (t.contains(QStringLiteral("删除全部")) || t.contains(QStringLiteral("清空场景")) ||
		t.contains(QStringLiteral("删除对象")) || t.contains(QStringLiteral("删除选中")) ||
		t.contains(QStringLiteral("删掉")) || t.contains(QStringLiteral("Delete all"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Clear scene"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Delete object"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Remove object"), Qt::CaseInsensitive) || t.contains(QStringLiteral("平移")) ||
		t.contains(QStringLiteral("移动")) || t.contains(QStringLiteral("旋转")) ||
		t.contains(QStringLiteral("位姿")) || t.contains(QStringLiteral("Move"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Translate"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("Rotate"), Qt::CaseInsensitive))
		return AiDomainIds::sceneOps();

	return AiDomainIds::meshCreate();
}
