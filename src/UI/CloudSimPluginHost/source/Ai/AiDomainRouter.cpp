#include "Ai/AiDomainRouter.h"

#include "AiDomainTypes.h"

AiDomainRouter::AiDomainRouter(const IAiDomainRegistry* registry)
	: m_registry(registry)
{
}

QString AiDomainRouter::resolve(const QString& requestedDomainId, const QString& userText) const
{
	const QString req = requestedDomainId.trimmed();
	if (!req.isEmpty() && req != AiDomainIds::autoDomain())
		return req;

	const QString t = userText;
	if (t.contains(QStringLiteral("识别")) || t.contains(QStringLiteral("是什么形状"))
		|| t.contains(QStringLiteral("选中")) || t.contains(QStringLiteral("recognize"), Qt::CaseInsensitive))
	{
		return AiDomainIds::geometryRecognize();
	}
	if (t.contains(QStringLiteral("挖")) || t.contains(QStringLiteral("通孔")) || t.contains(QStringLiteral("盲孔"))
		|| t.contains(QStringLiteral("布尔")) || t.contains(QStringLiteral("差集")) || t.contains(QStringLiteral("并集"))
		|| t.contains(QStringLiteral("相交")) || t.contains(QStringLiteral("boolean"), Qt::CaseInsensitive))
	{
		return AiDomainIds::meshCompose();
	}
	if (t.contains(QStringLiteral("导入")) || t.contains(QStringLiteral("import"), Qt::CaseInsensitive))
		return AiDomainIds::documentImport();
	if (t.contains(QStringLiteral("点云")) || t.contains(QStringLiteral("下采样")) || t.contains(QStringLiteral("point cloud"), Qt::CaseInsensitive))
		return AiDomainIds::pointCloudOps();
	if (t.contains(QStringLiteral("轨迹")) || t.contains(QStringLiteral("焊缝")) || t.contains(QStringLiteral("涂胶"))
		|| t.contains(QStringLiteral("打磨")) || t.contains(QStringLiteral("trajectory"), Qt::CaseInsensitive))
		return AiDomainIds::trajectoryFeature();

	return AiDomainIds::meshCreate();
}
