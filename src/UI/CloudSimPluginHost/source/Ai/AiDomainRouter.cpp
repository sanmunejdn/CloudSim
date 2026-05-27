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
	if (t.contains(QStringLiteral("导入")) || t.contains(QStringLiteral("import"), Qt::CaseInsensitive))
		return AiDomainIds::documentImport();
	if (t.contains(QStringLiteral("点云")) || t.contains(QStringLiteral("下采样")) || t.contains(QStringLiteral("point cloud"), Qt::CaseInsensitive))
		return AiDomainIds::pointCloudOps();

	return AiDomainIds::meshCreate();
}
