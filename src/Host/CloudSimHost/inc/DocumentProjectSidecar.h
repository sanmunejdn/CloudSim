#ifndef CLOUDSIMHOST_DOCUMENTPROJECTSIDECAR_H
#define CLOUDSIMHOST_DOCUMENTPROJECTSIDECAR_H

/// @file DocumentProjectSidecar.h
/// @brief 工程旁路表（与 Data SSOT 分离的文档级镜像）

#include "cloudsim_host_global.h"

#include <QMap>
#include <QString>

namespace cloudsim::host
{
/// 源路径/类型/逻辑父等工程侧车，避免继续堆在 DocumentHost 字段区
class CLOUDSIM_HOST_EXPORT DocumentProjectSidecar
{
public:
	QMap<QString, QString>& sourcePath() { return m_sourcePath; }
	const QMap<QString, QString>& sourcePath() const { return m_sourcePath; }
	QMap<QString, QString>& sourceType() { return m_sourceType; }
	const QMap<QString, QString>& sourceType() const { return m_sourceType; }
	QMap<QString, QString>& parentId() { return m_parentId; }
	const QMap<QString, QString>& parentId() const { return m_parentId; }

	void setProjectFilePath(const QString& path) { m_projectFilePath = path; }
	const QString& projectFilePath() const { return m_projectFilePath; }

private:
	QMap<QString, QString> m_sourcePath;
	QMap<QString, QString> m_sourceType;
	QMap<QString, QString> m_parentId;
	QString m_projectFilePath;
};

} // namespace cloudsim::host

#endif
