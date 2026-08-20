#ifndef CLOUDSIMPLUGINHOST_AIAGENTPICKDIALOG_H
#define CLOUDSIMPLUGINHOST_AIAGENTPICKDIALOG_H

/// @file AiAgentPickDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AI 执行缺参时弹出：选后端对象 / 选文件路径

#include <QString>
#include <QStringList>
#include <QWidget>
#include <vector>

class IPluginDocument;

namespace AiAgentPickDialog
{
enum class BackendKindFilter
{
	Any,
	PointCloud,
	Mesh,
	Brep,
	PointCloudOrMesh,
	BrepOrMesh
};

struct BackendEntry
{
	QString id;
	QString label;
	QString className;
};

std::vector<BackendEntry> listBackends(IPluginDocument* doc, BackendKindFilter filter);

/// 选一个对象；取消返回 false
bool pickOneBackend(QWidget* parent, const std::vector<BackendEntry>& entries, const QString& title, QString* outId);

/// 选源 + 目标（配准等）；取消返回 false
bool pickSourceAndTarget(QWidget* parent, const std::vector<BackendEntry>& entries, const QString& title,
						 QString* outSourceId, QString* outTargetId);

/// 打开/保存文件；取消返回 false
bool pickOpenFilePath(QWidget* parent, const QString& title, const QString& filter, QString* outPath);
bool pickSaveFilePath(QWidget* parent, const QString& title, const QString& filter, const QString& defaultName,
					  QString* outPath);
bool pickExistingDirectory(QWidget* parent, const QString& title, QString* outDir);
} // namespace AiAgentPickDialog

#endif
