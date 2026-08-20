#ifndef ROBOTWIDGET_USERTEMPLATELIBRARY_H
#define ROBOTWIDGET_USERTEMPLATELIBRARY_H

/// @file UserTemplateLibrary.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 用户命名模板库（流水线 / 离散策略参数）

#include "robotwidget_global.h"

#include <QString>
#include <QVector>
#include <json.hpp>

enum class UserTemplateKind
{
	Pipeline,
	Discretize
};

struct ROBOTWIDGET_EXPORT UserTemplateEntry
{
	QString id;
	QString name;
	QString updatedAtIso;
};

/// AppData 下命名模板 CRUD；导入导出整文件
class ROBOTWIDGET_EXPORT UserTemplateLibrary
{
public:
	static QString templatesRoot(UserTemplateKind kind);
	static QVector<UserTemplateEntry> list(UserTemplateKind kind);
	static bool save(UserTemplateKind kind, const QString& name, const nlohmann::json& payload, QString* outId = nullptr,
					 QString* outError = nullptr);
	static bool load(UserTemplateKind kind, const QString& id, nlohmann::json* outPayload, QString* outName = nullptr,
					 QString* outError = nullptr);
	static bool remove(UserTemplateKind kind, const QString& id, QString* outError = nullptr);
	static bool importFile(UserTemplateKind kind, const QString& filePath, QString* outId = nullptr,
						   QString* outError = nullptr);
	static bool exportFile(UserTemplateKind kind, const QString& id, const QString& filePath,
						   QString* outError = nullptr);

	/// 将旧单槽 QSettings pipelineJson 迁为命名模板（幂等）
	static void migrateLegacyPipelineSlot();
};

#endif // ROBOTWIDGET_USERTEMPLATELIBRARY_H
