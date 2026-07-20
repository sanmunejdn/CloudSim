#ifndef WIDGET_RUNINFOPAGE_H
#define WIDGET_RUNINFOPAGE_H

/// @file RunInfoPage.h
/// @brief 运行信息页：以只读文本显示日志、警告与错误，只读展示运行输出

#include "widget_global.h"

#include <QWidget>

class QPlainTextEdit;
class QPushButton;

/// 运行信息页：以只读文本显示日志、警告与错误，只读展示运行输出
class WIDGET_EXPORT RunInfoPage : public QWidget
{
	Q_OBJECT

public:
	explicit RunInfoPage(QWidget* parent = nullptr);
	~RunInfoPage() override;

	void appendInfo(const QString& text);
	void appendWarning(const QString& text);
	void appendError(const QString& text);
	void clearLogs();
	void setUiLanguage(bool useChinese);

private:
	void appendLine(const QString& level, const QString& text);

private:
	QPlainTextEdit* m_logEdit = nullptr;
	QPushButton* m_clearBtn = nullptr;
};

#endif // WIDGET_RUNINFOPAGE_H
