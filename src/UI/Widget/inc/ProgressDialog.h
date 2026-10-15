#ifndef WIDGET_PROGRESSDIALOG_H
#define WIDGET_PROGRESSDIALOG_H

/// @file ProgressDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 通用进度对话框（标题 / 文案 / 进度条 / 可选取消）

#include "widget_global.h"

#include <QDialog>
#include <QString>

class QLabel;
class QProgressBar;
class QPushButton;

class WIDGET_EXPORT ProgressDialog : public QDialog
{
	Q_OBJECT

public:
	explicit ProgressDialog(QWidget* parent = nullptr);

	void setJobTitle(const QString& title);
	void setStatusText(const QString& text);
	void setFraction(double fraction01);
	void setIndeterminate(bool on);
	void setCancelEnabled(bool enabled);
	void setExtraJobsHint(int otherCount);
	void setChinese(bool useChinese);

signals:
	void cancelRequested();

private:
	QLabel* m_titleLabel = nullptr;
	QLabel* m_statusLabel = nullptr;
	QLabel* m_extraLabel = nullptr;
	QProgressBar* m_bar = nullptr;
	QPushButton* m_cancelBtn = nullptr;
	bool m_useChinese = true;
};

#endif // WIDGET_PROGRESSDIALOG_H
