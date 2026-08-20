#ifndef ROBOTWIDGET_MOTIONPATHPLANDIALOG_H
#define ROBOTWIDGET_MOTIONPATHPLANDIALOG_H

/// @file MotionPathPlanDialog.h
/// @brief 运动规划：下拉选择起终点路点

#include "robotwidget_global.h"

#include <QDialog>
#include <QString>
#include <QVector>

class QComboBox;
class QLabel;
class QPushButton;

struct ROBOTWIDGET_EXPORT MotionPathWaypointItem
{
	QString id;
	QString label;
};

class ROBOTWIDGET_EXPORT MotionPathPlanDialog : public QDialog
{
	Q_OBJECT

public:
	explicit MotionPathPlanDialog(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setWaypoints(const QVector<MotionPathWaypointItem>& items);
	void selectWaypointIds(const QString& startId, const QString& endId);
	void setStatusText(const QString& text);
	void setConfirmEnabled(bool enabled);

	QString selectedStartId() const;
	QString selectedEndId() const;

signals:
	void planClicked();
	void clearPreviewClicked();
	void confirmInsertClicked();

private:
	void retranslateUi();

	QLabel* m_startLabel = nullptr;
	QLabel* m_endLabel = nullptr;
	QComboBox* m_startCombo = nullptr;
	QComboBox* m_endCombo = nullptr;
	QPushButton* m_planBtn = nullptr;
	QPushButton* m_clearBtn = nullptr;
	QPushButton* m_confirmBtn = nullptr;
	QPushButton* m_closeBtn = nullptr;
	QLabel* m_statusLabel = nullptr;
	QLabel* m_hintLabel = nullptr;
	bool m_chinese = true;
};

#endif // ROBOTWIDGET_MOTIONPATHPLANDIALOG_H
