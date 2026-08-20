#ifndef ROBOTWIDGET_AXISCONTROLTARGETSERVICE_H
#define ROBOTWIDGET_AXISCONTROLTARGETSERVICE_H

/// @file AxisControlTargetService.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 轴控目标列表与切换时滑条填充

#include "robotwidget_global.h"

#include <QObject>
#include <QString>
#include <QVector>

class IRobotMainWindowHost;
class RobotSimulationController;
enum class AxisControlTargetKind : int;

class ROBOTWIDGET_EXPORT AxisControlTargetService : public QObject
{
	Q_OBJECT

public:
	explicit AxisControlTargetService(QObject* parent = nullptr);

	void setHost(IRobotMainWindowHost* host);
	/// 读聚合关节角 / sync 外轴滑条
	void setSimulationController(RobotSimulationController* controller);

	void refreshTargets();
	void onTargetChanged(AxisControlTargetKind kind, const QString& id);

signals:
	void catalogChanged();

private:
	IRobotMainWindowHost* m_host = nullptr;
	RobotSimulationController* m_controller = nullptr;
};

#endif // ROBOTWIDGET_AXISCONTROLTARGETSERVICE_H
