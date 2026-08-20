#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWSIMSIDEWIDGET_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWSIMSIDEWIDGET_H

/// @file ProcessFlowSimSideWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 右侧：JobSet 编辑 + 仿真报表

#include <QWidget>

class ProcessFlowJobSetPanel;
class ProcessFlowReportPanel;

class ProcessFlowSimSideWidget final : public QWidget
{
	Q_OBJECT

public:
	explicit ProcessFlowSimSideWidget(QWidget* parent = nullptr);

	void applyLanguage(bool useChinese);
	ProcessFlowJobSetPanel* jobSetPanel() const { return m_jobSetPanel; }
	ProcessFlowReportPanel* reportPanel() const { return m_reportPanel; }

private:
	ProcessFlowJobSetPanel* m_jobSetPanel = nullptr;
	ProcessFlowReportPanel* m_reportPanel = nullptr;
};

#endif
