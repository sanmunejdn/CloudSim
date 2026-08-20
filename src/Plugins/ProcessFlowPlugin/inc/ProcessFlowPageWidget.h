#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWPAGEWIDGET_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWPAGEWIDGET_H

/// @file ProcessFlowPageWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 中央流程页：顶栏工具 + 画布

#include <QWidget>

class ProcessFlowCanvasWidget;
class QCheckBox;
class QPushButton;

class ProcessFlowPageWidget final : public QWidget
{
	Q_OBJECT

public:
	explicit ProcessFlowPageWidget(QWidget* parent = nullptr);

	ProcessFlowCanvasWidget* canvas() const { return m_canvas; }
	void applyLanguage(bool useChinese);

private:
	ProcessFlowCanvasWidget* m_canvas = nullptr;
	QCheckBox* m_connectCheck = nullptr;
	QCheckBox* m_gridCheck = nullptr;
	QPushButton* m_layoutBtn = nullptr;
	QPushButton* m_fitBtn = nullptr;
	QPushButton* m_deleteBtn = nullptr;
	QPushButton* m_exportBtn = nullptr;
};

#endif // PROCESSFLOWPLUGIN_PROCESSFLOWPAGEWIDGET_H
