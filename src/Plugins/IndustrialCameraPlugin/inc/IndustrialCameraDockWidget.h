#ifndef INDUSTRIALCAMERAPLUGIN_INDUSTRIALCAMERADOCKWIDGET_H
#define INDUSTRIALCAMERAPLUGIN_INDUSTRIALCAMERADOCKWIDGET_H

/// @file IndustrialCameraDockWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 工业相机侧栏根控件：内嵌 Tab（相机 / 手眼标定）

#include <QWidget>

class IPluginHostContext;
class QTabWidget;
class CameraPanelWidget;
class HandEyePanelWidget;

class IndustrialCameraDockWidget : public QWidget
{
	Q_OBJECT
public:
	explicit IndustrialCameraDockWidget(IPluginHostContext* host, QWidget* parent = nullptr);

	void setUseChinese(bool zh);
	void applyLanguage();

	CameraPanelWidget* cameraPanel() const { return cameraPanel_; }
	HandEyePanelWidget* handEyePanel() const { return handEyePanel_; }

private:
	IPluginHostContext* host_ = nullptr;
	bool zh_ = true;
	QTabWidget* tabs_ = nullptr;
	CameraPanelWidget* cameraPanel_ = nullptr;
	HandEyePanelWidget* handEyePanel_ = nullptr;
};

#endif // INDUSTRIALCAMERAPLUGIN_INDUSTRIALCAMERADOCKWIDGET_H
