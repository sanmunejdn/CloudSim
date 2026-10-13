#ifndef CLOUDSIMHOST_DETACHEDVIEWWINDOW_H
#define CLOUDSIMHOST_DETACHEDVIEWWINDOW_H

/// @file DetachedViewWindow.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 浮动视角窗：内嵌 SatelliteOsgView，随文档销毁

#include "widget_global.h"

#include <QDialog>

class OsgWidget;
class SatelliteOsgView;

/// 浮动视角窗：内嵌 SatelliteOsgView，随文档销毁
class OSG_WIDGET_API DetachedViewWindow : public QDialog
{
	Q_OBJECT
public:
	explicit DetachedViewWindow(OsgWidget* primary, QWidget* parent = nullptr);
	~DetachedViewWindow() override;

	SatelliteOsgView* satelliteView() const { return m_satellite; }

private:
	SatelliteOsgView* m_satellite = nullptr;
};

#endif // CLOUDSIMHOST_DETACHEDVIEWWINDOW_H
