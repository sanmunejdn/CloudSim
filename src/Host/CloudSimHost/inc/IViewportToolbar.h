#ifndef CLOUDSIMHOST_IVIEWPORTTOOLBAR_H
#define CLOUDSIMHOST_IVIEWPORTTOOLBAR_H

/// @file IViewportToolbar.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口浮动工具栏挂载与快捷操作（IOsgWidgetView 切面）

class QWidget;

/// 聚焦/线框/截图与 overlay 宿主控件
class IViewportToolbar
{
public:
	virtual ~IViewportToolbar() = default;

	/// 视口浮动工具栏挂载与快捷操作（避免 Widget 层直连 OsgWidget）
	virtual QWidget* viewportOverlayHostWidget() const = 0;
	virtual void onViewportFocusRequested() = 0;
	virtual void setWireframeMode(bool enabled) = 0;
	virtual void onViewportScreenshotRequested() = 0;
};

#endif // CLOUDSIMHOST_IVIEWPORTTOOLBAR_H
