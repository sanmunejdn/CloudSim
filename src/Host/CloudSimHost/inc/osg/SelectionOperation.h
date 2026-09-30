#ifndef CLOUDSIMHOST_SELECTIONOPERATION_H
#define CLOUDSIMHOST_SELECTIONOPERATION_H

/// @file SelectionOperation.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 选择交互基类：在 OsgWidget 的 eventFilter 中统一接收 Qt 事件，

#include <QEvent>
#include <QMouseEvent>
#include <QWheelEvent>

#include "ViewportInteraction/IViewportInteractionHost.h"

class QObject;

/// 选择交互基类：在 OsgWidget 的 eventFilter 中统一接收 Qt 事件，
/// 内部分发到 onMouseMove / onMouseButtonPress 等虚函数，子类只实现具体模式逻辑。
class SelectionOperation
{
public:
	explicit SelectionOperation(IViewportInteractionHost* host) : m_host(host) {}
	virtual ~SelectionOperation() = default;

	virtual bool handleEvent(QObject* watched, QEvent* event)
	{
		if (!event)
		{
			return false;
		}
		if (!canHandle(watched, event))
		{
			return false;
		}

		switch (event->type())
		{
		case QEvent::MouseMove:
			return onMouseMove(static_cast<QMouseEvent*>(event));
		case QEvent::MouseButtonPress:
			return onMouseButtonPress(static_cast<QMouseEvent*>(event));
		case QEvent::MouseButtonRelease:
			return onMouseButtonRelease(static_cast<QMouseEvent*>(event));
		case QEvent::Wheel:
			return onWheel(static_cast<QWheelEvent*>(event));
		case QEvent::MouseButtonDblClick:
			return onMouseDoubleClick(static_cast<QMouseEvent*>(event));
		default:
			return false;
		}
	}

protected:
	virtual bool canHandle(QObject* watched, QEvent* event) const
	{
		(void)watched;
		(void)event;
		return false;
	}

	virtual bool onMouseMove(QMouseEvent* e)
	{
		(void)e;
		return false;
	}
	virtual bool onMouseButtonPress(QMouseEvent* e)
	{
		(void)e;
		return false;
	}
	virtual bool onMouseButtonRelease(QMouseEvent* e)
	{
		(void)e;
		return false;
	}
	virtual bool onWheel(QWheelEvent* e)
	{
		(void)e;
		return false;
	}
	virtual bool onMouseDoubleClick(QMouseEvent* e)
	{
		(void)e;
		return false;
	}

	IViewportInteractionHost* host() const { return m_host; }

	IViewportInteractionHost* m_host = nullptr;
};

#endif // CLOUDSIMHOST_SELECTIONOPERATION_H
