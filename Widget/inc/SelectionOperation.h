#pragma once

#include <QEvent>
#include <QMouseEvent>
#include <QWheelEvent>

class QObject;
class OsgWidget;

/// 选择交互基类：在 OsgWidget 的 eventFilter 中统一接收 Qt 事件，
/// 内部分发到 onMouseMove / onMouseButtonPress 等虚函数，子类只实现具体模式逻辑。
class SelectionOperation
{
public:
	explicit SelectionOperation(OsgWidget* owner) : m_owner(owner) {}
	virtual ~SelectionOperation() = default;

	// Base-class event dispatcher:
	// - Derivations only implement hook methods (below).
	// - This function remains stable so OsgWidget can call it uniformly.
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
	// Whether this operation should consume/process the event right now.
	// Usually depends on mode flags and event target widget.
	virtual bool canHandle(QObject* watched, QEvent* event) const
	{
		(void)watched;
		(void)event;
		return false;
	}

	// Hooks: default do nothing (do not consume).
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

	OsgWidget* m_owner = nullptr;
};

