#ifndef CLOUDSIMHOST_QWIDGETVIEWER_H
#define CLOUDSIMHOST_QWIDGETVIEWER_H

#include "widget_global.h"

/// @file QWidgetViewer.h
/// @brief OpenGL/OSG 嵌入 Qt；输入事件转发 osgViewer，供 GraphicsWindowQt1 使用

#include <QCursor>
#include <QGLWidget>
#include <QGestureEvent>
#include <QtCore>
#include <QtWidgets>

#include <osg/Referenced>
#include <osgViewer/Viewer>

class GraphicsWindowQt1;

/// OpenGL/OSG 嵌入 Qt；输入事件转发 osgViewer，供 GraphicsWindowQt1 使用
class WIDGET_EXPORT QWidgetViewer : public QGLWidget
{
	Q_OBJECT
	typedef QGLWidget inherited;

public:
	QWidgetViewer(QWidget* parent = NULL, const QGLWidget* shareWidget = NULL, Qt::WindowFlags f = 0,
				  bool forwardKeyEvents = false);
	QWidgetViewer(QGLContext* context, QWidget* parent = NULL, const QGLWidget* shareWidget = NULL,
				  Qt::WindowFlags f = 0, bool forwardKeyEvents = false);
	QWidgetViewer(const QGLFormat& format, QWidget* parent = NULL, const QGLWidget* shareWidget = NULL,
				  Qt::WindowFlags f = 0, bool forwardKeyEvents = false);
	virtual ~QWidgetViewer();

	inline void setGraphicsWindow(GraphicsWindowQt1* gw) { _gw = gw; }
	inline GraphicsWindowQt1* getGraphicsWindow() { return _gw; }
	inline const GraphicsWindowQt1* getGraphicsWindow() const { return _gw; }

	inline bool getForwardKeyEvents() const { return _forwardKeyEvents; }
	virtual void setForwardKeyEvents(bool f) { _forwardKeyEvents = f; }

	inline bool getTouchEventsEnabled() const { return _touchEventsEnabled; }
	void setTouchEventsEnabled(bool e);

	void setKeyboardModifiers(QInputEvent* event);

	virtual void keyPressEvent(QKeyEvent* event);
	virtual void keyReleaseEvent(QKeyEvent* event);
	virtual void mousePressEvent(QMouseEvent* event);
	virtual void mouseReleaseEvent(QMouseEvent* event);
	virtual void mouseDoubleClickEvent(QMouseEvent* event);
	virtual void mouseMoveEvent(QMouseEvent* event);
	virtual void wheelEvent(QWheelEvent* event);
	virtual bool gestureEvent(QGestureEvent* event);

signals:
	void windowResized(int width, int height);

protected:
	int getNumDeferredEvents()
	{
		QMutexLocker lock(&_deferredEventQueueMutex);
		return _deferredEventQueue.count();
	}
	void enqueueDeferredEvent(QEvent::Type eventType, QEvent::Type removeEventType = QEvent::None)
	{
		QMutexLocker lock(&_deferredEventQueueMutex);

		if (removeEventType != QEvent::None)
		{
			if (_deferredEventQueue.removeOne(removeEventType))
				_eventCompressor.remove(eventType);
		}

		if (_eventCompressor.find(eventType) == _eventCompressor.end())
		{
			_deferredEventQueue.enqueue(eventType);
			_eventCompressor.insert(eventType);
		}
	}
	void processDeferredEvents();

	friend class GraphicsWindowQt1;
	GraphicsWindowQt1* _gw;

	QMutex _deferredEventQueueMutex;
	QQueue<QEvent::Type> _deferredEventQueue;
	QSet<QEvent::Type> _eventCompressor;

	bool _touchEventsEnabled;

	bool _forwardKeyEvents;
	qreal _devicePixelRatio;

	virtual void resizeEvent(QResizeEvent* event) override;
	virtual void moveEvent(QMoveEvent* event);
	virtual void glDraw();
	virtual bool event(QEvent* event);
};

#endif // CLOUDSIMHOST_QWIDGETVIEWER_H
