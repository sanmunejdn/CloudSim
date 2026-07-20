#ifndef WIDGET_QWIDGETVIEWER_H
#define WIDGET_QWIDGETVIEWER_H

/// @file QWidgetViewer.h
/// @brief OpenGL/OSG 嵌入 Qt（QOpenGLWidget）；输入事件转发 osgViewer，供 GraphicsWindowQt1 使用

#include "widget_global.h"

#include <QEvent>
#include <QGestureEvent>
#include <QInputEvent>
#include <QMutex>
#include <QMutexLocker>
#include <QOpenGLWidget>
#include <QQueue>
#include <QSet>
#include <QSurfaceFormat>

#include <osg/Referenced>
#include <osgViewer/Viewer>

class GraphicsWindowQt1;

/// OpenGL/OSG 嵌入 Qt（QOpenGLWidget）；输入事件转发 osgViewer，供 GraphicsWindowQt1 使用
class OSG_WIDGET_API QWidgetViewer : public QOpenGLWidget
{
	Q_OBJECT
	typedef QOpenGLWidget inherited;

public:
	explicit QWidgetViewer(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags(),
						   bool forwardKeyEvents = false);
	QWidgetViewer(const QSurfaceFormat& format, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags(),
				  bool forwardKeyEvents = false);
	virtual ~QWidgetViewer();

	inline void setGraphicsWindow(GraphicsWindowQt1* gw) { _gw = gw; }
	inline GraphicsWindowQt1* getGraphicsWindow() { return _gw; }
	inline const GraphicsWindowQt1* getGraphicsWindow() const { return _gw; }

	inline bool getForwardKeyEvents() const { return _forwardKeyEvents; }
	virtual void setForwardKeyEvents(bool f) { _forwardKeyEvents = f; }

	inline bool getTouchEventsEnabled() const { return _touchEventsEnabled; }
	void setTouchEventsEnabled(bool e);

	void setKeyboardModifiers(QInputEvent* event);

	static qreal effectiveDevicePixelRatio(const QWidget* widget);

	/// 返回 OSG viewport 应使用的 framebuffer 像素尺寸
	bool queryFramebufferPixelSize(int& outWidth, int& outHeight) const;
	bool resolveOpenGlFramebufferSize(int& outWidth, int& outHeight) const;

	virtual void keyPressEvent(QKeyEvent* event);
	virtual void keyReleaseEvent(QKeyEvent* event);
	virtual void mousePressEvent(QMouseEvent* event);
	virtual void mouseReleaseEvent(QMouseEvent* event);
	virtual void mouseDoubleClickEvent(QMouseEvent* event);
	virtual void mouseMoveEvent(QMouseEvent* event);
	virtual void wheelEvent(QWheelEvent* event);
	bool gestureEvent(QGestureEvent* event);

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
			{
				_eventCompressor.remove(eventType);
			}
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
	qreal _devicePixelRatio = 1.0;
	int _lastSyncedFramebufferWidth = 0;
	int _lastSyncedFramebufferHeight = 0;
	int _lastSyncedLogicalWidth = 0;
	int _lastSyncedLogicalHeight = 0;

	void syncFramebufferSize(int deviceFramebufferWidth = 0, int deviceFramebufferHeight = 0);
	void emitFramebufferResizeIfChanged(int framebufferWidth, int framebufferHeight);

	void initializeGL() override;
	void resizeGL(int w, int h) override;
	void paintGL() override;
	void showEvent(QShowEvent* event) override;
	bool event(QEvent* event) override;
};

#endif // WIDGET_QWIDGETVIEWER_H
