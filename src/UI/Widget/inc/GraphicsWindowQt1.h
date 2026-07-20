#ifndef WIDGET_GRAPHICSWINDOWQT1_H
#define WIDGET_GRAPHICSWINDOWQT1_H

/// @file GraphicsWindowQt1.h
/// @brief OSG 图形窗口适配：实现 osgViewer::GraphicsWindow，与 QOpenGLWidget 同步尺寸与事件

#include "widget_global.h"

#include "QWidgetViewer.h"

#include <QCursor>
#include <QGestureEvent>
#include <QOpenGLWidget>
#include <QSurfaceFormat>
#include <QtCore>
#include <QtWidgets>

#include <osg/Referenced>
#include <osgViewer/Viewer>

/// OSG 图形窗口适配：实现 osgViewer::GraphicsWindow，与 QOpenGLWidget 同步尺寸与事件
class WIDGET_EXPORT GraphicsWindowQt1 : public osgViewer::GraphicsWindow
{
public:
	GraphicsWindowQt1(osg::GraphicsContext::Traits* traits, QWidget* parent = NULL, Qt::WindowFlags f = 0);
	GraphicsWindowQt1(QWidgetViewer* widget);
	virtual ~GraphicsWindowQt1();

	inline QWidgetViewer* getGLWidget() { return _widget; }
	inline const QWidgetViewer* getGLWidget() const { return _widget; }

	/// 已弃用
	inline QWidgetViewer* getGraphWidget() { return _widget; }
	/// 已弃用
	inline const QWidgetViewer* getGraphWidget() const { return _widget; }

	struct WindowData : public osg::Referenced
	{
		WindowData(QWidgetViewer* widget = NULL, QWidget* parent = NULL) : _widget(widget), _parent(parent) {}
		QWidgetViewer* _widget;
		QWidget* _parent;
	};

	/// 同步 traits 宽高并触发 resized
	void updateSize(int width, int height)
	{
		_traits->width = width;
		_traits->height = height;
		if (isRealized())
		{
			resized(_traits->x, _traits->y, width, height);
		}
	}

	bool init(QWidget* parent, Qt::WindowFlags f);

	static QSurfaceFormat traits2qSurfaceFormat(const osg::GraphicsContext::Traits* traits);
	static void qSurfaceFormat2traits(const QSurfaceFormat& format, osg::GraphicsContext::Traits* traits);
	static osg::GraphicsContext::Traits* createTraits(const QWidgetViewer* widget);

	virtual bool setWindowRectangleImplementation(int x, int y, int width, int height);
	virtual void getWindowRectangle(int& x, int& y, int& width, int& height);
	virtual bool setWindowDecorationImplementation(bool windowDecoration);
	virtual bool getWindowDecoration() const;
	virtual void grabFocus();
	virtual void grabFocusIfPointerInWindow();
	virtual void raiseWindow();
	virtual void setWindowName(const std::string& name);
	virtual std::string getWindowName();
	virtual void useCursor(bool cursorOn);
	virtual void setCursor(MouseCursor cursor);
	inline bool getTouchEventsEnabled() const { return _widget->getTouchEventsEnabled(); }
	virtual void setTouchEventsEnabled(bool e) { _widget->setTouchEventsEnabled(e); }

	virtual bool valid() const;
	virtual bool realizeImplementation();
	virtual bool isRealizedImplementation() const;
	virtual void closeImplementation();
	virtual bool makeCurrentImplementation();
	virtual bool releaseContextImplementation();
	virtual void swapBuffersImplementation();
	virtual void runOperations();

	virtual void requestWarpPointer(float x, float y);

	osgViewer::Viewer* getViewer() const { return _viewer.get(); }
	void setViewer(osgViewer::Viewer* viewer) { _viewer = viewer; }

protected:
	friend class QWidgetViewer;
	QWidgetViewer* _widget;
	bool _ownsWidget;
	QCursor _currentCursor;
	bool _realized;

	osg::observer_ptr<osgViewer::Viewer> _viewer;
};

#endif // WIDGET_GRAPHICSWINDOWQT1_H
