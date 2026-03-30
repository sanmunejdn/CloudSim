#include "QWidgetViewer.h"
#include "GraphicsWindowQt1.h"
#include "QtKeyboardMap.h"

#include <osg/DeleteHandler>
#include <osgViewer/ViewerBase>
#include <QInputEvent>
#include <QPointer>

#if (QT_VERSION>=QT_VERSION_CHECK(4, 6, 0))
# define USE_GESTURES
# include <QGestureEvent>
# include <QGesture>
#endif


#if (QT_VERSION < QT_VERSION_CHECK(5, 2, 0))
#define GETDEVICEPIXELRATIO() 1.0
#else
#define GETDEVICEPIXELRATIO() devicePixelRatio()
#endif


QWidgetViewer::QWidgetViewer(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f, bool forwardKeyEvents)
	: QGLWidget(parent, shareWidget, f),
	_gw(NULL),
	_touchEventsEnabled(false),
	_forwardKeyEvents(forwardKeyEvents)
{
	_devicePixelRatio = GETDEVICEPIXELRATIO();
}

QWidgetViewer::QWidgetViewer(QGLContext* context, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f,
	bool forwardKeyEvents)
	: QGLWidget(context, parent, shareWidget, f),
	_gw(NULL),
	_touchEventsEnabled(false),
	_forwardKeyEvents(forwardKeyEvents)
{
	_devicePixelRatio = GETDEVICEPIXELRATIO();
}

QWidgetViewer::QWidgetViewer(const QGLFormat& format, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f,
	bool forwardKeyEvents)
	: QGLWidget(format, parent, shareWidget, f),
	_gw(NULL),
	_touchEventsEnabled(false),
	_forwardKeyEvents(forwardKeyEvents)
{
	_devicePixelRatio = GETDEVICEPIXELRATIO();
}

QWidgetViewer::~QWidgetViewer()
{
	// close GraphicsWindowQt1 and remove the reference to us
	if (_gw)
	{
		_gw->close();
		_gw->_widget = NULL;
		_gw = NULL;
	}
}

void QWidgetViewer::setTouchEventsEnabled(bool e)
{
#ifdef USE_GESTURES
	if (e == _touchEventsEnabled)
		return;

	_touchEventsEnabled = e;

	if (_touchEventsEnabled)
	{
		grabGesture(Qt::PinchGesture);
	}
	else
	{
		ungrabGesture(Qt::PinchGesture);
	}
#endif
}

void QWidgetViewer::processDeferredEvents()
{
	QQueue<QEvent::Type> deferredEventQueueCopy;
	{
		QMutexLocker lock(&_deferredEventQueueMutex);
		deferredEventQueueCopy = _deferredEventQueue;
		_eventCompressor.clear();
		_deferredEventQueue.clear();
	}

	while (!deferredEventQueueCopy.isEmpty())
	{
		QEvent event(deferredEventQueueCopy.dequeue());
		QGLWidget::event(&event);
	}
}

bool QWidgetViewer::event(QEvent* event)
{
#ifdef USE_GESTURES
	if (event->type() == QEvent::Gesture)
		return gestureEvent(static_cast<QGestureEvent*>(event));
#endif

	// QEvent::Hide
	//
	// workaround "Qt-workaround" that does glFinish before hiding the widget
	// (the Qt workaround was seen at least in Qt 4.6.3 and 4.7.0)
	//
	// Qt makes the context current, performs glFinish, and releases the context.
	// This makes the problem in OSG multithreaded environment as the context
	// is active in another thread, thus it can not be made current for the purpose
	// of glFinish in this thread.

	// QEvent::ParentChange
	//
	// Reparenting QWidgetViewer may create a new underlying window and a new GL context.
	// Qt will then call doneCurrent on the GL context about to be deleted. The thread
	// where old GL context was current has no longer current context to render to and
	// we cannot make new GL context current in this thread.

	// We workaround above problems by deferring execution of problematic event requests.
	// These events has to be enqueue and executed later in a main GUI thread (GUI operations
	// outside the main thread are not allowed) just before makeCurrent is called from the
	// right thread. The good place for doing that is right after swap in a swapBuffersImplementation.

	if (event->type() == QEvent::Hide)
	{
		// enqueue only the last of QEvent::Hide and QEvent::Show
		enqueueDeferredEvent(QEvent::Hide, QEvent::Show);
		return true;
	}
	else if (event->type() == QEvent::Show)
	{
		// enqueue only the last of QEvent::Show or QEvent::Hide
		enqueueDeferredEvent(QEvent::Show, QEvent::Hide);
		return true;
	}
	else if (event->type() == QEvent::ParentChange)
	{
		// enqueue only the last QEvent::ParentChange
		enqueueDeferredEvent(QEvent::ParentChange);
		return true;
	}

	// perform regular event handling
	return QGLWidget::event(event);
}

void QWidgetViewer::setKeyboardModifiers(QInputEvent* event)
{
	int modkey = event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier);
	unsigned int mask = 0;
	if (modkey & Qt::ShiftModifier) mask |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
	if (modkey & Qt::ControlModifier) mask |= osgGA::GUIEventAdapter::MODKEY_CTRL;
	if (modkey & Qt::AltModifier) mask |= osgGA::GUIEventAdapter::MODKEY_ALT;
	_gw->getEventQueue()->getCurrentEventState()->setModKeyMask(mask);
}

void QWidgetViewer::resizeEvent(QResizeEvent* event)
{
	QGLWidget::resizeEvent(event);

	// 获取缩放后的尺寸（考虑高DPI缩放）
	qreal dpr = devicePixelRatio();
	int scaledWidth = static_cast<int>(width() * dpr);
	int scaledHeight = static_cast<int>(height() * dpr);

	// 通知GraphicsWindow尺寸变化
	if (_gw) {
		_gw->resized(x(), y(), scaledWidth, scaledHeight);
		_gw->getEventQueue()->windowResize(x(), y(), scaledWidth, scaledHeight);
	}

	// 发射信号
	emit windowResized(scaledWidth, scaledHeight);
}

void QWidgetViewer::moveEvent(QMoveEvent* event)
{

	//写上自己的事件处理
#if 1
	const QPoint& pos = event->pos();
	int scaled_width = static_cast<int>(width() * _devicePixelRatio);
	int scaled_height = static_cast<int>(height() * _devicePixelRatio);
	//_gw->resized(pos.x(), pos.y(), scaled_width, scaled_height);
	//_gw->getEventQueue()->windowResize(pos.x(), pos.y(), scaled_width, scaled_height);
#endif
}

void QWidgetViewer::glDraw()
{
	if (_gw && _gw->getViewer()) {
		_gw->requestRedraw();
		_gw->getViewer()->frame();
	}
}
void QWidgetViewer::keyPressEvent(QKeyEvent* event)
{
	//写上自己的事件处理

#if 1
	setKeyboardModifiers(event);
	int value = s_QtKeyboardMap.remapKey(event);
	_gw->getEventQueue()->keyPress(value);

	// this passes the event to the regular Qt key event processing,
	// among others, it closes popup windows on ESC and forwards the event to the parent widgets
	if (_forwardKeyEvents)
		inherited::keyPressEvent(event);
#endif

}

void QWidgetViewer::keyReleaseEvent(QKeyEvent* event)
{
	//写上自己的事件处理

#if 1
	if (event->isAutoRepeat())
	{
		event->ignore();
	}
	else
	{
		setKeyboardModifiers(event);
		int value = s_QtKeyboardMap.remapKey(event);
		_gw->getEventQueue()->keyRelease(value);
	}

	// this passes the event to the regular Qt key event processing,
	// among others, it closes popup windows on ESC and forwards the event to the parent widgets
	if (_forwardKeyEvents)
		inherited::keyReleaseEvent(event);
#endif
}

void QWidgetViewer::mousePressEvent(QMouseEvent* event)
{
	if (!_gw) return;

	int button = 0;
	switch (event->button()) {
	case Qt::LeftButton: button = 1; break;
	case Qt::MidButton: button = 2; break;
	case Qt::RightButton: button = 3; break;
	default: button = 0; break;
	}

	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseButtonPress(
		event->x() * _devicePixelRatio,
		event->y() * _devicePixelRatio,
		button
	);

	// 确保事件被处理
	event->accept();
}


void QWidgetViewer::mouseReleaseEvent(QMouseEvent* event)
{
	if (!_gw) return;

	int button = 0;
	switch (event->button()) {
	case Qt::LeftButton: button = 1; break;
	case Qt::MidButton: button = 2; break;
	case Qt::RightButton: button = 3; break;
	default: button = 0; break;
	}

	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseButtonRelease(
		event->x() * _devicePixelRatio,
		event->y() * _devicePixelRatio,
		button
	);

	event->accept();
}

void QWidgetViewer::mouseDoubleClickEvent(QMouseEvent* event)
{

	//写上自己的事件处理

#if 1
	int button = 0;
	switch (event->button())
	{
	case Qt::LeftButton: button = 1; break;
	case Qt::MidButton: button = 2; break;
	case Qt::RightButton: button = 3; break;
	case Qt::NoButton: button = 0; break;
	default: button = 0; break;
	}
	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseDoubleButtonPress(event->x() * _devicePixelRatio, event->y() * _devicePixelRatio, button);
#endif
}

void QWidgetViewer::mouseMoveEvent(QMouseEvent* event)
{
	if (!_gw) return;

	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseMotion(
		event->x() * _devicePixelRatio,
		event->y() * _devicePixelRatio
	);

	event->accept();
	update(); // 强制重绘
}

void QWidgetViewer::wheelEvent(QWheelEvent* event)
{
	//写上自己的事件处理

#if 1
	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseScroll(
		event->orientation() == Qt::Vertical ?
		(event->delta() > 0 ? osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN) :
		(event->delta() > 0 ? osgGA::GUIEventAdapter::SCROLL_LEFT : osgGA::GUIEventAdapter::SCROLL_RIGHT));
#endif
}

#ifdef USE_GESTURES
static osgGA::GUIEventAdapter::TouchPhase translateQtGestureState(Qt::GestureState state)
{
	osgGA::GUIEventAdapter::TouchPhase touchPhase;
	switch (state)
	{
	case Qt::GestureStarted:
		touchPhase = osgGA::GUIEventAdapter::TOUCH_BEGAN;
		break;
	case Qt::GestureUpdated:
		touchPhase = osgGA::GUIEventAdapter::TOUCH_MOVED;
		break;
	case Qt::GestureFinished:
	case Qt::GestureCanceled:
		touchPhase = osgGA::GUIEventAdapter::TOUCH_ENDED;
		break;
	default:
		touchPhase = osgGA::GUIEventAdapter::TOUCH_UNKNOWN;
	};

	return touchPhase;
}
#endif


bool QWidgetViewer::gestureEvent(QGestureEvent* qevent)
{
#ifndef USE_GESTURES
	return false;
#else

	bool accept = false;

	if (QPinchGesture* pinch = static_cast<QPinchGesture*>(qevent->gesture(Qt::PinchGesture)))
	{
		const QPointF qcenterf = pinch->centerPoint();
		const float angle = pinch->totalRotationAngle();
		const float scale = pinch->totalScaleFactor();

		const QPoint pinchCenterQt = mapFromGlobal(qcenterf.toPoint());
		const osg::Vec2 pinchCenter(pinchCenterQt.x(), pinchCenterQt.y());

		//We don't have absolute positions of the two touches, only a scale and rotation
		//Hence we create pseudo-coordinates which are reasonable, and centered around the
		//real position
		const float radius = (width() + height()) / 4;
		const osg::Vec2 vector(scale * cos(angle) * radius, scale * sin(angle) * radius);
		const osg::Vec2 p0 = pinchCenter + vector;
		const osg::Vec2 p1 = pinchCenter - vector;

		osg::ref_ptr<osgGA::GUIEventAdapter> event = 0;
		const osgGA::GUIEventAdapter::TouchPhase touchPhase = translateQtGestureState(pinch->state());
		if (touchPhase == osgGA::GUIEventAdapter::TOUCH_BEGAN)
		{
			event = _gw->getEventQueue()->touchBegan(0, touchPhase, p0[0], p0[1]);
		}
		else if (touchPhase == osgGA::GUIEventAdapter::TOUCH_MOVED)
		{
			event = _gw->getEventQueue()->touchMoved(0, touchPhase, p0[0], p0[1]);
		}
		else
		{
			event = _gw->getEventQueue()->touchEnded(0, touchPhase, p0[0], p0[1], 1);
		}

		if (event)
		{
			event->addTouchPoint(1, touchPhase, p1[0], p1[1]);
			accept = true;
		}
	}

	if (accept)
		qevent->accept();

	return accept;
#endif
}
