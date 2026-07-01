#include "QWidgetViewer.h"
#include "GraphicsWindowQt1.h"
#include "QtKeyboardMap.h"

#include <osg/DeleteHandler>
#include <osgViewer/ViewerBase>

#include <cmath>
#include <QInputEvent>
#include <QGuiApplication>
#include <QOpenGLContext>
#include <QPointer>
#include <QScreen>
#include <QShowEvent>
#include <QWindow>

#if (QT_VERSION >= QT_VERSION_CHECK(4, 6, 0))
# define USE_GESTURES
# include <QGestureEvent>
# include <QGesture>
#endif

namespace {

double deviceCoord(const double logicalCoord, const qreal devicePixelRatio)
{
	return logicalCoord * devicePixelRatio;
}

qreal freshDevicePixelRatio(const QWidget* widget)
{
	if (!widget)
	{
		return 1.0;
	}

	const qreal widgetDpr = widget->devicePixelRatioF();
	if (widgetDpr > 1.0)
	{
		return widgetDpr;
	}

	qreal dpr = widget->devicePixelRatio();
	if (dpr > 1.0)
	{
		return dpr;
	}

	if (const QWindow* windowHandle = widget->window() ? widget->window()->windowHandle() : nullptr)
	{
		dpr = windowHandle->devicePixelRatio();
		if (dpr > 1.0)
		{
			return dpr;
		}
	}

	if (const QScreen* screen = widget->screen())
	{
		dpr = screen->devicePixelRatio();
		if (dpr > 1.0)
		{
			return dpr;
		}
	}

	if (const QScreen* primary = QGuiApplication::primaryScreen())
	{
		dpr = primary->devicePixelRatio();
		if (dpr > 1.0)
		{
			return dpr;
		}
	}

	return 1.0;
}

} // namespace

QWidgetViewer::QWidgetViewer(QWidget* parent, Qt::WindowFlags f, bool forwardKeyEvents)
	: QOpenGLWidget(parent, f)
	, _gw(nullptr)
	, _touchEventsEnabled(false)
	, _forwardKeyEvents(forwardKeyEvents)
{
	_devicePixelRatio = devicePixelRatio();
	setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);
}

QWidgetViewer::QWidgetViewer(const QSurfaceFormat& format, QWidget* parent, Qt::WindowFlags f, bool forwardKeyEvents)
	: QOpenGLWidget(parent, f)
	, _gw(nullptr)
	, _touchEventsEnabled(false)
	, _forwardKeyEvents(forwardKeyEvents)
{
	setFormat(format);
	_devicePixelRatio = devicePixelRatio();
	setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);
}

QWidgetViewer::~QWidgetViewer()
{
	if (_gw)
	{
		_gw->close();
		_gw->_widget = nullptr;
		_gw = nullptr;
	}
}

qreal QWidgetViewer::effectiveDevicePixelRatio(const QWidget* widget)
{
	if (!widget)
	{
		return 1.0;
	}

	if (const auto* viewer = qobject_cast<const QWidgetViewer*>(widget))
	{
		if (viewer->width() > 0
			&& viewer->_lastSyncedFramebufferWidth > 0
			&& viewer->width() == viewer->_lastSyncedLogicalWidth
			&& viewer->height() == viewer->_lastSyncedLogicalHeight)
		{
			const qreal cachedDpr = static_cast<qreal>(viewer->_lastSyncedFramebufferWidth)
				/ static_cast<qreal>(viewer->width());
			const qreal freshDpr = freshDevicePixelRatio(widget);
			if (std::abs(cachedDpr - freshDpr) < 0.01)
			{
				return cachedDpr;
			}
		}
	}

	return freshDevicePixelRatio(widget);
}

bool QWidgetViewer::resolveOpenGlFramebufferSize(int& outWidth, int& outHeight) const
{
	outWidth = 0;
	outHeight = 0;

	const int logicalW = width();
	const int logicalH = height();
	if (logicalW <= 0 || logicalH <= 0)
	{
		return false;
	}

	const qreal dpr = freshDevicePixelRatio(this);
	outWidth = static_cast<int>(std::lround(static_cast<double>(logicalW) * dpr));
	outHeight = static_cast<int>(std::lround(static_cast<double>(logicalH) * dpr));
	return outWidth > 0 && outHeight > 0;
}

bool QWidgetViewer::queryFramebufferPixelSize(int& outWidth, int& outHeight) const
{
	outWidth = 0;
	outHeight = 0;

	if (_lastSyncedFramebufferWidth > 0
		&& _lastSyncedFramebufferHeight > 0
		&& width() == _lastSyncedLogicalWidth
		&& height() == _lastSyncedLogicalHeight)
	{
		const int expectedW = static_cast<int>(std::lround(
			static_cast<double>(width()) * freshDevicePixelRatio(this)));
		const int expectedH = static_cast<int>(std::lround(
			static_cast<double>(height()) * freshDevicePixelRatio(this)));
		if (expectedW == _lastSyncedFramebufferWidth && expectedH == _lastSyncedFramebufferHeight)
		{
			outWidth = _lastSyncedFramebufferWidth;
			outHeight = _lastSyncedFramebufferHeight;
			return true;
		}
	}

	return resolveOpenGlFramebufferSize(outWidth, outHeight);
}

void QWidgetViewer::syncFramebufferSize(int deviceFramebufferWidth, int deviceFramebufferHeight)
{
	int framebufferW = deviceFramebufferWidth;
	int framebufferH = deviceFramebufferHeight;
	if (framebufferW <= 0 || framebufferH <= 0)
	{
		if (!resolveOpenGlFramebufferSize(framebufferW, framebufferH))
		{
			return;
		}
	}

	emitFramebufferResizeIfChanged(framebufferW, framebufferH);
}

void QWidgetViewer::emitFramebufferResizeIfChanged(int framebufferWidth, int framebufferHeight)
{
	if (framebufferWidth <= 0 || framebufferHeight <= 0)
	{
		return;
	}

	int resolvedW = 0;
	int resolvedH = 0;
	if (resolveOpenGlFramebufferSize(resolvedW, resolvedH))
	{
		framebufferWidth = resolvedW;
		framebufferHeight = resolvedH;
	}

	if (framebufferWidth == _lastSyncedFramebufferWidth
		&& framebufferHeight == _lastSyncedFramebufferHeight
		&& width() == _lastSyncedLogicalWidth
		&& height() == _lastSyncedLogicalHeight)
	{
		return;
	}

	_lastSyncedFramebufferWidth = framebufferWidth;
	_lastSyncedFramebufferHeight = framebufferHeight;
	_lastSyncedLogicalWidth = width();
	_lastSyncedLogicalHeight = height();
	_devicePixelRatio = static_cast<qreal>(framebufferWidth)
		/ static_cast<qreal>((std::max)(1, width()));

	if (_gw)
	{
		_gw->resized(x(), y(), framebufferWidth, framebufferHeight);
		_gw->getEventQueue()->windowResize(x(), y(), framebufferWidth, framebufferHeight);
	}

	emit windowResized(framebufferWidth, framebufferHeight);
}

void QWidgetViewer::setTouchEventsEnabled(bool e)
{
#ifdef USE_GESTURES
	if (e == _touchEventsEnabled)
	{
		return;
	}

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
		QOpenGLWidget::event(&event);
	}
}

bool QWidgetViewer::event(QEvent* event)
{
#ifdef USE_GESTURES
	if (event->type() == QEvent::Gesture)
	{
		return gestureEvent(static_cast<QGestureEvent*>(event));
	}
#endif

	if (event->type() == QEvent::Hide)
	{
		enqueueDeferredEvent(QEvent::Hide, QEvent::Show);
		return true;
	}
	if (event->type() == QEvent::Show)
	{
		enqueueDeferredEvent(QEvent::Show, QEvent::Hide);
		return true;
	}
	if (event->type() == QEvent::ParentChange)
	{
		enqueueDeferredEvent(QEvent::ParentChange);
		return true;
	}
	if (event->type() == QEvent::ScreenChangeInternal)
	{
		syncFramebufferSize();
	}

	return QOpenGLWidget::event(event);
}

void QWidgetViewer::showEvent(QShowEvent* event)
{
	QOpenGLWidget::showEvent(event);
	// 窗口显示后 devicePixelRatioF 才可靠，避免启动阶段 FBO 按 DPR=1 缓存
	syncFramebufferSize();
}

void QWidgetViewer::setKeyboardModifiers(QInputEvent* event)
{
	if (!_gw)
	{
		return;
	}

	int modkey = event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier);
	unsigned int mask = 0;
	if (modkey & Qt::ShiftModifier)
	{
		mask |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
	}
	if (modkey & Qt::ControlModifier)
	{
		mask |= osgGA::GUIEventAdapter::MODKEY_CTRL;
	}
	if (modkey & Qt::AltModifier)
	{
		mask |= osgGA::GUIEventAdapter::MODKEY_ALT;
	}
	_gw->getEventQueue()->getCurrentEventState()->setModKeyMask(mask);
}

void QWidgetViewer::initializeGL()
{
	syncFramebufferSize();
	if (_gw)
	{
		_gw->getEventQueue()->syncWindowRectangleWithGraphicsContext();
	}
}

void QWidgetViewer::resizeGL(int w, int h)
{
	Q_UNUSED(w);
	Q_UNUSED(h);
	syncFramebufferSize();
}

void QWidgetViewer::paintGL()
{
	if (!_gw || !_gw->getViewer())
	{
		return;
	}

	syncFramebufferSize();
	_gw->requestRedraw();
	_gw->getViewer()->frame();
}

void QWidgetViewer::keyPressEvent(QKeyEvent* event)
{
	if (!_gw)
	{
		return;
	}

	setKeyboardModifiers(event);
	const int value = s_QtKeyboardMap.remapKey(event);
	_gw->getEventQueue()->keyPress(value);

	if (_forwardKeyEvents)
	{
		inherited::keyPressEvent(event);
	}
}

void QWidgetViewer::keyReleaseEvent(QKeyEvent* event)
{
	if (!_gw)
	{
		return;
	}

	if (event->isAutoRepeat())
	{
		event->ignore();
	}
	else
	{
		setKeyboardModifiers(event);
		const int value = s_QtKeyboardMap.remapKey(event);
		_gw->getEventQueue()->keyRelease(value);
	}

	if (_forwardKeyEvents)
	{
		inherited::keyReleaseEvent(event);
	}
}

void QWidgetViewer::mousePressEvent(QMouseEvent* event)
{
	if (!_gw)
	{
		return;
	}

	int button = 0;
	switch (event->button())
	{
	case Qt::LeftButton: button = 1; break;
	case Qt::MidButton: button = 2; break;
	case Qt::RightButton: button = 3; break;
	default: button = 0; break;
	}

	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseButtonPress(
		deviceCoord(event->x(), _devicePixelRatio),
		deviceCoord(event->y(), _devicePixelRatio),
		button);

	event->accept();
	update();
}

void QWidgetViewer::mouseReleaseEvent(QMouseEvent* event)
{
	if (!_gw)
	{
		return;
	}

	int button = 0;
	switch (event->button())
	{
	case Qt::LeftButton: button = 1; break;
	case Qt::MidButton: button = 2; break;
	case Qt::RightButton: button = 3; break;
	default: button = 0; break;
	}

	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseButtonRelease(
		deviceCoord(event->x(), _devicePixelRatio),
		deviceCoord(event->y(), _devicePixelRatio),
		button);

	event->accept();
	update();
}

void QWidgetViewer::mouseDoubleClickEvent(QMouseEvent* event)
{
	if (!_gw)
	{
		return;
	}

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
	_gw->getEventQueue()->mouseDoubleButtonPress(
		deviceCoord(event->x(), _devicePixelRatio),
		deviceCoord(event->y(), _devicePixelRatio),
		button);
}

void QWidgetViewer::mouseMoveEvent(QMouseEvent* event)
{
	if (!_gw)
	{
		return;
	}

	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseMotion(
		deviceCoord(event->x(), _devicePixelRatio),
		deviceCoord(event->y(), _devicePixelRatio));

	event->accept();
	update();
}

void QWidgetViewer::wheelEvent(QWheelEvent* event)
{
	if (!_gw)
	{
		return;
	}

	setKeyboardModifiers(event);
	_gw->getEventQueue()->mouseScroll(
		event->orientation() == Qt::Vertical
			? (event->delta() > 0 ? osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN)
			: (event->delta() > 0 ? osgGA::GUIEventAdapter::SCROLL_LEFT : osgGA::GUIEventAdapter::SCROLL_RIGHT));
	update();
}

#ifdef USE_GESTURES
static osgGA::GUIEventAdapter::TouchPhase translateQtGestureState(Qt::GestureState state)
{
	osgGA::GUIEventAdapter::TouchPhase touchPhase;
	switch (state)
	{
	case Qt::GestureStarted: touchPhase = osgGA::GUIEventAdapter::TOUCH_BEGAN; break;
	case Qt::GestureUpdated: touchPhase = osgGA::GUIEventAdapter::TOUCH_MOVED; break;
	case Qt::GestureFinished:
	case Qt::GestureCanceled: touchPhase = osgGA::GUIEventAdapter::TOUCH_ENDED; break;
	default: touchPhase = osgGA::GUIEventAdapter::TOUCH_UNKNOWN; break;
	}
	return touchPhase;
}
#endif

bool QWidgetViewer::gestureEvent(QGestureEvent* qevent)
{
#ifndef USE_GESTURES
	Q_UNUSED(qevent);
	return false;
#else
	if (!_gw)
	{
		return false;
	}

	bool accept = false;

	if (QPinchGesture* pinch = static_cast<QPinchGesture*>(qevent->gesture(Qt::PinchGesture)))
	{
		const QPointF qcenterf = pinch->centerPoint();
		const float angle = pinch->totalRotationAngle();
		const float scale = pinch->totalScaleFactor();

		const QPoint pinchCenterQt = mapFromGlobal(qcenterf.toPoint());
		const osg::Vec2 pinchCenter(
			static_cast<float>(deviceCoord(pinchCenterQt.x(), _devicePixelRatio)),
			static_cast<float>(deviceCoord(pinchCenterQt.y(), _devicePixelRatio)));

		const float radius = static_cast<float>((width() + height()) * _devicePixelRatio) / 4.0f;
		const osg::Vec2 vector(scale * cos(angle) * radius, scale * sin(angle) * radius);
		const osg::Vec2 p0 = pinchCenter + vector;
		const osg::Vec2 p1 = pinchCenter - vector;

		osg::ref_ptr<osgGA::GUIEventAdapter> event;
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
	{
		qevent->accept();
	}

	return accept;
#endif
}
