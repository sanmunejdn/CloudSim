#include "GraphicsWindowQt1.h"
#include <osg/DeleteHandler>
#include <osgViewer/ViewerBase>
#include <QInputEvent>
#include <QPointer>



GraphicsWindowQt1::GraphicsWindowQt1(osg::GraphicsContext::Traits* traits, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f)
	: _realized(false)
{

	_widget = NULL;
	_traits = traits;
	init(parent, shareWidget, f);
}

GraphicsWindowQt1::GraphicsWindowQt1(QWidgetViewer* widget)
	: _realized(false)
{
	_widget = widget;
	_traits = _widget ? createTraits(_widget) : new osg::GraphicsContext::Traits;
	init(NULL, NULL, 0);
}

GraphicsWindowQt1::~GraphicsWindowQt1()
{
	// 先断开与QWidgetViewer的连接
	if (_widget) {
		_widget->_gw = nullptr;
		_widget = nullptr;  // 防止野指针
	}

	// 安全关闭（检查是否已关闭）
	if (isRealized()) {
		releaseContext();
		closeImplementation();
	}
}

bool GraphicsWindowQt1::init(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f)
{
	// update _widget and parent by WindowData
	WindowData* windowData = _traits.get() ? dynamic_cast<WindowData*>(_traits->inheritedWindowData.get()) : 0;
	if (!_widget)
		_widget = windowData ? windowData->_widget : NULL;
	if (!parent)
		parent = windowData ? windowData->_parent : NULL;

	// create widget if it does not exist
	_ownsWidget = _widget == NULL;
	if (!_widget)
	{
		// shareWidget
		if (!shareWidget) {
			GraphicsWindowQt1* sharedContextQt = dynamic_cast<GraphicsWindowQt1*>(_traits->sharedContext.get());
			if (sharedContextQt)
				shareWidget = sharedContextQt->getGLWidget();
		}

		// WindowFlags
		Qt::WindowFlags flags = f | Qt::Window | Qt::CustomizeWindowHint;

		_traits->windowDecoration = false;
		if (_traits->windowDecoration)
			flags |= Qt::WindowTitleHint | Qt::WindowMinMaxButtonsHint | Qt::WindowSystemMenuHint
#if (QT_VERSION_CHECK(4, 5, 0) <= QT_VERSION)
			| Qt::WindowCloseButtonHint
#endif
			;

		// create widget
		_widget = new QWidgetViewer(traits2qglFormat(_traits.get()), parent, shareWidget, flags);
	}

	// set widget name and position
	// (do not set it when we inherited the widget)
	if (_ownsWidget)
	{
		_widget->setWindowTitle(_traits->windowName.c_str());
		_widget->move(_traits->x, _traits->y);
		if (!_traits->supportsResize) _widget->setFixedSize(_traits->width, _traits->height);
		else _widget->resize(_traits->width, _traits->height);
	}

	// initialize widget properties
	_widget->setAutoBufferSwap(false);
	_widget->setMouseTracking(true);
	_widget->setFocusPolicy(Qt::WheelFocus);
	_widget->setGraphicsWindow(this);
	useCursor(_traits->useCursor);

	// initialize State
	setState(new osg::State);
	getState()->setGraphicsContext(this);

	// initialize contextID
	if (_traits.valid() && _traits->sharedContext.valid())
	{
		getState()->setContextID(_traits->sharedContext->getState()->getContextID());
		incrementContextIDUsageCount(getState()->getContextID());
	}
	else
	{
		getState()->setContextID(osg::GraphicsContext::createNewContextID());
	}

	// make sure the event queue has the correct window rectangle size and input range
	getEventQueue()->syncWindowRectangleWithGraphicsContext();

	return true;
}

QGLFormat GraphicsWindowQt1::traits2qglFormat(const osg::GraphicsContext::Traits* traits)
{
	QGLFormat format(QGLFormat::defaultFormat());

	format.setAlphaBufferSize(traits->alpha);
	format.setRedBufferSize(traits->red);
	format.setGreenBufferSize(traits->green);
	format.setBlueBufferSize(traits->blue);
	format.setDepthBufferSize(traits->depth);
	format.setStencilBufferSize(traits->stencil);
	format.setSampleBuffers(traits->sampleBuffers);
	format.setSamples(traits->samples);

	format.setAlpha(traits->alpha > 0);
	format.setDepth(traits->depth > 0);
	format.setStencil(traits->stencil > 0);
	format.setDoubleBuffer(traits->doubleBuffer);
	format.setSwapInterval(traits->vsync ? 1 : 0);
	format.setStereo(traits->quadBufferStereo ? 1 : 0);

	return format;
}

void GraphicsWindowQt1::qglFormat2traits(const QGLFormat& format, osg::GraphicsContext::Traits* traits)
{
	traits->red = format.redBufferSize();
	traits->green = format.greenBufferSize();
	traits->blue = format.blueBufferSize();
	traits->alpha = format.alpha() ? format.alphaBufferSize() : 0;
	traits->depth = format.depth() ? format.depthBufferSize() : 0;
	traits->stencil = format.stencil() ? format.stencilBufferSize() : 0;

	traits->sampleBuffers = format.sampleBuffers() ? 1 : 0;
	traits->samples = format.samples();

	traits->quadBufferStereo = format.stereo();
	traits->doubleBuffer = format.doubleBuffer();

	traits->vsync = format.swapInterval() >= 1;
}

osg::GraphicsContext::Traits* GraphicsWindowQt1::createTraits(const QGLWidget* widget)
{
	osg::GraphicsContext::Traits* traits = new osg::GraphicsContext::Traits;

	qglFormat2traits(widget->format(), traits);

	QRect r = widget->geometry();
	traits->x = r.x();
	traits->y = r.y();
	traits->width = r.width();
	traits->height = r.height();

	traits->windowName = widget->windowTitle().toLocal8Bit().data();
	Qt::WindowFlags f = widget->windowFlags();
	traits->windowDecoration = (f & Qt::WindowTitleHint) &&
		(f & Qt::WindowMinMaxButtonsHint) &&
		(f & Qt::WindowSystemMenuHint);
	QSizePolicy sp = widget->sizePolicy();
	traits->supportsResize = sp.horizontalPolicy() != QSizePolicy::Fixed ||
		sp.verticalPolicy() != QSizePolicy::Fixed;

	return traits;
}

bool GraphicsWindowQt1::setWindowRectangleImplementation(int x, int y, int width, int height)
{
	if (_widget == NULL)
		return false;

	_widget->setGeometry(x, y, width, height);
	return true;
}

void GraphicsWindowQt1::getWindowRectangle(int& x, int& y, int& width, int& height)
{
	if (_widget)
	{
		const QRect& geom = _widget->geometry();
		x = geom.x();
		y = geom.y();
		width = geom.width();
		height = geom.height();
	}
}

bool GraphicsWindowQt1::setWindowDecorationImplementation(bool windowDecoration)
{
	Qt::WindowFlags flags = Qt::Window | Qt::CustomizeWindowHint;                 //|Qt::WindowStaysOnTopHint;
	if (windowDecoration)
		flags |= Qt::WindowTitleHint | Qt::WindowMinMaxButtonsHint | Qt::WindowSystemMenuHint;
	_traits->windowDecoration = windowDecoration;

	if (_widget)
	{
		_widget->setWindowFlags(flags);

		return true;
	}

	return false;
}

bool GraphicsWindowQt1::getWindowDecoration() const
{
	return _traits->windowDecoration;
}

void GraphicsWindowQt1::grabFocus()
{
	if (_widget)
		_widget->setFocus(Qt::ActiveWindowFocusReason);
}

void GraphicsWindowQt1::grabFocusIfPointerInWindow()
{
	if (_widget->underMouse())
		_widget->setFocus(Qt::ActiveWindowFocusReason);
}

void GraphicsWindowQt1::raiseWindow()
{
	if (_widget)
		_widget->raise();
}

void GraphicsWindowQt1::setWindowName(const std::string& name)
{
	if (_widget)
		_widget->setWindowTitle(name.c_str());
}

std::string GraphicsWindowQt1::getWindowName()
{
	return _widget ? _widget->windowTitle().toStdString() : "";
}

void GraphicsWindowQt1::useCursor(bool cursorOn)
{
	if (_widget)
	{
		_traits->useCursor = cursorOn;
		if (!cursorOn) _widget->setCursor(Qt::BlankCursor);
		else _widget->setCursor(_currentCursor);
	}
}

void GraphicsWindowQt1::setCursor(MouseCursor cursor)
{
	if (cursor == InheritCursor && _widget)
	{
		_widget->unsetCursor();
	}

	switch (cursor)
	{
	case NoCursor: _currentCursor = Qt::BlankCursor; break;
	case RightArrowCursor: case LeftArrowCursor: _currentCursor = Qt::ArrowCursor; break;
	case InfoCursor: _currentCursor = Qt::SizeAllCursor; break;
	case DestroyCursor: _currentCursor = Qt::ForbiddenCursor; break;
	case HelpCursor: _currentCursor = Qt::WhatsThisCursor; break;
	case CycleCursor: _currentCursor = Qt::ForbiddenCursor; break;
	case SprayCursor: _currentCursor = Qt::SizeAllCursor; break;
	case WaitCursor: _currentCursor = Qt::WaitCursor; break;
	case TextCursor: _currentCursor = Qt::IBeamCursor; break;
	case CrosshairCursor: _currentCursor = Qt::CrossCursor; break;
	case HandCursor: _currentCursor = Qt::OpenHandCursor; break;
	case UpDownCursor: _currentCursor = Qt::SizeVerCursor; break;
	case LeftRightCursor: _currentCursor = Qt::SizeHorCursor; break;
	case TopSideCursor: case BottomSideCursor: _currentCursor = Qt::UpArrowCursor; break;
	case LeftSideCursor: case RightSideCursor: _currentCursor = Qt::SizeHorCursor; break;
	case TopLeftCorner: _currentCursor = Qt::SizeBDiagCursor; break;
	case TopRightCorner: _currentCursor = Qt::SizeFDiagCursor; break;
	case BottomRightCorner: _currentCursor = Qt::SizeBDiagCursor; break;
	case BottomLeftCorner: _currentCursor = Qt::SizeFDiagCursor; break;
	default: break;
	};
	if (_widget) _widget->setCursor(_currentCursor);
}

bool GraphicsWindowQt1::valid() const
{
	return _widget && _widget->isValid();
}

bool GraphicsWindowQt1::realizeImplementation()
{
	// save the current context
	// note: this will save only Qt-based contexts
	const QGLContext* savedContext = QGLContext::currentContext();

	// initialize GL context for the widget
	if (!valid())
		_widget->glInit();

	// make current
	_realized = true;
	bool result = makeCurrent();
	_realized = false;

	// fail if we do not have current context
	if (!result)
	{
		if (savedContext)
			const_cast<QGLContext*>(savedContext)->makeCurrent();

		OSG_WARN << "Window realize: Can make context current." << std::endl;
		return false;
	}

	_realized = true;

	// make sure the event queue has the correct window rectangle size and input range
	getEventQueue()->syncWindowRectangleWithGraphicsContext();

	// make this window's context not current
	// note: this must be done as we will probably make the context current from another thread
	//       and it is not allowed to have one context current in two threads
	if (!releaseContext())
		OSG_WARN << "Window realize: Can not release context." << std::endl;

	// restore previous context
	if (savedContext)
		const_cast<QGLContext*>(savedContext)->makeCurrent();

	return true;
}

bool GraphicsWindowQt1::isRealizedImplementation() const
{
	return _realized;
}

void GraphicsWindowQt1::closeImplementation()
{
	if (_widget)
		_widget->close();
	_realized = false;
}

void GraphicsWindowQt1::runOperations()
{
	// While in graphics thread this is last chance to do something useful before
	// graphics thread will execute its operations.
	if (_widget->getNumDeferredEvents() > 0)
		_widget->processDeferredEvents();

	if (QGLContext::currentContext() != _widget->context())
		_widget->makeCurrent();

	GraphicsWindow::runOperations();
}

bool GraphicsWindowQt1::makeCurrentImplementation()
{
	if (_widget->getNumDeferredEvents() > 0)
		_widget->processDeferredEvents();

	_widget->makeCurrent();

	return true;
}

bool GraphicsWindowQt1::releaseContextImplementation()
{
	if (!_widget) {
		return false; // 返回失败状态
	}

	_widget->doneCurrent();
	return true;
}

void GraphicsWindowQt1::swapBuffersImplementation()
{
	_widget->swapBuffers();

	// FIXME: the processDeferredEvents should really be executed in a GUI (main) thread context but
	// I couln't find any reliable way to do this. For now, lets hope non of *GUI thread only operations* will
	// be executed in a QGLWidget::event handler. On the other hand, calling GUI only operations in the
	// QGLWidget event handler is an indication of a Qt bug.
	if (_widget->getNumDeferredEvents() > 0)
		_widget->processDeferredEvents();

	// We need to call makeCurrent here to restore our previously current context
	// which may be changed by the processDeferredEvents function.
	if (QGLContext::currentContext() != _widget->context())
		_widget->makeCurrent();
}

void GraphicsWindowQt1::requestWarpPointer(float x, float y)
{
	if (_widget)
		QCursor::setPos(_widget->mapToGlobal(QPoint((int)x, (int)y)));
}
