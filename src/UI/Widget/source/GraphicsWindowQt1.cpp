/// @file GraphicsWindowQt1.cpp
/// @brief GraphicsWindowQt1 实现

#include "GraphicsWindowQt1.h"

#include <QInputEvent>
#include <QOpenGLContext>
#include <QPointer>
#include <cmath>

#include <osg/DeleteHandler>
#include <osgViewer/ViewerBase>

GraphicsWindowQt1::GraphicsWindowQt1(osg::GraphicsContext::Traits* traits, QWidget* parent, Qt::WindowFlags f)
	: _realized(false)
{
	_widget = NULL;
	_traits = traits;
	init(parent, f);
}

GraphicsWindowQt1::GraphicsWindowQt1(QWidgetViewer* widget) : _realized(false)
{
	_widget = widget;
	_traits = _widget ? createTraits(_widget) : new osg::GraphicsContext::Traits;
	init(NULL, 0);
}

GraphicsWindowQt1::~GraphicsWindowQt1()
{
	if (_widget)
	{
		_widget->_gw = nullptr;
		_widget = nullptr;
	}

	if (isRealized())
	{
		releaseContext();
		closeImplementation();
	}
}

bool GraphicsWindowQt1::init(QWidget* parent, Qt::WindowFlags f)
{
	WindowData* windowData = nullptr;
	if (!_widget && _traits.get())
	{
		windowData = dynamic_cast<WindowData*>(_traits->inheritedWindowData.get());
	}
	if (!_widget)
	{
		_widget = windowData ? windowData->_widget : NULL;
	}
	if (!parent)
	{
		parent = windowData ? windowData->_parent : NULL;
	}

	_ownsWidget = _widget == NULL;
	if (!_widget)
	{
		Qt::WindowFlags flags = f | Qt::Window | Qt::CustomizeWindowHint;

		_traits->windowDecoration = false;
		if (_traits->windowDecoration)
		{
			flags |= Qt::WindowTitleHint | Qt::WindowMinMaxButtonsHint | Qt::WindowSystemMenuHint
#if (QT_VERSION_CHECK(4, 5, 0) <= QT_VERSION)
					 | Qt::WindowCloseButtonHint
#endif
				;
		}

		_widget = new QWidgetViewer(traits2qSurfaceFormat(_traits.get()), parent, flags);
	}

	if (_ownsWidget)
	{
		_widget->setWindowTitle(_traits->windowName.c_str());
		_widget->move(_traits->x, _traits->y);
		if (!_traits->supportsResize)
		{
			_widget->setFixedSize(_traits->width, _traits->height);
		}
		else
		{
			_widget->resize(_traits->width, _traits->height);
		}
	}

	_widget->setMouseTracking(true);
	_widget->setFocusPolicy(Qt::WheelFocus);
	_widget->setGraphicsWindow(this);
	useCursor(_traits->useCursor);

	setState(new osg::State);
	getState()->setGraphicsContext(this);

	if (_traits.valid() && _traits->sharedContext.valid())
	{
		getState()->setContextID(_traits->sharedContext->getState()->getContextID());
		incrementContextIDUsageCount(getState()->getContextID());
	}
	else
	{
		getState()->setContextID(osg::GraphicsContext::createNewContextID());
	}

	getEventQueue()->syncWindowRectangleWithGraphicsContext();

	return true;
}

QSurfaceFormat GraphicsWindowQt1::traits2qSurfaceFormat(const osg::GraphicsContext::Traits* traits)
{
	QSurfaceFormat format = QSurfaceFormat::defaultFormat();

	if (traits->alpha > 0)
	{
		format.setAlphaBufferSize(traits->alpha);
	}
	if (traits->red > 0)
	{
		format.setRedBufferSize(traits->red);
	}
	if (traits->green > 0)
	{
		format.setGreenBufferSize(traits->green);
	}
	if (traits->blue > 0)
	{
		format.setBlueBufferSize(traits->blue);
	}
	if (traits->depth > 0)
	{
		format.setDepthBufferSize(traits->depth);
	}
	if (traits->stencil > 0)
	{
		format.setStencilBufferSize(traits->stencil);
	}
	if (traits->sampleBuffers > 0)
	{
		format.setSamples(traits->samples);
	}

	format.setSwapBehavior(traits->doubleBuffer ? QSurfaceFormat::DoubleBuffer : QSurfaceFormat::SingleBuffer);
	format.setSwapInterval(traits->vsync ? 1 : 0);
	format.setStereo(traits->quadBufferStereo);
	format.setRenderableType(QSurfaceFormat::OpenGL);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);

	return format;
}

void GraphicsWindowQt1::qSurfaceFormat2traits(const QSurfaceFormat& format, osg::GraphicsContext::Traits* traits)
{
	traits->red = format.redBufferSize();
	traits->green = format.greenBufferSize();
	traits->blue = format.blueBufferSize();
	traits->alpha = format.alphaBufferSize() > 0 ? format.alphaBufferSize() : 0;
	traits->depth = format.depthBufferSize() > 0 ? format.depthBufferSize() : 0;
	traits->stencil = format.stencilBufferSize() > 0 ? format.stencilBufferSize() : 0;

	traits->sampleBuffers = format.samples() > 0 ? 1 : 0;
	traits->samples = format.samples();

	traits->quadBufferStereo = format.stereo();
	traits->doubleBuffer = format.swapBehavior() != QSurfaceFormat::SingleBuffer;
	traits->vsync = format.swapInterval() >= 1;
}

osg::GraphicsContext::Traits* GraphicsWindowQt1::createTraits(const QWidgetViewer* widget)
{
	osg::GraphicsContext::Traits* traits = new osg::GraphicsContext::Traits{};

	qSurfaceFormat2traits(widget->format(), traits);

	const QRect r = widget->geometry();
	traits->x = r.x();
	traits->y = r.y();
	const qreal dpr = QWidgetViewer::effectiveDevicePixelRatio(widget);
	traits->width = static_cast<int>(std::lround(static_cast<double>(r.width()) * dpr));
	traits->height = static_cast<int>(std::lround(static_cast<double>(r.height()) * dpr));

	{
		const QByteArray titleUtf8 = widget->windowTitle().toUtf8();
		if (!titleUtf8.isEmpty())
		{
			traits->windowName.assign(titleUtf8.constData(), static_cast<size_t>(titleUtf8.size()));
		}
	}
	const Qt::WindowFlags flags = widget->windowFlags();
	traits->windowDecoration =
		(flags & Qt::WindowTitleHint) && (flags & Qt::WindowMinMaxButtonsHint) && (flags & Qt::WindowSystemMenuHint);
	const QSizePolicy sp = widget->sizePolicy();
	traits->supportsResize = sp.horizontalPolicy() != QSizePolicy::Fixed || sp.verticalPolicy() != QSizePolicy::Fixed;

	return traits;
}

bool GraphicsWindowQt1::setWindowRectangleImplementation(int x, int y, int width, int height)
{
	if (_widget == NULL)
	{
		return false;
	}

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
	Qt::WindowFlags flags = Qt::Window | Qt::CustomizeWindowHint;
	if (windowDecoration)
	{
		flags |= Qt::WindowTitleHint | Qt::WindowMinMaxButtonsHint | Qt::WindowSystemMenuHint;
	}
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
	{
		_widget->setFocus(Qt::ActiveWindowFocusReason);
	}
}

void GraphicsWindowQt1::grabFocusIfPointerInWindow()
{
	if (_widget && _widget->underMouse())
	{
		_widget->setFocus(Qt::ActiveWindowFocusReason);
	}
}

void GraphicsWindowQt1::raiseWindow()
{
	if (_widget)
	{
		_widget->raise();
	}
}

void GraphicsWindowQt1::setWindowName(const std::string& name)
{
	if (_widget)
	{
		_widget->setWindowTitle(name.c_str());
	}
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
		if (!cursorOn)
		{
			_widget->setCursor(Qt::BlankCursor);
		}
		else
		{
			_widget->setCursor(_currentCursor);
		}
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
	case NoCursor:
		_currentCursor = Qt::BlankCursor;
		break;
	case RightArrowCursor:
	case LeftArrowCursor:
		_currentCursor = Qt::ArrowCursor;
		break;
	case InfoCursor:
		_currentCursor = Qt::SizeAllCursor;
		break;
	case DestroyCursor:
		_currentCursor = Qt::ForbiddenCursor;
		break;
	case HelpCursor:
		_currentCursor = Qt::WhatsThisCursor;
		break;
	case CycleCursor:
		_currentCursor = Qt::ForbiddenCursor;
		break;
	case SprayCursor:
		_currentCursor = Qt::SizeAllCursor;
		break;
	case WaitCursor:
		_currentCursor = Qt::WaitCursor;
		break;
	case TextCursor:
		_currentCursor = Qt::IBeamCursor;
		break;
	case CrosshairCursor:
		_currentCursor = Qt::CrossCursor;
		break;
	case HandCursor:
		_currentCursor = Qt::OpenHandCursor;
		break;
	case UpDownCursor:
		_currentCursor = Qt::SizeVerCursor;
		break;
	case LeftRightCursor:
		_currentCursor = Qt::SizeHorCursor;
		break;
	case TopSideCursor:
	case BottomSideCursor:
		_currentCursor = Qt::UpArrowCursor;
		break;
	case LeftSideCursor:
	case RightSideCursor:
		_currentCursor = Qt::SizeHorCursor;
		break;
	case TopLeftCorner:
		_currentCursor = Qt::SizeBDiagCursor;
		break;
	case TopRightCorner:
		_currentCursor = Qt::SizeFDiagCursor;
		break;
	case BottomRightCorner:
		_currentCursor = Qt::SizeBDiagCursor;
		break;
	case BottomLeftCorner:
		_currentCursor = Qt::SizeFDiagCursor;
		break;
	default:
		break;
	}
	if (_widget)
	{
		_widget->setCursor(_currentCursor);
	}
}

bool GraphicsWindowQt1::valid() const
{
	return _widget && _widget->isValid();
}

bool GraphicsWindowQt1::realizeImplementation()
{
	if (!_widget)
	{
		return false;
	}

	QOpenGLContext* savedContext = QOpenGLContext::currentContext();

	_realized = true;
	const bool result = makeCurrent();
	if (!result)
	{
		_realized = false;
		if (savedContext)
		{
			savedContext->makeCurrent(savedContext->surface());
		}
		OSG_WARN << "Window realize: Can make context current." << std::endl;
		return false;
	}

	getEventQueue()->syncWindowRectangleWithGraphicsContext();

	if (!releaseContext())
	{
		OSG_WARN << "Window realize: Can not release context." << std::endl;
	}

	if (savedContext)
	{
		savedContext->makeCurrent(savedContext->surface());
	}

	return true;
}

bool GraphicsWindowQt1::isRealizedImplementation() const
{
	return _realized;
}

void GraphicsWindowQt1::closeImplementation()
{
	// Viewer 析构时勿 _widget->close()：会向半析构的 OsgWidget 重入 eventFilter
	if (_widget)
	{
		_widget->_gw = nullptr;
	}
	_realized = false;
}

void GraphicsWindowQt1::runOperations()
{
	if (_widget->getNumDeferredEvents() > 0)
	{
		_widget->processDeferredEvents();
	}

	if (QOpenGLContext::currentContext() != _widget->context())
	{
		_widget->makeCurrent();
	}

	GraphicsWindow::runOperations();
}

bool GraphicsWindowQt1::makeCurrentImplementation()
{
	if (_widget->getNumDeferredEvents() > 0)
	{
		_widget->processDeferredEvents();
	}

	_widget->makeCurrent();
	return _widget->context() && _widget->context()->isValid();
}

bool GraphicsWindowQt1::releaseContextImplementation()
{
	if (!_widget)
	{
		return false;
	}

	_widget->doneCurrent();
	return true;
}

void GraphicsWindowQt1::swapBuffersImplementation()
{
	// paintGL 结束时 Qt 自动 swap；此处仅处理延迟事件
	if (_widget->getNumDeferredEvents() > 0)
	{
		_widget->processDeferredEvents();
	}
}

void GraphicsWindowQt1::requestWarpPointer(float x, float y)
{
	if (_widget)
	{
		QCursor::setPos(_widget->mapToGlobal(QPoint(static_cast<int>(x), static_cast<int>(y))));
	}
}
