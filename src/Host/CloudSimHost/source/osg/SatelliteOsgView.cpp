/// @file SatelliteOsgView.cpp
/// @brief 轻量卫星视口实现

#include "SatelliteOsgView.h"

#include "GraphicsWindowQt1.h"
#include "OsgScene.h"
#include "OsgWidget.h"
#include "QWidgetViewer.h"

#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QPalette>
#include <QShowEvent>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

#include <osg/CopyOp>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/StateSet>
#include <osg/View>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

namespace
{
QColor chromeColor(bool dark)
{
	return dark ? QColor(36, 36, 41) : QColor(237, 237, 240);
}

void applyGlChrome(QWidgetViewer* gl, bool dark)
{
	if (!gl)
	{
		return;
	}
	QPalette pal = gl->palette();
	pal.setColor(QPalette::Window, chromeColor(dark));
	gl->setPalette(pal);
}

/// 跟主相机旋转，使角标 HUD 随本格 Trackball 动
class HudFollowMainCameraCallback : public osg::NodeCallback
{
public:
	explicit HudFollowMainCameraCallback(osg::Camera* mainCamera) : m_mainCamera(mainCamera) {}

	void operator()(osg::Node* node, osg::NodeVisitor* nv) override
	{
		auto* hud = static_cast<osg::Camera*>(node);
		if (m_mainCamera.valid())
		{
			hud->setViewMatrix(osg::Matrix::rotate(m_mainCamera->getViewMatrix().getRotate()));
		}
		traverse(node, nv);
	}

private:
	osg::observer_ptr<osg::Camera> m_mainCamera;
};

void applyHudSquareOrtho(osg::Camera* camera, float halfExtent, int viewportWidth, int viewportHeight)
{
	if (!camera || viewportWidth <= 0 || viewportHeight <= 0)
	{
		return;
	}
	const double aspect = static_cast<double>(viewportWidth) / static_cast<double>(viewportHeight);
	const double half = static_cast<double>(halfExtent);
	if (aspect >= 1.0)
	{
		camera->setProjectionMatrixAsOrtho(static_cast<float>(-half * aspect), static_cast<float>(half * aspect),
										   -halfExtent, halfExtent, -10.0, 10.0);
	}
	else
	{
		camera->setProjectionMatrixAsOrtho(-halfExtent, halfExtent, static_cast<float>(-half / aspect),
										   static_cast<float>(half / aspect), -10.0, 10.0);
	}
}

struct HudCornerVp
{
	int x = 0;
	int y = 0;
	int width = 1;
	int height = 1;
};

HudCornerVp computeCornerViewport(int fbW, int fbH, int marginLogical, int sizeLogical, bool topRight, double dpr)
{
	fbW = (std::max)(1, fbW);
	fbH = (std::max)(1, fbH);
	dpr = dpr > 0.0 ? dpr : 1.0;
	const int marginPx = static_cast<int>(std::lround(static_cast<double>(marginLogical) * dpr));
	const int safeMargin = (std::min)(marginPx, (std::max)(0, (std::min)(fbW, fbH) / 2 - 4));
	const int nominalPx = static_cast<int>(std::lround(static_cast<double>(sizeLogical) * dpr));
	const int maxFit = (std::min)(fbW - 2 * safeMargin, fbH - 2 * safeMargin);
	const int capPx = static_cast<int>(static_cast<double>((std::min)(fbW, fbH)) * 0.38);
	int sizePx = nominalPx;
	if (maxFit > 0)
	{
		sizePx = (std::min)({nominalPx, maxFit, capPx});
		sizePx = (std::max)(static_cast<int>(std::lround(32.0 * dpr)), sizePx);
		sizePx = (std::min)(sizePx, maxFit);
	}
	HudCornerVp out;
	out.width = sizePx;
	out.height = sizePx;
	if (topRight)
	{
		out.x = (std::max)(0, fbW - safeMargin - sizePx);
		out.y = (std::max)(0, fbH - safeMargin - sizePx);
	}
	else
	{
		out.x = safeMargin;
		out.y = safeMargin;
	}
	return out;
}

/// 深拷贝 Primary HUD 内容，保证线框立方体/中文面标/箭头轴样式一致
osg::ref_ptr<osg::Node> clonePrimaryHudContent(osg::Camera* primaryHudCamera)
{
	if (!primaryHudCamera || primaryHudCamera->getNumChildren() == 0)
	{
		return nullptr;
	}
	osg::ref_ptr<osg::Node> cloned =
		static_cast<osg::Node*>(primaryHudCamera->getChild(0)->clone(osg::CopyOp::DEEP_COPY_ALL));
	return cloned;
}
} // namespace

SatelliteOsgView::SatelliteOsgView(OsgWidget* primary, QWidget* parent)
	: QWidget(parent), m_primary(primary)
{
	setMinimumSize(120, 90);
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	setFocusPolicy(Qt::StrongFocus);

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(0);

	QSurfaceFormat fmt = QSurfaceFormat::defaultFormat();
	fmt.setDepthBufferSize(24);
	fmt.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
	fmt.setRenderableType(QSurfaceFormat::OpenGL);
	fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
	if (primary && primary->viewportWidget())
	{
		if (auto* gl = qobject_cast<QOpenGLWidget*>(primary->viewportWidget()))
		{
			fmt = gl->format();
		}
	}

	m_glWidget = new QWidgetViewer(fmt, this);
	m_glWidget->setMinimumSize(80, 60);
	m_glWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_glWidget->setAttribute(Qt::WA_OpaquePaintEvent);
	m_glWidget->setAttribute(Qt::WA_NoSystemBackground, true);
	layout->addWidget(m_glWidget, 1);
	m_glWidget->installEventFilter(this);
	installEventFilter(this);

	initViewer(primary);

	if (primary)
	{
		connect(primary, &OsgWidget::sceneRedrawRequested, this,
				[this]()
				{
					syncBackgroundFromPrimary();
					requestRedraw();
				});
	}
}

SatelliteOsgView::~SatelliteOsgView()
{
	if (m_glWidget)
	{
		m_glWidget->removeEventFilter(this);
	}
	if (m_viewer.valid())
	{
		m_viewer->setDone(true);
		m_viewer->setSceneData(nullptr);
		m_viewer = nullptr;
	}
	m_graphicsWindow = nullptr;
	m_manipulator = nullptr;
	m_localRoot = nullptr;
	m_bgCamera = nullptr;
	m_bgGeom = nullptr;
	m_axesHudCamera = nullptr;
	m_viewCubeHudCamera = nullptr;
}

void SatelliteOsgView::initViewer(OsgWidget* primary)
{
	m_viewer = new osgViewer::Viewer;
	m_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
	m_viewer->setRunFrameScheme(osgViewer::Viewer::ON_DEMAND);

	// 构造前挂 sharedContext，否则 init 已按独立 contextID 建 State
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = GraphicsWindowQt1::createTraits(m_glWidget);
	traits->inheritedWindowData = new GraphicsWindowQt1::WindowData(m_glWidget, this);
	if (primary && primary->m_graphicsWindow.valid())
	{
		traits->sharedContext = primary->m_graphicsWindow.get();
	}
	auto* gwQt = new GraphicsWindowQt1(traits.get(), this);
	m_graphicsWindow = gwQt;
	gwQt->setViewer(m_viewer.get());
	m_glWidget->setGraphicsWindow(gwQt);

	connect(m_glWidget, &QWidgetViewer::windowResized, this,
			[this](int w, int h)
			{
				if (m_graphicsWindow.valid())
				{
					static_cast<GraphicsWindowQt1*>(m_graphicsWindow.get())->updateSize(w, h);
				}
				if (m_viewer.valid() && m_viewer->getCamera())
				{
					m_viewer->getCamera()->setViewport(0, 0, w, h);
					const double aspect =
						static_cast<double>((std::max)(1, w)) / static_cast<double>((std::max)(1, h));
					m_viewer->getCamera()->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
				}
				if (m_bgCamera.valid())
				{
					m_bgCamera->setViewport(0, 0, w, h);
				}
				updateLocalHudViewport(w, h);
			});

	m_viewer->getCamera()->setGraphicsContext(m_graphicsWindow.get());
	// 排除 Primary 专用 HUD/渐变（绑错 GC/viewport）
	m_viewer->getCamera()->setCullMask(~OsgScene::kMaskPrimaryViewportOnly);

	const int w = (std::max)(1, m_glWidget->width());
	const int h = (std::max)(1, m_glWidget->height());
	m_viewer->getCamera()->setViewport(0, 0, w, h);
	m_viewer->getCamera()->setProjectionMatrixAsPerspective(
		30.0, static_cast<double>(w) / static_cast<double>(h), 10.0, 1e8);

	m_viewer->setLightingMode(osg::View::HEADLIGHT);

	osg::Group* sharedRoot = nullptr;
	if (primary)
	{
		sharedRoot = const_cast<osg::Group*>(primary->sceneGraphRoot());
	}
	ensureLocalBackground(primary ? primary->darkUiTheme() : false);
	ensureLocalHud();
	bindSharedSceneUnderLocalRoot(sharedRoot);
	syncBackgroundFromPrimary();
	updateLocalHudViewport(w, h);

	m_manipulator = new osgGA::TrackballManipulator;
	m_manipulator->setAllowThrow(false);
	m_viewer->setCameraManipulator(m_manipulator.get());
	m_viewer->getEventQueue()->syncWindowRectangleWithGraphicsContext();
	applyHomeView();
}

void SatelliteOsgView::ensureLocalBackground(bool dark)
{
	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return;
	}

	if (!m_localRoot.valid())
	{
		m_localRoot = new osg::Group;
	}

	if (!m_bgCamera.valid())
	{
		m_bgCamera = new osg::Camera;
		m_bgCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
		m_bgCamera->setProjectionMatrixAsOrtho2D(-1.0, 1.0, -1.0, 1.0);
		m_bgCamera->setViewMatrix(osg::Matrix::identity());
		m_bgCamera->setRenderOrder(osg::Camera::PRE_RENDER);
		m_bgCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
		m_bgCamera->setGraphicsContext(m_graphicsWindow.get());

		m_bgGeom = new osg::Geometry;
		osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
		vertices->push_back(osg::Vec3(-1.0f, -1.0f, 0.0f));
		vertices->push_back(osg::Vec3(1.0f, -1.0f, 0.0f));
		vertices->push_back(osg::Vec3(1.0f, -0.15f, 0.0f));
		vertices->push_back(osg::Vec3(-1.0f, -0.15f, 0.0f));
		vertices->push_back(osg::Vec3(1.0f, 1.0f, 0.0f));
		vertices->push_back(osg::Vec3(-1.0f, 1.0f, 0.0f));
		m_bgGeom->setVertexArray(vertices);

		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(6);
		m_bgGeom->setColorArray(colors);
		m_bgGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

		osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
		normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
		m_bgGeom->setNormalArray(normals);
		m_bgGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
		m_bgGeom->addPrimitiveSet(new osg::DrawArrays(osg::DrawArrays::TRIANGLE_STRIP, 0, 6));

		osg::ref_ptr<osg::StateSet> ss = m_bgGeom->getOrCreateStateSet();
		ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		ss->setRenderBinDetails(-1, "RenderBin");

		osg::ref_ptr<osg::Geode> geode = new osg::Geode;
		geode->addDrawable(m_bgGeom);
		m_bgCamera->addChild(geode);
		m_localRoot->insertChild(0, m_bgCamera);
	}

	const int w = (std::max)(1, m_glWidget ? m_glWidget->width() : 1);
	const int h = (std::max)(1, m_glWidget ? m_glWidget->height() : 1);
	m_bgCamera->setViewport(0, 0, w, h);
	updateLocalGradientColors(dark);
	m_darkBackground = dark;
}

void SatelliteOsgView::updateLocalGradientColors(bool dark)
{
	if (!m_bgGeom.valid())
	{
		return;
	}
	auto* colors = dynamic_cast<osg::Vec4Array*>(m_bgGeom->getColorArray());
	if (!colors || colors->size() < 6)
	{
		return;
	}
	if (dark)
	{
		(*colors)[0] = osg::Vec4(0.10f, 0.10f, 0.11f, 1.0f);
		(*colors)[1] = osg::Vec4(0.10f, 0.10f, 0.11f, 1.0f);
		(*colors)[2] = osg::Vec4(0.15f, 0.16f, 0.18f, 1.0f);
		(*colors)[3] = osg::Vec4(0.15f, 0.16f, 0.18f, 1.0f);
		(*colors)[4] = osg::Vec4(0.22f, 0.24f, 0.29f, 1.0f);
		(*colors)[5] = osg::Vec4(0.22f, 0.24f, 0.29f, 1.0f);
	}
	else
	{
		(*colors)[0] = osg::Vec4(0.96f, 0.95f, 0.94f, 1.0f);
		(*colors)[1] = osg::Vec4(0.96f, 0.95f, 0.94f, 1.0f);
		(*colors)[2] = osg::Vec4(0.92f, 0.93f, 0.94f, 1.0f);
		(*colors)[3] = osg::Vec4(0.92f, 0.93f, 0.94f, 1.0f);
		(*colors)[4] = osg::Vec4(0.84f, 0.87f, 0.92f, 1.0f);
		(*colors)[5] = osg::Vec4(0.84f, 0.87f, 0.92f, 1.0f);
	}
	m_bgGeom->dirtyDisplayList();
}

void SatelliteOsgView::bindSharedSceneUnderLocalRoot(osg::Group* sharedRoot)
{
	if (!m_viewer.valid())
	{
		return;
	}
	ensureLocalBackground(m_primary ? m_primary->darkUiTheme() : m_darkBackground);
	ensureLocalHud();

	// 仅保留本地 bg/HUD，再挂共享场景
	for (int i = static_cast<int>(m_localRoot->getNumChildren()) - 1; i >= 0; --i)
	{
		osg::Node* child = m_localRoot->getChild(static_cast<unsigned int>(i));
		if (child == m_bgCamera.get() || child == m_axesHudCamera.get() || child == m_viewCubeHudCamera.get())
		{
			continue;
		}
		m_localRoot->removeChild(static_cast<unsigned int>(i));
	}
	if (sharedRoot)
	{
		m_localRoot->addChild(sharedRoot);
	}
	m_viewer->setSceneData(m_localRoot.get());
}

void SatelliteOsgView::ensureLocalHud()
{
	if (!m_viewer.valid() || !m_viewer->getCamera() || !m_graphicsWindow.valid())
	{
		return;
	}
	if (!m_localRoot.valid())
	{
		m_localRoot = new osg::Group;
	}

	if (!m_axesHudCamera.valid())
	{
		osg::ref_ptr<osg::Node> axesContent =
			m_primary ? clonePrimaryHudContent(m_primary->m_worldAxesHudCamera.get()) : nullptr;
		if (axesContent.valid())
		{
			m_axesHudCamera = new osg::Camera;
			m_axesHudCamera->setGraphicsContext(m_graphicsWindow.get());
			m_axesHudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
			m_axesHudCamera->setRenderOrder(osg::Camera::POST_RENDER, 1);
			m_axesHudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
			m_axesHudCamera->setAllowEventFocus(false);
			m_axesHudCamera->setProjectionMatrixAsOrtho(-1.2f, 1.2f, -1.2f, 1.2f, -10.0, 10.0);
			m_axesHudCamera->setCullMask(0xffffffffu);
			m_axesHudCamera->setUpdateCallback(new HudFollowMainCameraCallback(m_viewer->getCamera()));
			m_axesHudCamera->addChild(axesContent.get());
			m_localRoot->addChild(m_axesHudCamera.get());
		}
	}

	if (!m_viewCubeHudCamera.valid())
	{
		osg::ref_ptr<osg::Node> cubeContent =
			m_primary ? clonePrimaryHudContent(m_primary->m_viewCubeHudCamera.get()) : nullptr;
		if (cubeContent.valid())
		{
			m_viewCubeHudCamera = new osg::Camera;
			m_viewCubeHudCamera->setGraphicsContext(m_graphicsWindow.get());
			m_viewCubeHudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
			m_viewCubeHudCamera->setRenderOrder(osg::Camera::POST_RENDER, 2);
			m_viewCubeHudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
			m_viewCubeHudCamera->setAllowEventFocus(false);
			m_viewCubeHudCamera->setProjectionMatrixAsOrtho(-1.45f, 1.45f, -1.45f, 1.45f, -10.0, 10.0);
			m_viewCubeHudCamera->setCullMask(0xffffffffu);
			m_viewCubeHudCamera->setUpdateCallback(new HudFollowMainCameraCallback(m_viewer->getCamera()));
			m_viewCubeHudCamera->addChild(cubeContent.get());
			m_localRoot->addChild(m_viewCubeHudCamera.get());
		}
	}
}

void SatelliteOsgView::updateLocalHudViewport(int framebufferWidth, int framebufferHeight)
{
	const double dpr = m_glWidget ? QWidgetViewer::effectiveDevicePixelRatio(m_glWidget) : 1.0;
	// 与 OsgScene 主视口角标尺寸一致
	const int axesMargin = m_primary ? m_primary->m_worldAxesHudMargin : 8;
	const int axesSize = m_primary ? m_primary->m_worldAxesHudSize : 80;
	const int cubeMargin = m_primary ? m_primary->m_viewCubeHudMargin : 10;
	const int cubeSize = m_primary ? m_primary->m_viewCubeHudSize : 132;
	if (m_axesHudCamera.valid())
	{
		const HudCornerVp vp =
			computeCornerViewport(framebufferWidth, framebufferHeight, axesMargin, axesSize, false, dpr);
		m_axesHudCamera->setViewport(vp.x, vp.y, vp.width, vp.height);
		applyHudSquareOrtho(m_axesHudCamera.get(), 1.2f, vp.width, vp.height);
	}
	if (m_viewCubeHudCamera.valid())
	{
		const HudCornerVp vp =
			computeCornerViewport(framebufferWidth, framebufferHeight, cubeMargin, cubeSize, true, dpr);
		m_viewCubeHudCamera->setViewport(vp.x, vp.y, vp.width, vp.height);
		applyHudSquareOrtho(m_viewCubeHudCamera.get(), 1.45f, vp.width, vp.height);
	}
}

void SatelliteOsgView::syncBackgroundFromPrimary()
{
	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return;
	}

	const bool dark = m_primary ? m_primary->darkUiTheme() : false;
	const osg::Vec4 clearColor =
		dark ? osg::Vec4(0.14f, 0.14f, 0.16f, 1.0f) : osg::Vec4(0.93f, 0.93f, 0.94f, 1.0f);

	// Primary 已有清屏色时优先拷贝，避免主题切换瞬间不同步
	if (m_primary && m_primary->m_viewer.valid() && m_primary->m_viewer->getCamera())
	{
		m_viewer->getCamera()->setClearColor(m_primary->m_viewer->getCamera()->getClearColor());
	}
	else
	{
		m_viewer->getCamera()->setClearColor(clearColor);
	}

	if (dark != m_darkBackground || !m_bgCamera.valid())
	{
		ensureLocalBackground(dark);
	}
	else
	{
		updateLocalGradientColors(dark);
	}
	applyGlChrome(m_glWidget, dark);
	m_darkBackground = dark;
}

void SatelliteOsgView::applyHomeView()
{
	applyViewPreset(ViewPreset::Iso);
}

void SatelliteOsgView::setSharedSceneRoot(osg::Group* root)
{
	if (!m_viewer.valid())
	{
		return;
	}
	bindSharedSceneUnderLocalRoot(root);
	syncBackgroundFromPrimary();
	requestRedraw();
}

void SatelliteOsgView::setViewDirection(const osg::Vec3d& eyeDirectionFromCenter, const osg::Vec3d& upHint)
{
	if (!m_manipulator.valid())
	{
		return;
	}
	osg::Vec3d presetDir = eyeDirectionFromCenter;
	if (presetDir.length2() < 1e-12)
	{
		return;
	}
	presetDir.normalize();

	osg::Vec3d eye;
	osg::Vec3d center;
	osg::Vec3d upCurrent;
	m_manipulator->getTransformation(eye, center, upCurrent);

	osg::Vec3d offset = eye - center;
	double dist = offset.length();
	if (dist < 1e-3 || !std::isfinite(dist))
	{
		dist = 3000.0;
	}

	osg::Vec3d up = upHint;
	if (up.length2() < 1e-12)
	{
		up.set(0.0, 0.0, 1.0);
		if (std::abs(presetDir.z()) > 0.9)
		{
			up.set(0.0, 1.0, 0.0);
		}
	}
	up.normalize();

	const osg::Vec3d forward = -presetDir;
	if (std::abs(forward * up) > 0.95)
	{
		up.set(0.0, 1.0, 0.0);
	}

	m_manipulator->setTransformation(center + presetDir * dist, center, up);
	requestRedraw();
}

void SatelliteOsgView::applyViewPreset(ViewPreset preset)
{
	osg::Vec3d dir(1.0, 1.0, 1.05);
	osg::Vec3d up(0.0, 0.0, 1.0);
	switch (preset)
	{
	case ViewPreset::Front:
		dir.set(0.0, 1.0, 0.0);
		break;
	case ViewPreset::Top:
		dir.set(0.0, 0.0, 1.0);
		up.set(0.0, 1.0, 0.0);
		break;
	case ViewPreset::Right:
		dir.set(1.0, 0.0, 0.0);
		break;
	case ViewPreset::Reset:
	case ViewPreset::Iso:
	default:
		dir.set(1.0, 1.0, 1.05);
		dir.normalize();
		break;
	}
	setViewDirection(dir, up);
}

osg::Matrixd SatelliteOsgView::cameraMatrix() const
{
	if (!m_manipulator.valid())
	{
		return osg::Matrixd();
	}
	osg::Vec3d eye;
	osg::Vec3d center;
	osg::Vec3d up;
	m_manipulator->getTransformation(eye, center, up);
	osg::Matrixd m;
	m.makeLookAt(eye, center, up);
	return m;
}

void SatelliteOsgView::setCameraMatrix(const osg::Matrixd& matrix)
{
	if (!m_manipulator.valid())
	{
		return;
	}
	osg::Vec3d eye;
	osg::Vec3d center;
	osg::Vec3d up;
	matrix.getLookAt(eye, center, up);
	m_manipulator->setTransformation(eye, center, up);
	requestRedraw();
}

void SatelliteOsgView::requestRedraw()
{
	if (m_glWidget)
	{
		m_glWidget->update();
	}
}

bool SatelliteOsgView::eventFilter(QObject* watched, QEvent* event)
{
	if (event && (event->type() == QEvent::MouseButtonPress || event->type() == QEvent::FocusIn))
	{
		emit activated();
	}
	return QWidget::eventFilter(watched, event);
}

void SatelliteOsgView::showEvent(QShowEvent* event)
{
	QWidget::showEvent(event);
	// Primary HUD 可能晚于卫星构造完成，显示时再补拷一次
	ensureLocalHud();
	syncBackgroundFromPrimary();
	if (m_glWidget)
	{
		int w = 0;
		int h = 0;
		if (!m_glWidget->resolveOpenGlFramebufferSize(w, h) && !m_glWidget->queryFramebufferPixelSize(w, h))
		{
			const double dpr = QWidgetViewer::effectiveDevicePixelRatio(m_glWidget);
			w = static_cast<int>(std::lround(static_cast<double>(m_glWidget->width()) * dpr));
			h = static_cast<int>(std::lround(static_cast<double>(m_glWidget->height()) * dpr));
		}
		updateLocalHudViewport(w, h);
	}
	requestRedraw();
}
