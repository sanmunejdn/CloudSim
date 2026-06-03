#include "adapters/OsgRenderViewAdapter.h"

#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "IDataService.h"
#include "OsgWidget.h"

#include "BackendDataManager.h"

#include <osg/Matrixd>
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/AutoTransform>

namespace cloudsim::host {

OsgRenderViewAdapter::OsgRenderViewAdapter(OsgWidget& widget) : m_widget(widget) {}

OsgRenderViewAdapter::OsgRenderViewAdapter(OsgWidget& widget, DocumentHost& host)
	: m_widget(widget)
	, m_host(&host)
{
}

OsgRenderViewAdapter::OsgRenderViewAdapter(DocumentHost& host)
	: OsgRenderViewAdapter(*osgWidgetFrom(host), host)
{
}

QWidget* OsgRenderViewAdapter::widget()
{
	return &m_widget;
}

const QWidget* OsgRenderViewAdapter::widget() const
{
	return &m_widget;
}

void OsgRenderViewAdapter::setWorldMatrix(const core::ObjectId& id, const core::Mat4& columnMajor)
{
	osg::Matrixd m;
	for (int i = 0; i < 16; ++i)
	{
		m.ptr()[i] = columnMajor[static_cast<size_t>(i)];
	}
	m_widget.setBackendRootWorldMatrixFromWorld(id.toStdString(), m);
}

bool OsgRenderViewAdapter::getWorldMatrix(const core::ObjectId& id, core::Mat4& outColumnMajor) const
{
	osg::Matrixd m;
	if (!m_widget.getBackendRootWorldMatrix(id.toStdString(), m))
	{
		return false;
	}
	for (int i = 0; i < 16; ++i)
	{
		outColumnMajor[static_cast<size_t>(i)] = m.ptr()[i];
	}
	return true;
}

void OsgRenderViewAdapter::setVisible(const core::ObjectId& id, bool visible)
{
	m_widget.setBackendObjectVisible(id.toStdString(), visible);
}

void OsgRenderViewAdapter::removeVisual(const core::ObjectId& id)
{
	m_widget.removeBackendObjectVisual(id.toStdString());
}

bool OsgRenderViewAdapter::hasVisualBranch(const core::ObjectId& id) const
{
	return m_widget.hasBackendObjectBranch(id.toStdString());
}

bool OsgRenderViewAdapter::tryGetModelCenterMm(const core::ObjectId& id, double& outCx, double& outCy,
	double& outCz) const
{
	return m_widget.tryGetBackendModelCenterMm(id.toStdString(), outCx, outCy, outCz);
}

void OsgRenderViewAdapter::setPickHandler(core::PickHandler handler)
{
	m_pickHandler = std::move(handler);
}

void OsgRenderViewAdapter::clearPickHandler()
{
	m_pickHandler = nullptr;
}

void OsgRenderViewAdapter::requestRedraw()
{
	m_widget.requestRedraw();
}

void OsgRenderViewAdapter::focusCameraOnBackend(const core::ObjectId& id)
{
	m_widget.focusCameraOnBackend(id.toStdString());
}

void OsgRenderViewAdapter::setBackendLogicalParent(const core::ObjectId& childId, const core::ObjectId& parentId)
{
	m_widget.setBackendLogicalParent(childId.toStdString(), parentId.toStdString());
}

namespace {

QString formatMatrix(const osg::Matrixd& m)
{
	QString s;
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			if (c > 0)
			{
				s += QLatin1Char(' ');
			}
			s += QString::number(m(r, c), 'g', 6);
		}
		if (r < 3)
		{
			s += QLatin1Char('\n');
		}
	}
	return s;
}

QString localMatrixSummary(const osg::Node* node)
{
	if (!node)
	{
		return QStringLiteral("—");
	}
	if (const auto* cam = dynamic_cast<const osg::Camera*>(node))
	{
		return QStringLiteral("View:\n%1\nProj:\n%2").arg(formatMatrix(cam->getViewMatrix()))
			.arg(formatMatrix(cam->getProjectionMatrix()));
	}
	if (const auto* mt = dynamic_cast<const osg::MatrixTransform*>(node))
	{
		return formatMatrix(mt->getMatrix());
	}
	if (const auto* pat = dynamic_cast<const osg::PositionAttitudeTransform*>(node))
	{
		return formatMatrix(osg::Matrixd::translate(pat->getPosition()) * osg::Matrixd::rotate(pat->getAttitude())
			* osg::Matrixd::scale(pat->getScale()));
	}
	if (const auto* at = dynamic_cast<const osg::AutoTransform*>(node))
	{
		return formatMatrix(osg::Matrixd::translate(at->getPosition()) * osg::Matrixd::rotate(at->getRotation())
			* osg::Matrixd::scale(at->getScale()));
	}
	return QStringLiteral("—");
}

void buildSnapshotRecursive(core::IRenderView::SceneNodeInfo& info, const osg::Node* node, int depthLeft)
{
	if (!node || depthLeft <= 0)
	{
		return;
	}
	info.className = QString::fromLatin1(node->className());
	info.name = QString::fromStdString(node->getName());
	info.localMatrixSummary = localMatrixSummary(node);
	if (const auto* g = node->asGroup())
	{
		for (unsigned i = 0; i < g->getNumChildren(); ++i)
		{
			core::IRenderView::SceneNodeInfo child;
			buildSnapshotRecursive(child, g->getChild(i), depthLeft - 1);
			info.children.push_back(std::move(child));
		}
	}
}

} // namespace

core::IRenderView::SceneNodeInfo OsgRenderViewAdapter::sceneGraphSnapshot(int maxDepth) const
{
	SceneNodeInfo root;
	const osg::Node* sceneRoot = m_widget.sceneGraphRoot();
	buildSnapshotRecursive(root, sceneRoot, maxDepth);
	return root;
}

bool OsgRenderViewAdapter::selectedPosition(float& outX, float& outY, float& outZ) const
{
	const osg::Vec3f p = m_widget.selectedPosition();
	outX = p.x();
	outY = p.y();
	outZ = p.z();
	return true;
}

bool OsgRenderViewAdapter::selectedRotationEulerDeg(float& outRx, float& outRy, float& outRz) const
{
	const osg::Vec3f r = m_widget.selectedRotationEulerDeg();
	outRx = r.x();
	outRy = r.y();
	outRz = r.z();
	return true;
}

void OsgRenderViewAdapter::ensureSelectionVisualForBackend(const core::ObjectId& id, const bool urdfLinkMesh)
{
	if (m_host)
	{
		m_host->ensureSelectionVisualForBackend(id.toStdString(), urdfLinkMesh);
	}
}

bool OsgRenderViewAdapter::syncOuterPatFromBackend(const core::ObjectId& id)
{
	if (m_host)
	{
		return m_host->syncOuterPatFromBackendId(id.toStdString());
	}
	return false;
}

core::GeometryKind OsgRenderViewAdapter::geometryKindForBackend(const core::ObjectId& id) const
{
	if (m_host)
	{
		return m_host->data().geometryKind(id);
	}
	return core::GeometryKind::None;
}

bool OsgRenderViewAdapter::commitGizmoPoseToBackend(const core::ObjectId& id)
{
	if (!m_host)
	{
		return false;
	}
	const auto obj = m_host->backend().getData(id.toStdString());
	if (!obj)
	{
		return false;
	}
	return m_widget.writeActiveBackendPoseFromOsg(*obj);
}

} // namespace cloudsim::host
