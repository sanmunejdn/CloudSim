#include "adapters/OsgRenderViewAdapter.h"

#include "OsgWidget.h"

#include <osg/Matrixd>

namespace cloudsim::host {

OsgRenderViewAdapter::OsgRenderViewAdapter(OsgWidget& widget) : m_widget(widget) {}

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
		m.ptr()[i] = columnMajor[static_cast<size_t>(i)];
	m_widget.setBackendRootWorldMatrixFromWorld(id.toStdString(), m);
}

bool OsgRenderViewAdapter::getWorldMatrix(const core::ObjectId& id, core::Mat4& outColumnMajor) const
{
	osg::Matrixd m;
	if (!m_widget.getBackendRootWorldMatrix(id.toStdString(), m))
		return false;
	for (int i = 0; i < 16; ++i)
		outColumnMajor[static_cast<size_t>(i)] = m.ptr()[i];
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

bool OsgRenderViewAdapter::tryGetModelCenterMm(const core::ObjectId& id, double& outCx, double& outCy, double& outCz) const
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

} // namespace cloudsim::host
