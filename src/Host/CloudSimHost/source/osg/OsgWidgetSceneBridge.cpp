#include "OsgWidgetSceneBridge.h"

#include "BackendDataBase.h"
#include "OsgWidget.h"

#include <osg/Matrixd>

namespace
{
osg::Matrixd matrixFromColumnMajor16(const std::array<double, 16>& a)
{
	osg::Matrixd m;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			m(r, c) = a[static_cast<std::size_t>(c * 4 + r)];
		}
	}
	return m;
}

void matrixToColumnMajor16(const osg::Matrixd& m, std::array<double, 16>& out)
{
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out[static_cast<std::size_t>(c * 4 + r)] = m(r, c);
		}
	}
}
} // namespace

void OsgWidgetSceneBridge::setBackendRootWorldMatrixColumnMajor(const std::string& backendId,
	const std::array<double, 16>& columnMajor4x4)
{
	if (!m_widget)
	{
		return;
	}
	const osg::Matrixd world = matrixFromColumnMajor16(columnMajor4x4);
	m_widget->setBackendRootWorldMatrixFromWorld(backendId, world);
}

bool OsgWidgetSceneBridge::getBackendRootWorldMatrixColumnMajor(const std::string& backendId,
	std::array<double, 16>& outColumnMajor4x4) const
{
	if (!m_widget)
	{
		return false;
	}
	osg::Matrixd world;
	if (!m_widget->getBackendRootWorldMatrix(backendId, world))
	{
		return false;
	}
	matrixToColumnMajor16(world, outColumnMajor4x4);
	return true;
}

void OsgWidgetSceneBridge::setBackendObjectVisible(const std::string& backendId, bool visible)
{
	if (!m_widget)
	{
		return;
	}
	m_widget->setBackendObjectVisible(backendId, visible);
}

void OsgWidgetSceneBridge::removeBackendObjectVisual(const std::string& backendId)
{
	if (!m_widget)
	{
		return;
	}
	m_widget->removeBackendObjectVisual(backendId);
}

bool OsgWidgetSceneBridge::hasBackendObjectBranch(const std::string& backendId) const
{
	if (!m_widget)
	{
		return false;
	}
	return m_widget->hasBackendObjectBranch(backendId);
}

bool OsgWidgetSceneBridge::tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
	double& outCz) const
{
	if (!m_widget)
	{
		return false;
	}
	return m_widget->tryGetBackendModelCenterMm(backendId, outCx, outCy, outCz);
}

void OsgWidgetSceneBridge::syncOuterPatFromBackend(const BackendDataBase& data)
{
	if (!m_widget)
	{
		return;
	}
	(void)m_widget->syncOuterPatFromBackend(data);
}

void OsgWidgetSceneBridge::setBackendParent(const std::string& backendId, const std::string& parentBackendId)
{
	if (!m_widget)
	{
		return;
	}
	m_widget->setBackendParent(backendId, parentBackendId);
}
