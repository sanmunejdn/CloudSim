#include "OsgWidgetBackendLoadController.h"

#include "OsgWidget.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

bool OsgWidgetBackendLoadController::loadPointCloudFromBackendData(
	OsgWidget& self,
	const PointCloudBackendData& data,
	QString* errorMessage,
	bool resetViewToHome)
{
	self.clearStagingGeometry();
	if (!self.upsertPointCloudBranchInScene(data, errorMessage, resetViewToHome))
	{
		return false;
	}
	self.setSelectionActive(true);
	self.syncGizmoAndPickFromBackend(data);
	self.syncCompassGizmoOrientation();
	self.clearPointAnnotations();
	self.clearPointPickMarker();
	if (self.m_viewer.valid())
	{
		self.m_viewer->setSceneData(self.m_root.get());
	}
	self.requestRedraw();
	return true;
}

bool OsgWidgetBackendLoadController::loadMeshFromBackendData(
	OsgWidget& self,
	const MeshBackendData& data,
	QString* errorMessage,
	bool resetViewToHome,
	bool showWireOutline,
	bool useSceneLighting,
	bool skipInnerModelCenterRebase)
{
	self.clearStagingGeometry();
	if (!self.upsertMeshBranchInScene(data, errorMessage, resetViewToHome, showWireOutline, useSceneLighting,
			skipInnerModelCenterRebase))
	{
		return false;
	}
	self.setSelectionActive(true);
	self.syncGizmoAndPickFromBackend(data);
	self.syncCompassGizmoOrientation();
	self.clearPointAnnotations();
	self.clearPointPickMarker();
	if (self.m_viewer.valid())
	{
		self.m_viewer->setSceneData(self.m_root.get());
	}
	self.requestRedraw();
	return true;
}

