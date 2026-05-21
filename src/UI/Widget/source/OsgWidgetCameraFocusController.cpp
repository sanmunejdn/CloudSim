#include "OsgWidgetCameraFocusController.h"

#include "OsgWidget.h"

void OsgWidgetCameraFocusController::focusCameraOnBackend(OsgWidget& self, const std::string& backendId)
{
	self.OsgScene::focusCameraOnBackend(backendId);
}
