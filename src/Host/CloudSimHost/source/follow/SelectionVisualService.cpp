/// @file SelectionVisualService.cpp
/// @brief SelectionVisualService 实现

#include "SelectionVisualService.h"

#include "DocumentHost.h"

namespace cloudsim::host
{
void SelectionVisualService::ensureSelectionVisual(DocumentHost& host, const core::ObjectId& backendId,
												   const bool urdfLinkMesh)
{
	host.ensureSelectionVisualForBackend(backendId.toStdString(), urdfLinkMesh);
}

} // namespace cloudsim::host
