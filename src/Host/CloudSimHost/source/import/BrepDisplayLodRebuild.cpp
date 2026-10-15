/// @file BrepDisplayLodRebuild.cpp
/// @brief 按档重剖 B-rep 显示网格

#include "BrepDisplayLodRebuild.h"

#include "BackendDataBase.h"
#include "BrepBackendData.h"
#include "DocumentHost.h"
#include "visual/VisualAspect.h"

#include <memory>

#include <BrepImportArtifacts.h>
#include <Types.h>

namespace cloudsim::host
{
bool rebuildBrepDisplayLod(DocumentHost& host, const std::string& backendId, int lod, QString* outError)
{
	const auto obj = host.findObject(backendId);
	auto brep = std::dynamic_pointer_cast<BrepBackendData>(obj);
	if (!brep || !brep->hasGeometry())
	{
		if (outError)
		{
			*outError = QStringLiteral("Backend is not a B-rep with geometry.");
		}
		return false;
	}
	const geoalgo::BrepDisplayLod lodE = geoalgo::clampBrepDisplayLod(lod);
	brep->setDisplayLod(static_cast<int>(lodE));
	geoalgo::removeBrepImportArtifactsCacheForShape(brep->shapeRef());
	std::string artifactErr;
	geoalgo::BrepImportBuildTimings timings;
	const auto artifacts =
		geoalgo::getOrBuildBrepImportArtifacts(brep->shapeRef(), &artifactErr, &timings, lodE);
	if (!artifacts || !artifacts->hasDisplayData())
	{
		if (outError)
		{
			*outError = artifactErr.empty() ? QStringLiteral("B-rep re-tessellation failed.")
										   : QString::fromStdString(artifactErr);
		}
		return false;
	}
	(void)timings;
	host.markVisualDirty(backendId, VisualAspect::Geometry);
	(void)host.flushVisualSync();
	return true;
}

} // namespace cloudsim::host
