#include "MeshSurfaceReconstruction.h"
#include "MeshSurfaceReconstruction/MeshSurfaceReconstructionInternal.h"

namespace geoalgo
{

bool reconstructBrepFromMeshSoup(
	const std::vector<float>& soup,
	const MeshSurfaceReconstructParams& params,
	ShapeHandle& outShape,
	MeshSurfaceReconstructReport& report,
	std::string* errMsg)
{
	report = MeshSurfaceReconstructReport{};
	outShape = ShapeHandle{};

	try
	{
		meshrecon::IndexedMeshLite mesh;
		if (!meshrecon::soupToIndexed(soup, mesh, errMsg))
		{
			return false;
		}

		std::vector<meshrecon::QuadPatch> patches;
		if (!meshrecon::partitionQuadDomains(mesh, params, patches, report.junctionCount, errMsg))
		{
			return false;
		}
		report.patchCount = static_cast<int>(patches.size());

		if (!meshrecon::samplePatchGrids(mesh, patches, params, errMsg))
		{
			return false;
		}
		if (!meshrecon::buildInitialBsplinePatches(patches, params, errMsg))
		{
			return false;
		}

		bool blendOk = false;
		if (!meshrecon::applyBoundaryC2Blend(patches, params, blendOk, errMsg))
		{
			return false;
		}
		report.c2BlendSucceeded = blendOk;

		if (!meshrecon::applyJunctionC2Blend(patches, report.junctionCount, params, errMsg))
		{
			return false;
		}

		if (!meshrecon::fairBsplinePatches(patches, params, report.globalFairingMetric, errMsg))
		{
			return false;
		}

		if (!meshrecon::assembleBrepShape(patches, outShape, errMsg))
		{
			return false;
		}
		if (!meshrecon::validateTessellationSanity(outShape, errMsg))
		{
			outShape = ShapeHandle{};
			return false;
		}

		report.maxDeviationMm = meshrecon::computeMaxDeviationMm(soup, patches);
		if (!meshrecon::validateOutputBbox(soup, outShape, 3.0, errMsg))
		{
			outShape = ShapeHandle{};
			return false;
		}
		return true;
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = ex.what();
		}
		return false;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "surface reconstruction OCCT/internal error";
		}
		return false;
	}
}

} // namespace geoalgo
