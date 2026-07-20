/// @file MeshRepair.cpp
/// @brief MeshRepair 实现

#include "MeshRepair.h"

#include "MeshRepairInternal.h"
#include "VcgMeshTypes.h"

namespace vcgalgo
{
bool repairMesh(const std::vector<float>& triangleSoup, std::vector<float>& outSoup, const RepairParams& params,
				RepairReport* report, std::string* errMsg)
{
	outSoup.clear();
	if (report != nullptr)
	{
		*report = RepairReport{};
	}

	VcgMesh mesh;
	if (!internal::soupToVcgMesh(triangleSoup, mesh, errMsg))
	{
		return false;
	}

	if (!internal::repairVcgMeshInPlace(mesh, params, report))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "repairMesh: no faces remain after repair";
		}
		return false;
	}

	internal::vcgMeshToSoup(mesh, outSoup);
	return !outSoup.empty();
}

} // namespace vcgalgo
