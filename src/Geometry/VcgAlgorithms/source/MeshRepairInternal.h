#pragma once

#include "MeshRepair.h"
#include "VcgMeshTypes.h"

namespace vcgalgo::internal
{

bool repairVcgMeshInPlace(VcgMesh& mesh, const RepairParams& params, RepairReport* report);

} // namespace vcgalgo::internal
