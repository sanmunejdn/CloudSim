#ifndef VCGALGORITHMS_MESHREPAIRINTERNAL_H
#define VCGALGORITHMS_MESHREPAIRINTERNAL_H

/// @file MeshRepairInternal.h
/// @brief MeshRepairInternal 接口

#include "MeshRepair.h"
#include "VcgMeshTypes.h"

namespace vcgalgo::internal
{
bool repairVcgMeshInPlace(VcgMesh& mesh, const RepairParams& params, RepairReport* report);

} // namespace vcgalgo::internal

#endif // VCGALGORITHMS_MESHREPAIRINTERNAL_H
