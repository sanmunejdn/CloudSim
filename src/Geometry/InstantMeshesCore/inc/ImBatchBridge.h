#pragma once

#include "InstantMeshesCore.h"

#include <string>

namespace instant_meshes
{

#if defined(INSTANT_MESHES_HAS_LIB)
bool remeshViaInProcessBatch(
	const std::string& inObj,
	const std::string& outObj,
	const Params& params,
	std::string* errMsg);
#endif

} // namespace instant_meshes
