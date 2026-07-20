#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGEXERUNNER_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGEXERUNNER_H

/// @file MeshSurfaceReconstructionGmcgExeRunner.h
/// @brief MeshSurfaceReconstructionGmcgExeRunner 接口

#include "MeshSurfaceReconstructionAmrtoTypes.h"

namespace geoalgo
{
namespace meshrecon
{
struct GmcgExeParams
{
	std::string workDir;
	std::string exePath;
	std::string inputObjPath;
	std::string globalResultObjName = "result_gmcg.obj";
	int timeoutSeconds = 300;
};

bool resolveDefaultGmcgExePath(std::string& outPath, std::string* errMsg);

bool createGmcgWorkDirectories(const std::string& workDir, std::string* errMsg);

bool runGmcgExe(const GmcgExeParams& params, std::string* errMsg);

bool loadGmcgExeOutput(const std::string& workDir, const std::string& globalResultObjName, GmcgResult& outResult,
					   std::string* errMsg);

} // namespace meshrecon
} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTIONGMCGEXERUNNER_H
