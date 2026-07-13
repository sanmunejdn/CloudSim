#pragma once

#include "MeshSurfaceReconstructionInternal.h"

#include <array>
#include <string>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{

enum class GmcgBackend : int
{
	GoldenLoader = 0,
	Exe = 1,
	Native = 2,
};

struct QuadMeshLite
{
	std::vector<float> vertices;
	std::vector<int> quadFaces;
	std::vector<float> vertexUv;
};

struct GmcgChart
{
	int chartId = -1;
	QuadMeshLite quadMesh;
	std::vector<float> vertexUv;
	std::array<int, 4> cornerVertexIndices = {-1, -1, -1, -1};
};

struct GmcgResult
{
	QuadMeshLite globalQuad;
	std::vector<GmcgChart> charts;
	std::string globalResultObjPath;
};

struct InstantMeshesParams
{
	int targetVertexCount = 0;
	float creaseAngleDeg = 30.f;
	bool deterministic = true;
	bool pureQuad = true;
	std::string exePath;
};

std::string resolveCloudSimSdkRoot();

std::string defaultAmrtoToolsDirectory();
std::string defaultAmrtoGoldenDataDirectory();

} // namespace meshrecon
} // namespace geoalgo
