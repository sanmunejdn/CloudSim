/// @file MeshSurfaceReconstructionAmrtoPartition.cpp
/// @brief MeshSurfaceReconstructionAmrtoPartition 实现

#include "MeshSurfaceReconstructionAmrtoPartition.h"

#include "MeshSurfaceReconstructionAmrtoLoader.h"
#include "MeshSurfaceReconstructionGmcgExeRunner.h"
#include "MeshSurfaceReconstructionGmcgNative.h"
#include "MeshSurfaceReconstructionInstantMeshes.h"
#include "MeshSurfaceReconstructionPartitionCommon.h"
#include "RunLogger.h"

#include <filesystem>

namespace fs = std::filesystem;

namespace geoalgo
{
namespace meshrecon
{
bool partitionQuadDomainsAmrtoImGmcg(const IndexedMeshLite& mesh, const MeshSurfaceReconstructParams& params,
									 std::vector<QuadPatch>& patches, int& outJunctionCount,
									 MeshSurfaceReconstructReport* partitionStats, std::string* errMsg,
									 QuadMeshLite* outCachedQuadMesh, GmcgResult* outCachedGmcg)
{
	patches.clear();
	outJunctionCount = 0;
	if (outCachedQuadMesh)
	{
		*outCachedQuadMesh = {};
	}
	if (outCachedGmcg)
	{
		*outCachedGmcg = {};
	}

	const GmcgBackend backend = static_cast<GmcgBackend>(static_cast<int>(params.gmcgBackend));
	if (backend == GmcgBackend::GoldenLoader)
	{
		const std::string root =
			params.amrtoGoldenDataPath.empty() ? defaultAmrtoGoldenDataDirectory() : params.amrtoGoldenDataPath;
		const std::string resultName =
			params.amrtoGoldenResultObj.empty() ? "smooth_060.obj" : params.amrtoGoldenResultObj;
		if (!partitionFromGoldenLoader(mesh, root, resultName, patches, outJunctionCount, errMsg))
		{
			return false;
		}
		if (outCachedGmcg)
		{
			loadAmrtoGoldenDataset(root, resultName, *outCachedGmcg, nullptr);
		}
	}
	else
	{
		QuadMeshLite quadMesh;
		InstantMeshesParams imParams;
		imParams.targetVertexCount = params.instantMeshesTargetQuads;
		imParams.creaseAngleDeg = static_cast<float>(params.instantMeshesCreaseAngleDeg);
		imParams.deterministic = true;
		imParams.exePath = params.instantMeshesExePath;
		if (!remeshToQuadMesh(mesh, quadMesh, imParams, errMsg))
		{
			const std::string goldenRoot =
				params.amrtoGoldenDataPath.empty() ? defaultAmrtoGoldenDataDirectory() : params.amrtoGoldenDataPath;
			const std::string resultName =
				params.amrtoGoldenResultObj.empty() ? "smooth_060.obj" : params.amrtoGoldenResultObj;
			const fs::path goldenQuadDir = fs::path(goldenRoot) / "output_quad";
			// 有预分 chart 时直接走金标准分块，避免对全局 quad 跑 Native GMCG 合成一整片
			if (params.amrtoFallbackGoldenOnImFailure && fs::is_directory(goldenQuadDir))
			{
				std::string compatErr;
				if (!isGoldenDatasetMeshCompatible(mesh, goldenRoot, resultName, &compatErr))
				{
					if (errMsg)
					{
						*errMsg =
							compatErr.empty()
								? "instant meshes unavailable and input mesh does not match CODE_AMRTO golden data"
								: compatErr + " | install instant-meshes (CLOUDSIM_INSTANT_MESHES_EXE) for this model";
					}
					return false;
				}
				RunLogger::warn(std::string("instant meshes unavailable; falling back to golden partition loader: ") +
								goldenRoot);
				if (!partitionFromGoldenLoader(mesh, goldenRoot, resultName, patches, outJunctionCount, errMsg))
				{
					return false;
				}
				if (outCachedGmcg)
				{
					loadAmrtoGoldenDataset(goldenRoot, resultName, *outCachedGmcg, nullptr);
				}
				if (partitionStats)
				{
					int pMin = 0;
					int pMax = 0;
					int pSmall = 0;
					computePatchFaceStats(patches, 1, pMin, pMax, pSmall);
					partitionStats->minFacesPerPatch = pMin;
					partitionStats->maxFacesPerPatch = pMax;
					partitionStats->smallPatchCount = pSmall;
					partitionStats->quadPatchCount = static_cast<int>(patches.size());
				}
				return !patches.empty();
			}
			return false;
		}
		if (outCachedQuadMesh)
		{
			*outCachedQuadMesh = quadMesh;
		}

		GmcgResult gmcg;
		if (backend == GmcgBackend::Native)
		{
			if (!partitionQuadMeshNativeGmcg(quadMesh, gmcg, errMsg))
			{
				return false;
			}
		}
		else
		{
			const fs::path workDir = params.gmcgWorkDir.empty() ? (fs::temp_directory_path() / "CloudSim_gmcg_work")
																: fs::path(params.gmcgWorkDir);
			fs::create_directories(workDir);
			const std::string quadObj = (workDir / "input_quad.obj").string();
			if (!writeQuadMeshObj(quadMesh, quadObj, errMsg))
			{
				return false;
			}
			GmcgExeParams exeParams;
			exeParams.workDir = workDir.string();
			exeParams.exePath = params.gmcgExePath;
			exeParams.inputObjPath = quadObj;
			exeParams.globalResultObjName =
				params.amrtoGoldenResultObj.empty() ? "result_gmcg.obj" : params.amrtoGoldenResultObj;
			if (!runGmcgExe(exeParams, errMsg))
			{
				RunLogger::warn("gmcg exe failed, fallback to native gmcg");
				if (!partitionQuadMeshNativeGmcg(quadMesh, gmcg, errMsg))
				{
					return false;
				}
			}
			else if (!loadGmcgExeOutput(exeParams.workDir, exeParams.globalResultObjName, gmcg, errMsg))
			{
				RunLogger::warn("gmcg exe output load failed, fallback to native gmcg");
				if (!partitionQuadMeshNativeGmcg(quadMesh, gmcg, errMsg))
				{
					return false;
				}
			}
		}

		if (outCachedGmcg)
		{
			*outCachedGmcg = gmcg;
		}

		if (!gmcgResultToQuadPatches(mesh, gmcg, patches, errMsg))
		{
			return false;
		}
		const int faceCount = static_cast<int>(mesh.faces.size() / 3U);
		rebuildPatchAdjacency(buildMeshAdjacency(mesh, faceCount).fullAdj, faceCount, patches);
		outJunctionCount = computeJunctionCount(patches);
	}

	if (partitionStats)
	{
		int pMin = 0;
		int pMax = 0;
		int pSmall = 0;
		computePatchFaceStats(patches, 1, pMin, pMax, pSmall);
		partitionStats->minFacesPerPatch = pMin;
		partitionStats->maxFacesPerPatch = pMax;
		partitionStats->smallPatchCount = pSmall;
		partitionStats->quadPatchCount = static_cast<int>(patches.size());
	}
	return !patches.empty();
}

} // namespace meshrecon
} // namespace geoalgo
