#include "MeshSurfaceReconstructionGmcgExeRunner.h"
#include "MeshSurfaceReconstructionAmrtoLoader.h"

#include "RunLogger.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace geoalgo
{
namespace meshrecon
{
namespace
{

bool fileSizeAtLeast(const fs::path& p, const std::size_t minBytes)
{
	std::error_code ec;
	const auto sz = fs::file_size(p, ec);
	return !ec && sz >= minBytes;
}

} // namespace

bool resolveDefaultGmcgExePath(std::string& outPath, std::string* errMsg)
{
	const fs::path dir(defaultAmrtoToolsDirectory());
	const fs::path candidateA = dir / "Instant-Meshes and GMCG_revision";
	const fs::path candidateB = dir / "GMCG_revision.exe";
	if (fileSizeAtLeast(candidateA, 1024U))
	{
		outPath = candidateA.string();
		return true;
	}
	if (fileSizeAtLeast(candidateB, 1024U))
	{
		outPath = candidateB.string();
		return true;
	}
	if (errMsg)
	{
		*errMsg = "GMCG exe not found or placeholder in: " + dir.string();
	}
	return false;
}

bool createGmcgWorkDirectories(const std::string& workDir, std::string* errMsg)
{
	std::error_code ec;
	fs::create_directories(fs::path(workDir) / "output_quad", ec);
	fs::create_directories(fs::path(workDir) / "output_tri", ec);
	if (ec)
	{
		if (errMsg)
		{
			*errMsg = "create gmcg work dirs failed: " + ec.message();
		}
		return false;
	}
	return true;
}

bool runGmcgExe(const GmcgExeParams& params, std::string* errMsg)
{
	std::string exePath = params.exePath;
	if (exePath.empty() && !resolveDefaultGmcgExePath(exePath, errMsg))
	{
		return false;
	}
	if (!createGmcgWorkDirectories(params.workDir, errMsg))
	{
		return false;
	}
	if (params.inputObjPath.empty() || !fs::exists(params.inputObjPath))
	{
		if (errMsg)
		{
			*errMsg = "gmcg input obj missing";
		}
		return false;
	}

	const fs::path stagedInput = fs::path(params.workDir) / "input_quad.obj";
	std::error_code ec;
	fs::copy_file(params.inputObjPath, stagedInput, fs::copy_options::overwrite_existing, ec);
	if (ec)
	{
		if (errMsg)
		{
			*errMsg = "copy gmcg input failed: " + ec.message();
		}
		return false;
	}

	std::string cmd = "\"" + exePath + "\" \"" + stagedInput.string() + "\"";
	RunLogger::info(std::string("gmcg exe: ") + cmd);
	const int code = std::system(cmd.c_str());
	if (code != 0)
	{
		if (errMsg)
		{
			*errMsg = "gmcg exe exit code " + std::to_string(code);
		}
		return false;
	}
	return true;
}

bool loadGmcgExeOutput(
	const std::string& workDir,
	const std::string& globalResultObjName,
	GmcgResult& outResult,
	std::string* errMsg)
{
	const fs::path resultPath = fs::path(workDir) / globalResultObjName;
	if (fs::exists(resultPath))
	{
		return loadAmrtoGoldenDataset(workDir, globalResultObjName, outResult, errMsg);
	}
	outResult = {};
	return loadAmrtoChartsFromDirectory(workDir, outResult, errMsg);
}

} // namespace meshrecon
} // namespace geoalgo
