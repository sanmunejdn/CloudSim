/// @file InstantMeshesCore.cpp
/// @brief InstantMeshesCore 实现

#include "InstantMeshesCore.h"

#include "ImBatchBridge.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace fs = std::filesystem;

namespace instant_meshes
{
namespace
{
constexpr std::size_t kMinToolBytes = 1024U;

std::string readEnvVar(const char* name)
{
#if defined(_MSC_VER)
	char* env = nullptr;
	size_t envLen = 0;
	if (_dupenv_s(&env, &envLen, name) == 0 && env != nullptr)
	{
		std::string value(env);
		free(env);
		return value;
	}
#else
	if (const char* env = std::getenv(name))
	{
		return env;
	}
#endif
	return {};
}

std::string resolveSdkRoot()
{
	const std::string fromEnv = readEnvVar("CLOUDSIM_SDK");
	if (!fromEnv.empty())
	{
		return fromEnv;
	}
	return "D:\\Project\\VSprogram\\CGAL5.5.2\\bin\\SDK";
}

bool isRunnableTool(const fs::path& path)
{
	std::error_code ec;
	if (!fs::exists(path, ec) || ec)
	{
		return false;
	}
	const auto sz = fs::file_size(path, ec);
	return !ec && sz >= kMinToolBytes;
}

std::string quotePath(const std::string& path)
{
	return "\"" + path + "\"";
}

std::string resolveFromPathList(const std::vector<std::string>& exeNames)
{
	const std::string pathList = readEnvVar("PATH");
	if (pathList.empty())
	{
		return {};
	}
	std::istringstream stream(pathList);
	std::string dir;
	while (std::getline(stream, dir, ';'))
	{
		if (dir.empty())
		{
			continue;
		}
		for (const std::string& exeName : exeNames)
		{
			const fs::path candidate = fs::path(dir) / exeName;
			if (isRunnableTool(candidate))
			{
				return candidate.string();
			}
		}
	}
	return {};
}

std::string defaultAmrtoToolPath()
{
	return resolveSdkRoot() + "\\AMRTO\\Instant-Meshes and GMCG_revision\\Instant-Meshes and GMCG_revision";
}

std::vector<std::string> amrtoToolCandidates()
{
	const std::string root = resolveSdkRoot();
	return {
		root + "\\AMRTO\\Instant-Meshes and GMCG_revision\\Instant-Meshes and GMCG_revision",
		root + "\\AMRTO\\Instant-Meshes and GMCG_revision\\GMCG_revision.exe",
		root + "\\CODE_AMRTO\\Instant-Meshes and GMCG_revision\\Instant-Meshes and GMCG_revision",
	};
}

std::string resolveInstantMeshesExe(const std::string& overridePath)
{
	if (!overridePath.empty() && isRunnableTool(fs::path(overridePath)))
	{
		return overridePath;
	}
	const std::vector<std::string> envKeys = {
		"CLOUDSIM_INSTANT_MESHES_EXE",
		"CLOUDSIM_IM_EXE",
	};
	for (const std::string& key : envKeys)
	{
		const std::string custom = readEnvVar(key.c_str());
		if (!custom.empty() && isRunnableTool(fs::path(custom)))
		{
			return custom;
		}
	}

	const std::vector<std::string> sdkCandidates = {
		resolveSdkRoot() + "\\instant-meshes\\instant-meshes.exe",
		resolveSdkRoot() + "\\instant-meshes\\InstantMeshes.exe",
		resolveSdkRoot() + "\\Tools\\instant-meshes.exe",
	};
	for (const std::string& candidate : sdkCandidates)
	{
		if (isRunnableTool(fs::path(candidate)))
		{
			return candidate;
		}
	}

	const std::vector<std::string> pathNames = {
		"instant-meshes.exe",
		"instant-meshes",
		"InstantMeshes.exe",
	};
	return resolveFromPathList(pathNames);
}

std::string buildInstantMeshesCommand(const std::string& exePath, const std::string& inObj, const std::string& outObj,
									  const Params& params)
{
	std::ostringstream cmd;
	cmd << quotePath(exePath);
	if (params.deterministic)
	{
		cmd << " -d";
	}
	if (params.creaseAngleDeg > 0.f)
	{
		cmd << " -c " << params.creaseAngleDeg;
	}
	if (params.targetVertexCount > 0)
	{
		cmd << " -v " << params.targetVertexCount;
	}
	if (params.pureQuad)
	{
		cmd << " -p 4";
	}
	cmd << " -o " << quotePath(outObj) << ' ' << quotePath(inObj);
	return cmd.str();
}

bool runProcess(const std::string& cmd, std::string* errMsg)
{
	const int code = std::system(cmd.c_str());
	if (code != 0)
	{
		if (errMsg)
		{
			*errMsg = "process failed code=" + std::to_string(code) + " cmd=" + cmd;
		}
		return false;
	}
	return true;
}
bool parseQuadFaceLine(const std::string& line, std::vector<int>& face, std::vector<int>& faceVt)
{
	std::istringstream iss(line.substr(2));
	std::string tok;
	while (iss >> tok)
	{
		int vi = -1;
		int vti = -1;
		const std::size_t slash = tok.find('/');
		if (slash == std::string::npos)
		{
			vi = std::stoi(tok) - 1;
		}
		else
		{
			vi = std::stoi(tok.substr(0, slash)) - 1;
			if (slash + 1U < tok.size())
			{
				const std::size_t slash2 = tok.find('/', slash + 1U);
				const std::string vtPart = (slash2 == std::string::npos) ? tok.substr(slash + 1U)
																		 : tok.substr(slash + 1U, slash2 - slash - 1U);
				if (!vtPart.empty())
				{
					vti = std::stoi(vtPart) - 1;
				}
			}
		}
		face.push_back(vi);
		faceVt.push_back(vti);
	}
	return face.size() >= 4U;
}

} // namespace

bool writeTriMeshObj(const TriMesh& mesh, const std::string& objPath, std::string* errMsg)
{
	std::ofstream out(objPath);
	if (!out)
	{
		if (errMsg)
		{
			*errMsg = "cannot write obj: " + objPath;
		}
		return false;
	}
	const int vCount = static_cast<int>(mesh.vertices.size() / 3U);
	for (int vi = 0; vi < vCount; ++vi)
	{
		const std::size_t b = static_cast<std::size_t>(vi) * 3U;
		out << "v " << mesh.vertices[b] << ' ' << mesh.vertices[b + 1U] << ' ' << mesh.vertices[b + 2U] << '\n';
	}
	const int fCount = static_cast<int>(mesh.faces.size() / 3U);
	for (int fi = 0; fi < fCount; ++fi)
	{
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		out << "f " << (mesh.faces[b] + 1) << ' ' << (mesh.faces[b + 1U] + 1) << ' ' << (mesh.faces[b + 2U] + 1)
			<< '\n';
	}
	return true;
}

bool writeQuadMeshObj(const QuadMesh& quad, const std::string& objPath, std::string* errMsg)
{
	std::ofstream out(objPath);
	if (!out)
	{
		if (errMsg)
		{
			*errMsg = "cannot write quad obj: " + objPath;
		}
		return false;
	}
	const int vCount = static_cast<int>(quad.vertices.size() / 3U);
	for (int vi = 0; vi < vCount; ++vi)
	{
		const std::size_t b = static_cast<std::size_t>(vi) * 3U;
		out << "v " << quad.vertices[b] << ' ' << quad.vertices[b + 1U] << ' ' << quad.vertices[b + 2U] << '\n';
	}
	const int qCount = static_cast<int>(quad.quadFaces.size() / 4U);
	for (int qi = 0; qi < qCount; ++qi)
	{
		const std::size_t b = static_cast<std::size_t>(qi) * 4U;
		out << "f " << (quad.quadFaces[b] + 1) << ' ' << (quad.quadFaces[b + 1U] + 1) << ' '
			<< (quad.quadFaces[b + 2U] + 1) << ' ' << (quad.quadFaces[b + 3U] + 1) << '\n';
	}
	return true;
}

bool loadQuadMeshObj(const std::string& objPath, QuadMesh& outMesh, std::string* errMsg)
{
	outMesh = {};
	std::ifstream in(objPath);
	if (!in)
	{
		if (errMsg)
		{
			*errMsg = "cannot open obj: " + objPath;
		}
		return false;
	}
	std::vector<float> rawVerts;
	std::vector<float> rawUv;
	std::vector<std::vector<int>> rawFaces;
	std::vector<std::vector<int>> rawFaceVt;
	std::string line;
	while (std::getline(in, line))
	{
		if (line.size() < 2U)
		{
			continue;
		}
		if (line[0] == 'v' && line[1] == ' ')
		{
			std::istringstream iss(line.substr(2));
			float x = 0.f;
			float y = 0.f;
			float z = 0.f;
			iss >> x >> y >> z;
			rawVerts.push_back(x);
			rawVerts.push_back(y);
			rawVerts.push_back(z);
		}
		else if (line[0] == 'v' && line[1] == 't')
		{
			std::istringstream iss(line.substr(3));
			float u = 0.f;
			float v = 0.f;
			iss >> u >> v;
			rawUv.push_back(u);
			rawUv.push_back(v);
		}
		else if (line[0] == 'f')
		{
			std::vector<int> face;
			std::vector<int> faceVt;
			if (parseQuadFaceLine(line, face, faceVt))
			{
				rawFaces.push_back(std::move(face));
				rawFaceVt.push_back(std::move(faceVt));
			}
		}
	}
	if (rawVerts.empty() || rawFaces.empty())
	{
		if (errMsg)
		{
			*errMsg = "obj has no faces or vertices: " + objPath;
		}
		return false;
	}
	outMesh.vertices = std::move(rawVerts);
	outMesh.vertexUv = std::move(rawUv);
	for (const auto& face : rawFaces)
	{
		for (int vi : face)
		{
			outMesh.quadFaces.push_back(vi);
		}
	}
	(void)rawFaceVt;
	return !outMesh.quadFaces.empty();
}

bool remeshToQuadMesh(const TriMesh& triIn, QuadMesh& quadOut, const Params& params, std::string* errMsg)
{
	quadOut = {};
	if (triIn.vertices.empty() || triIn.faces.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty input mesh for instant meshes";
		}
		return false;
	}

	const fs::path tempDir = fs::temp_directory_path() / "CloudSim_im_work";
	fs::create_directories(tempDir);
	const std::string inObj = (tempDir / "input_tri.obj").string();
	const std::string outObj = (tempDir / "output_quad.obj").string();
	if (!writeTriMeshObj(triIn, inObj, errMsg))
	{
		return false;
	}

	std::string lastErr;
	const auto tryLoadOutput = [&]() -> bool { return loadQuadMeshObj(outObj, quadOut, &lastErr); };

#if defined(INSTANT_MESHES_HAS_LIB)
	lastErr.clear();
	if (remeshViaInProcessBatch(inObj, outObj, params, &lastErr) && tryLoadOutput())
	{
		return true;
	}
	if (lastErr.empty())
	{
		lastErr = "in-process instant meshes batch failed";
	}
#endif

	const auto tryRun = [&](const std::string& cmd) -> bool
	{
		lastErr.clear();
		if (!runProcess(cmd, &lastErr))
		{
			return false;
		}
		return tryLoadOutput();
	};

	for (const std::string& amrtoTool : amrtoToolCandidates())
	{
		if (!isRunnableTool(fs::path(amrtoTool)))
		{
			continue;
		}
		const std::string cmd = buildInstantMeshesCommand(amrtoTool, inObj, outObj, params);
		if (tryRun(cmd))
		{
			return true;
		}
	}
	if (lastErr.empty())
	{
		lastErr = "AMRTO IM tools missing or placeholder under bin/SDK/AMRTO and CODE_AMRTO";
	}

	const std::string imExe = resolveInstantMeshesExe(params.exePath);
	if (!imExe.empty())
	{
		const std::string cmd = buildInstantMeshesCommand(imExe, inObj, outObj, params);
		if (tryRun(cmd))
		{
			return true;
		}
	}
	else if (lastErr.find("instant-meshes") == std::string::npos)
	{
		lastErr += " | instant-meshes not found (PATH/SDK/CLOUDSIM_INSTANT_MESHES_EXE)";
	}

	if (errMsg)
	{
		*errMsg = lastErr.empty() ? "instant meshes remesh failed"
								  : lastErr + " | hint: install instant-meshes, set CLOUDSIM_INSTANT_MESHES_EXE, or "
											  "use gmcgBackend=GoldenLoader for CODE_AMRTO data";
	}
	return false;
}
} // namespace instant_meshes
