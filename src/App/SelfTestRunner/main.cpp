/// @file main.cpp
/// @brief 聚合各算法 DLL runSelfTest；退出码 = 失败模块数
/// @note 启动前须 PATH 含 Qt/OSG（见 scripts/run_selftest.ps1）；GeometryAlgorithm 全量自检可能数分钟

#include "MeshBoolean.h"
#include "SelfTest.h"
#include "geometry_algorithm_global.h"
#include "geometry_engine_global.h"
#include "robot_path_planning_global.h"
#include "robot_urdf_global.h"
#include "vcg_algorithms_global.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace UrdfRobotLoader
{
ROBOT_URDF_API bool runSelfTest(std::vector<std::string>& failures);
}
namespace engine
{
GEOMETRY_ENGINE_API bool runSelfTest(std::vector<std::string>& failures);
}
namespace geoalgo
{
GEOMETRY_ALGORITHM_API bool runSelfTest(std::vector<std::string>& failures);
}
namespace vcgalgo
{
VCg_ALGORITHMS_API bool runSelfTest(std::vector<std::string>& failures);
}
namespace robot_path
{
ROBOT_PATH_PLANNING_API bool runSelfTest(std::vector<std::string>& failures);
}

namespace
{
#ifdef _WIN32
void prependPathIfExists(const std::wstring& dir)
{
	if (dir.empty() || GetFileAttributesW(dir.c_str()) == INVALID_FILE_ATTRIBUTES)
		return;
	wchar_t buf[32768];
	const DWORD n = GetEnvironmentVariableW(L"PATH", buf, 32768);
	std::wstring path = dir;
	if (n > 0 && n < 32768)
	{
		path.push_back(L';');
		path.append(buf, n);
	}
	SetEnvironmentVariableW(L"PATH", path.c_str());
}

void configureDllSearchPath()
{
	wchar_t exePath[MAX_PATH];
	const DWORD len = GetModuleFileNameW(nullptr, exePath, MAX_PATH);
	if (len == 0 || len >= MAX_PATH)
		return;
	std::wstring exeDir(exePath, len);
	const size_t slash = exeDir.find_last_of(L"\\/");
	if (slash != std::wstring::npos)
		exeDir.resize(slash);
	prependPathIfExists(exeDir);
	prependPathIfExists(exeDir + L"\\..\\SDK\\OSG3.6.5\\bin");
	prependPathIfExists(exeDir + L"\\..\\..\\SDK\\OSG3.6.5\\bin");
	prependPathIfExists(exeDir + L"\\OSG3.6.5\\bin");
	prependPathIfExists(L"D:\\Qt\\Qt5.14.2\\5.14.2\\msvc2017_64\\bin");
	wchar_t qtDir[MAX_PATH];
	const DWORD qtLen = GetEnvironmentVariableW(L"QTDIR", qtDir, MAX_PATH);
	if (qtLen > 0 && qtLen < MAX_PATH)
		prependPathIfExists(std::wstring(qtDir, qtLen) + L"\\bin");
}
#endif

using Failures = std::vector<std::string>;

bool runModule(const char* name, bool (*fn)(Failures&), int& failedModules)
{
	std::printf("[%s] running...\n", name);
	Failures failures;
	const bool ok = fn(failures);
	std::printf("[%s] %s\n", name, ok ? "PASS" : "FAIL");
	for (const std::string& f : failures)
		std::printf("  - %s\n", f.c_str());
	if (!ok)
		++failedModules;
	return ok;
}

bool runMeshBoolean(Failures& failures)
{
	failures.clear();
	std::string err;
	if (MeshBoolean::runSelfTest(&err))
		return true;
	failures.push_back(err.empty() ? "MeshBoolean::runSelfTest failed" : err);
	return false;
}

bool wantModule(const std::string& filter, const char* name)
{
	if (filter.empty() || filter == "all" || filter == "full")
		return true;
	if (filter == "gate")
	{
		// 跳过：GeometryAlgorithm/PointCloud 极慢；VcgAlgorithms Debug 会触发第三方 assert 中止
		return std::strcmp(name, "GeometryAlgorithm") != 0 && std::strcmp(name, "PointCloudAlgorithm") != 0 &&
			   std::strcmp(name, "VcgAlgorithms") != 0;
	}
	return filter.find(name) != std::string::npos;
}

std::string parseFilter(int argc, char** argv)
{
	for (int i = 1; i < argc; ++i)
	{
		if (std::strncmp(argv[i], "--preset=", 9) == 0)
			return argv[i] + 9;
		if (std::strcmp(argv[i], "--gate") == 0)
			return "gate";
		if (std::strcmp(argv[i], "--full") == 0)
			return "full";
	}
#ifdef _WIN32
	char env[256];
	const DWORD n = GetEnvironmentVariableA("CLOUDSIM_SELFTEST_PRESET", env, 256);
	if (n > 0 && n < 256)
		return std::string(env, n);
#endif
	return "gate";
}
} // namespace

int main(int argc, char** argv)
{
#ifdef _WIN32
	configureDllSearchPath();
#endif
	setvbuf(stdout, nullptr, _IONBF, 0);

	const std::string filter = parseFilter(argc, argv);
	std::printf("SelfTestRunner preset=%s\n", filter.c_str());

	int failedModules = 0;
	if (wantModule(filter, "RobotUrdf"))
		runModule("RobotUrdf", &UrdfRobotLoader::runSelfTest, failedModules);
	if (wantModule(filter, "GeometryEngine"))
		runModule("GeometryEngine", &engine::runSelfTest, failedModules);
	if (wantModule(filter, "GeometryAlgorithm"))
		runModule("GeometryAlgorithm", &geoalgo::runSelfTest, failedModules);
	if (wantModule(filter, "PointCloudAlgorithm"))
		runModule("PointCloudAlgorithm", &GeometryServicesSelfTest::runPointCloudSelfTest, failedModules);
	if (wantModule(filter, "VcgAlgorithms"))
		runModule("VcgAlgorithms", &vcgalgo::runSelfTest, failedModules);
	if (wantModule(filter, "RobotPathPlanning"))
		runModule("RobotPathPlanning", &robot_path::runSelfTest, failedModules);
	if (wantModule(filter, "GeometryServices.MeshBoolean") || wantModule(filter, "MeshBoolean"))
		runModule("GeometryServices.MeshBoolean", &runMeshBoolean, failedModules);

	std::printf("SelfTestRunner done: failedModules=%d\n", failedModules);
	return failedModules;
}
