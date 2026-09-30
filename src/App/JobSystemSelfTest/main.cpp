/// @file main.cpp
/// @brief JobSystem lifecycle SelfTest 独立入口（避免 SelfTestRunner 链 Widget 依赖爆炸）

#include "JobSystem.h"

#include <QCoreApplication>
#include <cstdio>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

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
	prependPathIfExists(L"D:\\Qt\\Qt5.14.2\\5.14.2\\msvc2017_64\\bin");
	wchar_t qtDir[MAX_PATH];
	const DWORD qtLen = GetEnvironmentVariableW(L"QTDIR", qtDir, MAX_PATH);
	if (qtLen > 0 && qtLen < MAX_PATH)
		prependPathIfExists(std::wstring(qtDir, qtLen) + L"\\bin");
}
#endif
} // namespace

int main(int argc, char** argv)
{
#ifdef _WIN32
	configureDllSearchPath();
#endif
	QCoreApplication app(argc, argv);
	setvbuf(stdout, nullptr, _IONBF, 0);

	std::printf("[JobSystem] running...\n");
	std::vector<std::string> failures;
	const bool ok = runJobSystemLifecycleSelfTest(&failures);
	std::printf("[JobSystem] %s\n", ok ? "PASS" : "FAIL");
	for (const std::string& f : failures)
		std::printf("  - %s\n", f.c_str());
	return ok ? 0 : 1;
}
