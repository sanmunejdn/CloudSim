#ifndef CLOUDSIM_CRASH_DUMP_H
#define CLOUDSIM_CRASH_DUMP_H

/// @file CrashDump.h
/// @brief 未处理异常写 MiniDump 到 {exe}/crash/

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

#include <DbgHelp.h>

#include <cstdio>
#include <string>

#pragma comment(lib, "dbghelp.lib")

namespace cloudsim
{
namespace crashdump
{
inline std::wstring exeDirW()
{
	wchar_t path[MAX_PATH];
	const DWORD n = GetModuleFileNameW(nullptr, path, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return L".";
	std::wstring dir(path, n);
	const size_t slash = dir.find_last_of(L"\\/");
	if (slash != std::wstring::npos)
		dir.resize(slash);
	return dir;
}

inline LONG WINAPI unhandledFilter(EXCEPTION_POINTERS* info)
{
	const std::wstring dir = exeDirW() + L"\\crash";
	CreateDirectoryW(dir.c_str(), nullptr);

	SYSTEMTIME st{};
	GetSystemTime(&st);
	wchar_t file[MAX_PATH];
	_snwprintf_s(file, _TRUNCATE, L"%s\\%04u%02u%02uT%02u%02u%02uZ_%lu.dmp", dir.c_str(), st.wYear, st.wMonth, st.wDay,
				 st.wHour, st.wMinute, st.wSecond, GetCurrentProcessId());

	HANDLE hFile = CreateFileW(file, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile != INVALID_HANDLE_VALUE)
	{
		MINIDUMP_EXCEPTION_INFORMATION mei{};
		mei.ThreadId = GetCurrentThreadId();
		mei.ExceptionPointers = info;
		mei.ClientPointers = FALSE;
		MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), hFile, MiniDumpWithDataSegs, &mei, nullptr,
						  nullptr);
		CloseHandle(hFile);
		fwprintf(stderr, L"CloudSim crash dump: %s\n", file);
	}
	return EXCEPTION_EXECUTE_HANDLER;
}

inline void install()
{
	SetUnhandledExceptionFilter(unhandledFilter);
}
} // namespace crashdump
} // namespace cloudsim

#else

namespace cloudsim
{
namespace crashdump
{
inline void install() {}
}
} // namespace cloudsim

#endif

#endif // CLOUDSIM_CRASH_DUMP_H
