/// @file PythonScriptCaller.cpp
/// @brief 嵌入式 Python 调用实现

#include "PythonScriptCaller.h"

#include <filesystem>
#include <iostream>
#include <string>

#include <Windows.h>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>

namespace fs = std::filesystem;
namespace py = pybind11;

namespace RobotWidget
{
namespace
{

fs::path resolvePythonHome()
{
	wchar_t exePath[MAX_PATH] = {};
	if (GetModuleFileNameW(nullptr, exePath, MAX_PATH) == 0)
	{
		return {};
	}
	const fs::path exeDir = fs::path(exePath).parent_path();
	// OutDir 为 bin/x64 或 bin/x64d，SDK 在 bin/SDK/python311
	const fs::path candidates[] = {
		exeDir / L".." / L"SDK" / L"python311",
		exeDir / L"SDK" / L"python311",
		exeDir / L"python311",
	};
	for (const fs::path& c : candidates)
	{
		std::error_code ec;
		const fs::path canon = fs::weakly_canonical(c, ec);
		if (!ec && fs::exists(canon / L"python311.dll"))
		{
			return canon;
		}
	}
	return {};
}

} // namespace

PythonScriptCaller& PythonScriptCaller::instance()
{
	static PythonScriptCaller caller;
	return caller;
}

PythonScriptCaller::PythonScriptCaller()
{
	try
	{
		if (Py_IsInitialized())
		{
			m_ok = true;
			return;
		}

		const fs::path pythonHome = resolvePythonHome();
		if (pythonHome.empty())
		{
			m_initError = "未找到 SDK python311（相对可执行目录 ../SDK/python311）";
			return;
		}

		static std::wstring pythonHomeStr = pythonHome.wstring();

		PyConfig config;
		PyConfig_InitPythonConfig(&config);
		PyStatus status = PyConfig_SetString(&config, &config.home, pythonHomeStr.c_str());
		if (PyStatus_Exception(status))
		{
			PyConfig_Clear(&config);
			m_initError = "PyConfig_SetString(home) 失败";
			return;
		}
		status = Py_InitializeFromConfig(&config);
		PyConfig_Clear(&config);
		if (PyStatus_Exception(status))
		{
			m_initError = "Py_InitializeFromConfig 失败";
			return;
		}

		const fs::path dllBin = pythonHome / L"Library" / L"bin";
		if (fs::exists(dllBin))
		{
			AddDllDirectory(dllBin.wstring().c_str());
		}
		AddDllDirectory(pythonHome.wstring().c_str());

		// 释放 GIL，后续 callPython 再 acquire
		PyEval_SaveThread();
		m_ok = true;
	}
	catch (const std::exception& e)
	{
		m_initError = std::string("Python 初始化异常: ") + e.what();
		m_ok = false;
	}
}

PythonScriptCaller::~PythonScriptCaller() = default;

bool PythonScriptCaller::isReady(std::string* outError) const
{
	if (!m_ok && outError)
	{
		*outError = m_initError.empty() ? "Python 未就绪" : m_initError;
	}
	return m_ok;
}

std::string PythonScriptCaller::callPython(const std::string& scriptPath, const std::string& funcName,
										   const std::string& jsonParams, std::string* outError)
{
	if (!isReady(outError))
	{
		return {};
	}

	py::gil_scoped_acquire acquire;
	std::string retValue;
	try
	{
		fs::path path(scriptPath);
		if (path.extension() != ".py")
		{
			throw std::runtime_error("脚本须为 .py");
		}
		if (!fs::exists(path))
		{
			throw std::runtime_error("脚本不存在: " + path.string());
		}
		if (path.is_relative())
		{
			path = fs::absolute(path);
		}
		path = fs::weakly_canonical(path);

		const std::string dir = path.parent_path().string();
		const std::string stem = path.stem().string();

		py::module_ sys = py::module_::import("sys");
		py::list sysPath = sys.attr("path");
		bool found = false;
		for (auto item : sysPath)
		{
			if (item.cast<std::string>() == dir)
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			sysPath.attr("append")(py::cast(dir));
		}

		py::module_ module = py::module_::import(stem.c_str());
		// 万级点导出时 reload 无收益且拖慢首次 import 后的热路径

		if (!py::hasattr(module, funcName.c_str()))
		{
			throw std::runtime_error("未找到函数: " + funcName);
		}
		py::object func = module.attr(funcName.c_str());
		if (!PyCallable_Check(func.ptr()))
		{
			throw std::runtime_error("对象不可调用: " + funcName);
		}

		py::module_ pyJson = py::module_::import("json");
		py::object pyParams = pyJson.attr("loads")(jsonParams);
		py::object result;
		if (py::isinstance<py::dict>(pyParams))
		{
			result = func(**pyParams.cast<py::dict>());
		}
		else
		{
			result = func(pyParams);
		}

		if (!result.is_none())
		{
			try
			{
				retValue = result.cast<std::string>();
			}
			catch (const py::cast_error&)
			{
				retValue = pyJson.attr("dumps")(result).cast<std::string>();
			}
		}
	}
	catch (const py::error_already_set& e)
	{
		if (outError)
		{
			*outError = std::string("Python 错误: ") + e.what();
		}
		if (PyErr_Occurred())
		{
			PyErr_Clear();
		}
		return {};
	}
	catch (const std::exception& e)
	{
		if (outError)
		{
			*outError = e.what();
		}
		if (PyErr_Occurred())
		{
			PyErr_Clear();
		}
		return {};
	}

	if (PyErr_Occurred())
	{
		PyErr_Clear();
	}
	return retValue;
}

} // namespace RobotWidget
