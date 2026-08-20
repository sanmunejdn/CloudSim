#ifndef ROBOTWIDGET_PYTHONSCRIPTCALLER_H
#define ROBOTWIDGET_PYTHONSCRIPTCALLER_H

/// @file PythonScriptCaller.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 嵌入式 Python（pybind）调用品牌导出脚本

#include "robotwidget_global.h"

#include <string>

namespace RobotWidget
{

/// 进程级单例：初始化 SDK python311，按脚本路径调用入口函数
class ROBOTWIDGET_EXPORT PythonScriptCaller
{
public:
	static PythonScriptCaller& instance();

	/// @param scriptPath .py 绝对或相对路径
	/// @param funcName 入口名，如 ExportScript
	/// @param jsonParams JSON 对象字符串，键与 Python 关键字参数对齐
	/// @param outError 失败原因（可空）
	/// @return 约定字符串结果；失败返回空并写 outError
	std::string callPython(const std::string& scriptPath, const std::string& funcName,
						   const std::string& jsonParams = "{}", std::string* outError = nullptr);

	bool isReady(std::string* outError = nullptr) const;

private:
	PythonScriptCaller();
	~PythonScriptCaller();
	PythonScriptCaller(const PythonScriptCaller&) = delete;
	PythonScriptCaller& operator=(const PythonScriptCaller&) = delete;

	bool m_ok = false;
	std::string m_initError;
};

} // namespace RobotWidget

#endif
