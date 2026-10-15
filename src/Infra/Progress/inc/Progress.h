#ifndef PROGRESS_PROGRESS_H
#define PROGRESS_PROGRESS_H

/// @file Progress.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 横切进度总线：无 Qt，任意模块可 begin/report/end；UI 经 setUiSink 注入

#include "progress_global.h"

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

namespace Progress
{
enum class Kind
{
	Started = 0,
	Update,
	Finished
};

struct Event
{
	Kind kind = Kind::Started;
	std::uint64_t id = 0;
	std::string title;
	/// Update：∈[0,1]；<0 表示不确定进度
	double fraction = 0.0;
	std::string message;
	bool success = true;
	std::string error;
	bool cancellable = false;
};

using UiSink = std::function<void(const Event&)>;

PROGRESS_API void initialize();
PROGRESS_API void shutdown();
PROGRESS_API void setUiSink(UiSink sink);
PROGRESS_API void clearUiSink();

PROGRESS_API std::uint64_t begin(std::string title, bool cancellable = false);
/// JobSystem 等已有 id 的任务：与 begin 同事件，不分配新 id
PROGRESS_API void announceStarted(std::uint64_t id, std::string title, bool cancellable = false);
PROGRESS_API void report(std::uint64_t id, double fraction01, std::string_view message);
PROGRESS_API void end(std::uint64_t id, bool ok, std::string_view error = {});

/// RAII：析构时自动 end(ok)
class PROGRESS_API Scope
{
public:
	explicit Scope(std::string title, bool cancellable = false);
	~Scope();

	Scope(const Scope&) = delete;
	Scope& operator=(const Scope&) = delete;

	std::uint64_t id() const { return m_id; }
	void report(double fraction01, std::string_view message);
	void setSuccess(bool ok, std::string_view error = {});

private:
	std::uint64_t m_id = 0;
	bool m_ended = false;
	bool m_ok = true;
	std::string m_error;
};
} // namespace Progress

#endif // PROGRESS_PROGRESS_H
