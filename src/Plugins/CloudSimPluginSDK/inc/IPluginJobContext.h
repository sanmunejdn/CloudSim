#ifndef CLOUDSIMPLUGINSDK_IPLUGINJOBCONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINJOBCONTEXT_H

/// @file IPluginJobContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后台任务窄接口（IPluginHostContext 按域拆分）

/// vtable 仅末尾追加，勿插入中间以免破坏旧插件 ABI

#include "cloudsim_plugin_sdk_global.h"

#include <QString>
#include <functional>
#include <utility>

using PluginJobProgressFn = std::function<void(double fraction, const QString& message)>;

/// 协作取消；长循环须查 canceled()，不会强杀线程
class PluginJobCancelToken
{
public:
	PluginJobCancelToken() = default;
	explicit PluginJobCancelToken(std::function<bool()> check) : m_check(std::move(check)) {}

	bool canceled() const { return m_check && m_check(); }

private:
	std::function<bool()> m_check;
};

using PluginCancellableJobWorkFn = std::function<void(const PluginJobProgressFn&, const PluginJobCancelToken&)>;

/// 后台任务上下文：UI 线程投递、JobSystem 普通/可取消任务
class IPluginJobContext
{
public:
	virtual ~IPluginJobContext() = default;

	/// 接口版本（0x00010000 = 1.0.0）
	virtual unsigned int contextVersion() const = 0;

	/// 投递 fn 到 UI 线程（已在则直跑）
	virtual void invokeOnUiThread(std::function<void()> fn) = 0;

	/// JobSystem 后台任务；onFinished 回 UI 线程
	virtual void enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
							std::function<void(bool threw, const QString& throwMessage)> onFinished) = 0;

	/// 可取消后台任务；onFinished 仍回 UI（Canceled 时 threw=false）
	virtual quint64 enqueueCancellableJob(const QString& title, PluginCancellableJobWorkFn work,
										  std::function<void(bool threw, const QString& throwMessage)> onFinished) = 0;
	virtual bool cancelJob(quint64 jobId) = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINJOBCONTEXT_H
