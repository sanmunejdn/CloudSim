#ifndef DATA_GEOMETRYMUTATOR_H
#define DATA_GEOMETRYMUTATOR_H

/// @file GeometryMutator.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 几何变更 RAII 守卫：作用域退出自动 bump geometryRevision，防 setter 漏调

#include "data_global.h"

#include <functional>
#include <utility>

/// 几何变更守卫：析构时自动执行提交动作（bumpGeometryRevision）。
/// bumpGeometryRevision 是 BackendDataBase 的 protected 成员，仅派生类成员函数内可及，
/// 故提交动作由调用点以 lambda 给出：GeometryMutator guard([this] { bumpGeometryRevision(); });
/// 校验失败「保持原状」的提前返回路径须调 dismiss() 取消提交
class DATA_EXPORT GeometryMutator
{
public:
	explicit GeometryMutator(std::function<void()> commit)
		: m_commit(std::move(commit))
	{
	}
	~GeometryMutator()
	{
		if (!m_dismissed && m_commit)
		{
			m_commit();
		}
	}

	GeometryMutator(const GeometryMutator&) = delete;
	GeometryMutator& operator=(const GeometryMutator&) = delete;

	/// 取消提交：校验失败、保持原状的提前返回路径调用
	void dismiss() { m_dismissed = true; }

private:
	std::function<void()> m_commit;
	bool m_dismissed = false;
};

#endif // DATA_GEOMETRYMUTATOR_H
