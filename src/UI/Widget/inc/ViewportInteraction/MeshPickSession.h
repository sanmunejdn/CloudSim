#ifndef WIDGET_MESHPICKSESSION_H
#define WIDGET_MESHPICKSESSION_H

/// @file MeshPickSession.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Mesh 边/面拾取会话：Commit 回调给业务

#include "../IInteractionSession.h"

#include <functional>

class MeshPickSession final : public IInteractionSession
{
public:
	using CommitFn = std::function<void(const ViewportHit&)>;
	using CancelFn = std::function<void()>;

	explicit MeshPickSession(CommitFn onCommit, CancelFn onCancel = {})
		: m_onCommit(std::move(onCommit)), m_onCancel(std::move(onCancel))
	{
	}

	void onCommit(const ViewportHit& hit) override
	{
		if (m_onCommit)
		{
			m_onCommit(hit);
		}
	}

	void onCancel() override
	{
		if (m_onCancel)
		{
			m_onCancel();
		}
	}

private:
	CommitFn m_onCommit;
	CancelFn m_onCancel;
};

#endif // WIDGET_MESHPICKSESSION_H
