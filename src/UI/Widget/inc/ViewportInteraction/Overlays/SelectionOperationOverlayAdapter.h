#ifndef WIDGET_SELECTIONOPERATIONOVERLAYADAPTER_H
#define WIDGET_SELECTIONOPERATIONOVERLAYADAPTER_H

/// @file SelectionOperationOverlayAdapter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 将既有 SelectionOperation 适配为 IOverlayOp

#include "../IOverlayOp.h"
#include "SelectionOperation.h"

#include <string>

#include "SelectionOperation.h"

class SelectionOperationOverlayAdapter final : public IOverlayOp
{
public:
	SelectionOperationOverlayAdapter(const char* id, SelectionOperation* operation)
		: m_id(id ? id : ""), m_operation(operation)
	{
	}

	const char* overlayId() const override { return m_id.c_str(); }

	bool handleEvent(QObject* watched, QEvent* event) override
	{
		return m_operation ? m_operation->handleEvent(watched, event) : false;
	}

private:
	std::string m_id;
	SelectionOperation* m_operation = nullptr;
};

#endif // WIDGET_SELECTIONOPERATIONOVERLAYADAPTER_H
