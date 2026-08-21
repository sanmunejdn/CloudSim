#ifndef WIDGET_SELECTIONOPERATIONTOOLADAPTER_H
#define WIDGET_SELECTIONOPERATIONTOOLADAPTER_H

/// @file SelectionOperationToolAdapter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 将既有 SelectionOperation 适配为 IPointerTool

#include "../IPointerTool.h"
#include "SelectionOperation.h"

#include <string>

#include "SelectionOperation.h"

class SelectionOperationToolAdapter final : public IPointerTool
{
public:
	SelectionOperationToolAdapter(const char* id, SelectionOperation* operation)
		: m_id(id ? id : ""), m_operation(operation)
	{
	}

	const char* toolId() const override { return m_id.c_str(); }

	bool handleEvent(QObject* watched, QEvent* event) override
	{
		return m_operation ? m_operation->handleEvent(watched, event) : false;
	}

private:
	std::string m_id;
	SelectionOperation* m_operation = nullptr;
};

#endif // WIDGET_SELECTIONOPERATIONTOOLADAPTER_H
