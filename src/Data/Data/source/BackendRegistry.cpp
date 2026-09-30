/// @file BackendRegistry.cpp
/// @brief BackendRegistry 进程级 override 槽（跨 TU 单点）

#include "BackendRegistry.h"

#include <cassert>

BackendRegistry*& BackendRegistry::processInstanceSlot()
{
	static BackendRegistry* slot = nullptr;
	return slot;
}

BackendRegistry& BackendRegistry::instance()
{
	BackendRegistry* slot = processInstanceSlot();
	assert(slot != nullptr);
	return *slot;
}
