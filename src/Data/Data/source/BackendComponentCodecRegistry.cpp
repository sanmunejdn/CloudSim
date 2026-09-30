/// @file BackendComponentCodecRegistry.cpp
/// @brief CodecRegistry 进程级 override 槽（跨 TU 单点）

#include "BackendComponentCodecRegistry.h"

#include <cassert>

BackendComponentCodecRegistry*& BackendComponentCodecRegistry::processInstanceSlot()
{
	static BackendComponentCodecRegistry* slot = nullptr;
	return slot;
}

BackendComponentCodecRegistry& BackendComponentCodecRegistry::instance()
{
	BackendComponentCodecRegistry* slot = processInstanceSlot();
	assert(slot != nullptr);
	return *slot;
}
