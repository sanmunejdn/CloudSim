/// @file BackendComponentCodecRegistry.cpp
/// @brief CodecRegistry 进程级 override 槽（跨 TU 单点）

#include "BackendComponentCodecRegistry.h"

BackendComponentCodecRegistry*& BackendComponentCodecRegistry::processInstanceSlot()
{
	static BackendComponentCodecRegistry* slot = nullptr;
	return slot;
}
