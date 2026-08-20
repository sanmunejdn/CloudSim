#ifndef PROCESSFLOWPLUGIN_SIM_DISPATCHPOLICIES_H
#define PROCESSFLOWPLUGIN_SIM_DISPATCHPOLICIES_H

/// @file DispatchPolicies.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief FIFO / SPT / LPT / EDD / CR

#include "IDispatchPolicy.h"

#include <memory>

class FifoPolicy final : public IDispatchPolicy
{
public:
	int select(const DispatchContext& ctx) const override;
};

class SptPolicy final : public IDispatchPolicy
{
public:
	int select(const DispatchContext& ctx) const override;
};

class LptPolicy final : public IDispatchPolicy
{
public:
	int select(const DispatchContext& ctx) const override;
};

class EddPolicy final : public IDispatchPolicy
{
public:
	int select(const DispatchContext& ctx) const override;
};

class CrPolicy final : public IDispatchPolicy
{
public:
	int select(const DispatchContext& ctx) const override;
};

std::unique_ptr<IDispatchPolicy> createDispatchPolicy(const QString& name);
QStringList allDispatchPolicyNames();

#endif
