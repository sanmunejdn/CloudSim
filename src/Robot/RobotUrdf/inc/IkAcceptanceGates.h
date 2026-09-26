#ifndef ROBOTURDF_IKACCEPTANCEGATES_H
#define ROBOTURDF_IKACCEPTANCEGATES_H

/// @file IkAcceptanceGates.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Core / 指令 / Widget 共用的 IK 残差门限，避免跨层语义漂移

#include "robot_urdf_global.h"

namespace UrdfRobotLoader
{
/// 对外验收门限（与内核 SoftAccepted 的 2° 软容差分离：后者仅分类收敛状态）
struct IkAcceptanceGates
{
	static constexpr double kHardPosMm = 1.0;
	static constexpr double kHardOrientDeg = 0.5;
	static constexpr double kSoftPosMm = 3.0;
	static constexpr double kSoftOrientDeg = 5.0;
	static constexpr double kTaughtReusePosMm = 1.0;
	static constexpr double kTaughtReuseOrientDeg = 5.0;

	static constexpr double acceptPosMm(const bool allowApprox)
	{
		return allowApprox ? kSoftPosMm : kHardPosMm;
	}

	static constexpr double acceptOrientDeg(const bool allowApprox)
	{
		return allowApprox ? kSoftOrientDeg : kHardOrientDeg;
	}

	/// orientDeg < 0：未测姿态，仅卡门限位置
	static inline bool isFreshAcceptable(const double posMm, const double orientDeg, const bool allowApprox)
	{
		if (posMm < 0.0 || posMm > acceptPosMm(allowApprox))
		{
			return false;
		}
		if (orientDeg < 0.0)
		{
			return true;
		}
		return orientDeg <= acceptOrientDeg(allowApprox);
	}

	static inline bool isTaughtReuseAcceptable(const double posMm, const double orientDeg, const bool allowApprox)
	{
		if (posMm < 0.0 || posMm > kTaughtReusePosMm || orientDeg < 0.0)
		{
			return false;
		}
		const double orientGate = allowApprox ? kTaughtReuseOrientDeg : kHardOrientDeg;
		return orientDeg <= orientGate;
	}
};
} // namespace UrdfRobotLoader

#endif // ROBOTURDF_IKACCEPTANCEGATES_H
