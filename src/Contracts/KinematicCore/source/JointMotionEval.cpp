#include "JointMotionEval.h"

#include "Mat4Ops.h"

#include <cmath>

namespace kinematic_core
{
void makeTranslateColumnMajor(const double tx, const double ty, const double tz, double out[16])
{
	mat4IdentityColumnMajor(out);
	out[12] = tx;
	out[13] = ty;
	out[14] = tz;
}

void makeRotateAboutAxisColumnMajor(const double ox, const double oy, const double oz, const double ax, const double ay,
									const double az, const double angleRad, double out[16])
{
	const double c = std::cos(angleRad);
	const double s = std::sin(angleRad);
	const double t = 1.0 - c;
	const double r00 = t * ax * ax + c;
	const double r01 = t * ax * ay - s * az;
	const double r02 = t * ax * az + s * ay;
	const double r10 = t * ax * ay + s * az;
	const double r11 = t * ay * ay + c;
	const double r12 = t * ay * az - s * ax;
	const double r20 = t * ax * az - s * ay;
	const double r21 = t * ay * az + s * ax;
	const double r22 = t * az * az + c;

	double toOrigin[16];
	double rot[16];
	double fromOrigin[16];
	makeTranslateColumnMajor(-ox, -oy, -oz, toOrigin);
	mat4IdentityColumnMajor(rot);
	rot[0] = r00;
	rot[1] = r10;
	rot[2] = r20;
	rot[4] = r01;
	rot[5] = r11;
	rot[6] = r21;
	rot[8] = r02;
	rot[9] = r12;
	rot[10] = r22;
	makeTranslateColumnMajor(ox, oy, oz, fromOrigin);
	double tmp[16];
	mat4MulColumnMajor16(toOrigin, rot, tmp);
	mat4MulColumnMajor16(tmp, fromOrigin, out);
}

void evaluateJointMotion1D(const JointMotion1D& motion, const double q, double outColumnMajor[16])
{
	const double qEff = q * motion.qScale;
	if (motion.motionType == JointMotionType::Translate)
	{
		makeTranslateColumnMajor(motion.axis[0] * qEff, motion.axis[1] * qEff, motion.axis[2] * qEff, outColumnMajor);
		return;
	}
	makeRotateAboutAxisColumnMajor(motion.originMm[0], motion.originMm[1], motion.originMm[2], motion.axis[0],
								   motion.axis[1], motion.axis[2], qEff, outColumnMajor);
}

} // namespace kinematic_core
