/// @file SerialLinkKinematics.cpp
/// @brief 闭式 MDH FK + 解析位置雅可比 DLS IK

#include "SerialLinkKinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_kinematics
{
namespace
{
void mat4Identity(double M[16])
{
	for (int i = 0; i < 16; ++i)
	{
		M[i] = 0.0;
	}
	M[0] = M[5] = M[10] = M[15] = 1.0;
}

void mat4Mul(const double A[16], const double B[16], double AB[16])
{
	double R[16]{};
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			R[c * 4 + r] = A[0 * 4 + r] * B[c * 4 + 0] + A[1 * 4 + r] * B[c * 4 + 1] + A[2 * 4 + r] * B[c * 4 + 2] +
						   A[3 * 4 + r] * B[c * 4 + 3];
		}
	}
	for (int i = 0; i < 16; ++i)
	{
		AB[i] = R[i];
	}
}

/// A_i = Rz(θ) Tz(d) Tx(a) Rx(α) 闭式展开
void dhRowToTransform(const DhRow& row, const std::vector<double>& q, double A[16])
{
	double theta = row.thetaOffset;
	double d = row.d;
	if (row.jointIndex >= 0 && row.jointIndex < static_cast<int>(q.size()))
	{
		const double qv = q[static_cast<std::size_t>(row.jointIndex)];
		if (row.isPrismatic)
		{
			d += qv;
		}
		else
		{
			theta += qv;
		}
	}

	const double ct = std::cos(theta);
	const double st = std::sin(theta);
	const double ca = std::cos(row.alpha);
	const double sa = std::sin(row.alpha);

	A[0 * 4 + 0] = ct;
	A[0 * 4 + 1] = st;
	A[0 * 4 + 2] = 0.0;
	A[0 * 4 + 3] = 0.0;

	A[1 * 4 + 0] = -st * ca;
	A[1 * 4 + 1] = ct * ca;
	A[1 * 4 + 2] = sa;
	A[1 * 4 + 3] = 0.0;

	A[2 * 4 + 0] = st * sa;
	A[2 * 4 + 1] = -ct * sa;
	A[2 * 4 + 2] = ca;
	A[2 * 4 + 3] = 0.0;

	A[3 * 4 + 0] = row.a * ct;
	A[3 * 4 + 1] = row.a * st;
	A[3 * 4 + 2] = d;
	A[3 * 4 + 3] = 1.0;
}

bool fkAccumulateWithPartials(const std::vector<DhRow>& rows, const std::vector<double>& q, double T_world[16],
							  std::vector<double>* T_after_each)
{
	mat4Identity(T_world);
	if (T_after_each)
	{
		T_after_each->assign(rows.size() * 16U, 0.0);
	}
	for (std::size_t i = 0; i < rows.size(); ++i)
	{
		double A[16];
		dhRowToTransform(rows[i], q, A);
		double Tmp[16];
		mat4Mul(T_world, A, Tmp);
		for (int k = 0; k < 16; ++k)
		{
			T_world[k] = Tmp[k];
		}
		if (T_after_each)
		{
			for (int k = 0; k < 16; ++k)
			{
				(*T_after_each)[i * 16U + static_cast<std::size_t>(k)] = T_world[k];
			}
		}
	}
	return true;
}

bool fkAccumulate(const std::vector<DhRow>& rows, const std::vector<double>& q, double T_world[16])
{
	return fkAccumulateWithPartials(rows, q, T_world, nullptr);
}

void positionFromT(const double T[16], double p[3])
{
	p[0] = T[3 * 4 + 0];
	p[1] = T[3 * 4 + 1];
	p[2] = T[3 * 4 + 2];
}

bool solve3x3(const double M[9], double b[3])
{
	const double a00 = M[0], a01 = M[1], a02 = M[2];
	const double a10 = M[3], a11 = M[4], a12 = M[5];
	const double a20 = M[6], a21 = M[7], a22 = M[8];
	const double det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20);
	if (std::abs(det) < 1e-18)
	{
		return false;
	}
	const double invDet = 1.0 / det;
	const double i00 = (a11 * a22 - a12 * a21) * invDet;
	const double i01 = (a02 * a21 - a01 * a22) * invDet;
	const double i02 = (a01 * a12 - a02 * a11) * invDet;
	const double i10 = (a12 * a20 - a10 * a22) * invDet;
	const double i11 = (a00 * a22 - a02 * a20) * invDet;
	const double i12 = (a02 * a10 - a00 * a12) * invDet;
	const double i20 = (a10 * a21 - a11 * a20) * invDet;
	const double i21 = (a01 * a20 - a00 * a21) * invDet;
	const double i22 = (a00 * a11 - a01 * a10) * invDet;
	const double b0 = b[0], b1 = b[1], b2 = b[2];
	b[0] = i00 * b0 + i01 * b1 + i02 * b2;
	b[1] = i10 * b0 + i11 * b1 + i12 * b2;
	b[2] = i20 * b0 + i21 * b1 + i22 * b2;
	return true;
}

double jointStepClamp(const DhRow* rowForJoint, const double dq)
{
	// 棱柱按 mm；旋转按 rad
	const double lim = (rowForJoint && rowForJoint->isPrismatic) ? 50.0 : 0.2;
	return std::max(-lim, std::min(lim, dq));
}

const DhRow* findRowForJoint(const std::vector<DhRow>& rows, const int jointIndex)
{
	for (const DhRow& r : rows)
	{
		if (r.jointIndex == jointIndex)
		{
			return &r;
		}
	}
	return nullptr;
}

} // namespace

std::size_t jointCountFromDhRows(const std::vector<DhRow>& rows)
{
	std::size_t n = 0;
	for (const DhRow& r : rows)
	{
		if (r.jointIndex >= 0)
		{
			n = std::max(n, static_cast<std::size_t>(r.jointIndex) + 1U);
		}
	}
	return n;
}

bool fkSerialDh(const std::vector<DhRow>& rows, const std::vector<double>& q, double T_end4x4_colMajor[16])
{
	if (!T_end4x4_colMajor)
	{
		return false;
	}
	return fkAccumulate(rows, q, T_end4x4_colMajor);
}

bool endEffectorPosition(const std::vector<DhRow>& rows, const std::vector<double>& q, double posOut[3])
{
	double T[16];
	if (!fkAccumulate(rows, q, T))
	{
		return false;
	}
	positionFromT(T, posOut);
	return true;
}

bool positionJacobianAnalytic(const std::vector<DhRow>& rows, const std::vector<double>& q, std::vector<double>& J_3xn)
{
	const std::size_t nJoint = jointCountFromDhRows(rows);
	if (nJoint == 0 || q.size() < nJoint)
	{
		return false;
	}
	double T_ee[16];
	std::vector<double> T_after;
	if (!fkAccumulateWithPartials(rows, q, T_ee, &T_after))
	{
		return false;
	}
	double p_ee[3];
	positionFromT(T_ee, p_ee);

	J_3xn.assign(3U * nJoint, 0.0);
	for (std::size_t i = 0; i < rows.size(); ++i)
	{
		const DhRow& row = rows[i];
		if (row.jointIndex < 0 || row.jointIndex >= static_cast<int>(nJoint))
		{
			continue;
		}
		const std::size_t j = static_cast<std::size_t>(row.jointIndex);
		// 关节 i 的 z 轴：T_{i-1} 的第三列；i=0 时为世界 Z
		double z[3] = {0.0, 0.0, 1.0};
		double p_j[3] = {0.0, 0.0, 0.0};
		if (i > 0)
		{
			const double* T_prev = &T_after[(i - 1U) * 16U];
			z[0] = T_prev[2 * 4 + 0];
			z[1] = T_prev[2 * 4 + 1];
			z[2] = T_prev[2 * 4 + 2];
			p_j[0] = T_prev[3 * 4 + 0];
			p_j[1] = T_prev[3 * 4 + 1];
			p_j[2] = T_prev[3 * 4 + 2];
		}
		if (row.isPrismatic)
		{
			J_3xn[0 * nJoint + j] = z[0];
			J_3xn[1 * nJoint + j] = z[1];
			J_3xn[2 * nJoint + j] = z[2];
		}
		else
		{
			const double rx = p_ee[0] - p_j[0];
			const double ry = p_ee[1] - p_j[1];
			const double rz = p_ee[2] - p_j[2];
			J_3xn[0 * nJoint + j] = z[1] * rz - z[2] * ry;
			J_3xn[1 * nJoint + j] = z[2] * rx - z[0] * rz;
			J_3xn[2 * nJoint + j] = z[0] * ry - z[1] * rx;
		}
	}
	return true;
}

bool ikPositionDampedLeastSquares(const std::vector<DhRow>& rows, const double targetPos[3],
								  std::vector<double>& qInOut, int maxIterations, double positionTolerance,
								  double lambdaDamping, int* iterationsUsed)
{
	const std::size_t nJoint = jointCountFromDhRows(rows);
	if (nJoint == 0 || qInOut.size() < nJoint || !targetPos)
	{
		return false;
	}
	qInOut.resize(nJoint);

	double lambda = lambdaDamping;
	const double lambda2Min = lambdaDamping * lambdaDamping;
	std::vector<double> J(3U * nJoint, 0.0);

	for (int iter = 0; iter < maxIterations; ++iter)
	{
		double p[3];
		if (!endEffectorPosition(rows, qInOut, p))
		{
			return false;
		}
		double e0 = targetPos[0] - p[0];
		double e1 = targetPos[1] - p[1];
		double e2 = targetPos[2] - p[2];
		const double err = std::sqrt(e0 * e0 + e1 * e1 + e2 * e2);
		if (err < positionTolerance)
		{
			if (iterationsUsed)
			{
				*iterationsUsed = iter + 1;
			}
			return true;
		}

		if (!positionJacobianAnalytic(rows, qInOut, J))
		{
			return false;
		}

		const double lambda2 = std::max(lambda2Min, lambda * lambda);
		double M[9]{};
		for (int r = 0; r < 3; ++r)
		{
			for (int c = 0; c < 3; ++c)
			{
				double s = 0.0;
				for (std::size_t k = 0; k < nJoint; ++k)
				{
					s += J[static_cast<std::size_t>(r) * nJoint + k] * J[static_cast<std::size_t>(c) * nJoint + k];
				}
				M[r * 3 + c] = s + (r == c ? lambda2 : 0.0);
			}
		}

		double g[3] = {e0, e1, e2};
		if (!solve3x3(M, g))
		{
			lambda *= 2.0;
			continue;
		}

		double pAfter[3];
		std::vector<double> qTry = qInOut;
		for (std::size_t j = 0; j < nJoint; ++j)
		{
			double dq = 0.0;
			for (int r = 0; r < 3; ++r)
			{
				dq += J[static_cast<std::size_t>(r) * nJoint + j] * g[r];
			}
			qTry[j] += jointStepClamp(findRowForJoint(rows, static_cast<int>(j)), dq);
		}
		if (!endEffectorPosition(rows, qTry, pAfter))
		{
			return false;
		}
		const double errAfter = std::sqrt((targetPos[0] - pAfter[0]) * (targetPos[0] - pAfter[0]) +
										  (targetPos[1] - pAfter[1]) * (targetPos[1] - pAfter[1]) +
										  (targetPos[2] - pAfter[2]) * (targetPos[2] - pAfter[2]));
		if (errAfter > err * 1.05)
		{
			lambda *= 2.0;
			continue;
		}
		lambda = std::max(lambdaDamping, lambda * 0.7);
		qInOut.swap(qTry);
	}

	if (iterationsUsed)
	{
		*iterationsUsed = maxIterations;
	}
	return false;
}

} // namespace robot_kinematics
