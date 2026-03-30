#include "SerialLinkKinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_kinematics
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

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
			R[c * 4 + r] = A[0 * 4 + r] * B[c * 4 + 0] + A[1 * 4 + r] * B[c * 4 + 1] + A[2 * 4 + r] * B[c * 4 + 2]
				+ A[3 * 4 + r] * B[c * 4 + 3];
		}
	}
	for (int i = 0; i < 16; ++i)
	{
		AB[i] = R[i];
	}
}

void rotX(double alpha, double R[16])
{
	mat4Identity(R);
	const double ca = std::cos(alpha);
	const double sa = std::sin(alpha);
	R[1 * 4 + 1] = ca;
	R[1 * 4 + 2] = -sa;
	R[2 * 4 + 1] = sa;
	R[2 * 4 + 2] = ca;
}

void rotZ(double theta, double R[16])
{
	mat4Identity(R);
	const double ct = std::cos(theta);
	const double st = std::sin(theta);
	R[0 * 4 + 0] = ct;
	R[0 * 4 + 1] = -st;
	R[1 * 4 + 0] = st;
	R[1 * 4 + 1] = ct;
}

void trans(double x, double y, double z, double T[16])
{
	mat4Identity(T);
	T[3 * 4 + 0] = x;
	T[3 * 4 + 1] = y;
	T[3 * 4 + 2] = z;
}

/// A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
void dhRowToTransform(const DhRow& row, const std::vector<double>& q, double A[16])
{
	double theta = row.thetaOffset;
	if (row.jointIndex >= 0 && row.jointIndex < static_cast<int>(q.size()))
	{
		theta += q[static_cast<std::size_t>(row.jointIndex)];
	}

	double Rz[16], Tz[16], Tx[16], Rx[16], T1[16], T2[16], T3[16];
	rotZ(theta, Rz);
	trans(0.0, 0.0, row.d, Tz);
	trans(row.a, 0.0, 0.0, Tx);
	rotX(row.alpha, Rx);
	mat4Mul(Rz, Tz, T1);
	mat4Mul(T1, Tx, T2);
	mat4Mul(T2, Rx, A);
}

bool fkAccumulate(const std::vector<DhRow>& rows, const std::vector<double>& q, double T_world[16])
{
	mat4Identity(T_world);
	for (const DhRow& row : rows)
	{
		double A[16];
		dhRowToTransform(row, q, A);
		double Tmp[16];
		mat4Mul(T_world, A, Tmp);
		for (int i = 0; i < 16; ++i)
		{
			T_world[i] = Tmp[i];
		}
	}
	return true;
}

void positionFromT(const double T[16], double p[3])
{
	p[0] = T[3 * 4 + 0];
	p[1] = T[3 * 4 + 1];
	p[2] = T[3 * 4 + 2];
}

bool solve3x3(const double M[9], double b[3])
{
	// Cramer / explicit inverse for 3x3
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

bool ikPositionDampedLeastSquares(
	const std::vector<DhRow>& rows,
	const double targetPos[3],
	std::vector<double>& qInOut,
	int maxIterations,
	double positionTolerance,
	double lambdaDamping,
	int* iterationsUsed)
{
	const std::size_t nJoint = jointCountFromDhRows(rows);
	if (nJoint == 0 || qInOut.size() < nJoint || !targetPos)
	{
		return false;
	}
	qInOut.resize(nJoint);

	const double eps = 1e-6;
	const double lambda2 = lambdaDamping * lambdaDamping;

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

		// J: 3 x nJoint, numerical columns
		std::vector<double> J(static_cast<std::size_t>(3 * nJoint), 0.0);
		for (std::size_t j = 0; j < nJoint; ++j)
		{
			std::vector<double> qP = qInOut;
			qP[j] += eps;
			double pp[3];
			if (!endEffectorPosition(rows, qP, pp))
			{
				return false;
			}
			J[0 * nJoint + j] = (pp[0] - p[0]) / eps;
			J[1 * nJoint + j] = (pp[1] - p[1]) / eps;
			J[2 * nJoint + j] = (pp[2] - p[2]) / eps;
		}

		// M = J * J^T + lambda^2 * I  (3x3)
		double M[9]{};
		for (int r = 0; r < 3; ++r)
		{
			for (int c = 0; c < 3; ++c)
			{
				double s = 0.0;
				for (std::size_t k = 0; k < nJoint; ++k)
				{
					s += J[static_cast<std::size_t>(r * static_cast<int>(nJoint) + k)]
						* J[static_cast<std::size_t>(c * static_cast<int>(nJoint) + k)];
				}
				M[r * 3 + c] = s + (r == c ? lambda2 : 0.0);
			}
		}

		double g[3] = {e0, e1, e2};
		if (!solve3x3(M, g))
		{
			return false;
		}

		// dq = J^T * g
		for (std::size_t j = 0; j < nJoint; ++j)
		{
			double dq = 0.0;
			for (int r = 0; r < 3; ++r)
			{
				dq += J[static_cast<std::size_t>(r * static_cast<int>(nJoint) + j)] * g[r];
			}
			qInOut[j] += dq;
		}
	}

	if (iterationsUsed)
	{
		*iterationsUsed = maxIterations;
	}
	return false;
}

} // namespace robot_kinematics
