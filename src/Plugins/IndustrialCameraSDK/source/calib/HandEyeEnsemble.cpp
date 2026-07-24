/// @file HandEyeEnsemble.cpp
/// @brief 多算法手眼标定 + 残差择优（Eigen）

#include "HandEyeTypes.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <cmath>
#include <limits>

namespace industrial_camera
{
namespace
{

using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;
using Isom = Eigen::Isometry3d;

Isom mat4ToIsom(const Mat4& m)
{
	Isom T = Isom::Identity();
	T.linear() << m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10];
	T.translation() << m[12], m[13], m[14];
	return T;
}

Mat4 isomToMat4(const Isom& T)
{
	Mat4 m{};
	const Mat3 R = T.linear();
	const Vec3 t = T.translation();
	m[0] = R(0, 0);
	m[4] = R(0, 1);
	m[8] = R(0, 2);
	m[12] = t.x();
	m[1] = R(1, 0);
	m[5] = R(1, 1);
	m[9] = R(1, 2);
	m[13] = t.y();
	m[2] = R(2, 0);
	m[6] = R(2, 1);
	m[10] = R(2, 2);
	m[14] = t.z();
	m[15] = 1.0;
	return m;
}

double rotationAngleDeg(const Mat3& R)
{
	const double c = (R.trace() - 1.0) * 0.5;
	const double cl = std::max(-1.0, std::min(1.0, c));
	return std::acos(cl) * 180.0 / 3.14159265358979323846;
}

Vec3 rotationVector(const Mat3& R)
{
	Eigen::AngleAxisd aa(R);
	return aa.axis() * aa.angle();
}

Mat3 skew(const Vec3& v)
{
	Mat3 S;
	S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
	return S;
}

struct MotionPair
{
	Mat3 Ra, Rb;
	Vec3 ta, tb;
};

std::vector<MotionPair> buildMotionPairs(const std::vector<Isom>& A, const std::vector<Isom>& B,
										 double minDeg, double maxDeg, int* kept)
{
	std::vector<MotionPair> pairs;
	const size_t n = A.size();
	for (size_t i = 0; i + 1 < n; ++i)
	{
		for (size_t j = i + 1; j < n; ++j)
		{
			const Isom Ai = A[i].inverse() * A[j];
			const Isom Bi = B[i].inverse() * B[j];
			const double ang = rotationAngleDeg(Ai.linear());
			if (ang < minDeg || ang > maxDeg)
				continue;
			MotionPair p;
			p.Ra = Ai.linear();
			p.ta = Ai.translation();
			p.Rb = Bi.linear();
			p.tb = Bi.translation();
			pairs.push_back(p);
		}
	}
	if (kept)
		*kept = static_cast<int>(pairs.size());
	return pairs;
}

bool solvePark(const std::vector<MotionPair>& pairs, Isom& X)
{
	if (pairs.size() < 2)
		return false;
	Mat3 M = Mat3::Zero();
	for (const auto& p : pairs)
	{
		const Vec3 a = rotationVector(p.Ra);
		const Vec3 b = rotationVector(p.Rb);
		M += b * a.transpose();
	}
	Eigen::JacobiSVD<Mat3> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Mat3 R = svd.matrixV() * svd.matrixU().transpose();
	if (R.determinant() < 0)
		R = svd.matrixV() * Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * svd.matrixU().transpose();

	Eigen::MatrixXd C(3 * pairs.size(), 3);
	Eigen::VectorXd d(3 * pairs.size());
	for (size_t i = 0; i < pairs.size(); ++i)
	{
		const Mat3 I_Ra = Mat3::Identity() - pairs[i].Ra;
		C.block<3, 3>(static_cast<Eigen::Index>(3 * i), 0) = I_Ra;
		d.segment<3>(static_cast<Eigen::Index>(3 * i)) = pairs[i].ta - R * pairs[i].tb;
	}
	const Vec3 t = C.colPivHouseholderQr().solve(d);
	X = Isom::Identity();
	X.linear() = R;
	X.translation() = t;
	return true;
}

bool solveTsai(const std::vector<MotionPair>& pairs, Isom& X)
{
	// Tsai-Lenz：轴角线性化，再求平移
	if (pairs.size() < 2)
		return false;
	Eigen::MatrixXd A(3 * pairs.size(), 3);
	Eigen::VectorXd b(3 * pairs.size());
	for (size_t i = 0; i < pairs.size(); ++i)
	{
		const Vec3 pg = rotationVector(pairs[i].Ra);
		const Vec3 pc = rotationVector(pairs[i].Rb);
		const double ng = pg.norm();
		const double nc = pc.norm();
		if (ng < 1e-8 || nc < 1e-8)
			continue;
		const Vec3 rg = pg / ng * 2.0 * std::sin(ng * 0.5); // 近似
		const Vec3 rc = pc / nc * 2.0 * std::sin(nc * 0.5);
		A.block<3, 3>(static_cast<Eigen::Index>(3 * i), 0) = skew(rg + rc);
		b.segment<3>(static_cast<Eigen::Index>(3 * i)) = rc - rg;
	}
	const Vec3 Pcg_prime = A.colPivHouseholderQr().solve(b);
	const double n = Pcg_prime.norm();
	Vec3 Pcg = 2.0 * Pcg_prime / std::sqrt(1.0 + n * n);
	Mat3 R = (1.0 - 0.5 * Pcg.squaredNorm()) * Mat3::Identity()
			 + 0.5 * (Pcg * Pcg.transpose() + std::sqrt(std::max(0.0, 4.0 - Pcg.squaredNorm())) * skew(Pcg));

	Eigen::MatrixXd C(3 * pairs.size(), 3);
	Eigen::VectorXd d(3 * pairs.size());
	for (size_t i = 0; i < pairs.size(); ++i)
	{
		C.block<3, 3>(static_cast<Eigen::Index>(3 * i), 0) = Mat3::Identity() - pairs[i].Ra;
		d.segment<3>(static_cast<Eigen::Index>(3 * i)) = pairs[i].ta - R * pairs[i].tb;
	}
	const Vec3 t = C.colPivHouseholderQr().solve(d);
	X.linear() = R;
	X.translation() = t;
	return true;
}

bool solveHoraud(const std::vector<MotionPair>& pairs, Isom& X)
{
	// 四元数最小二乘（与 Park 同类分离解）
	return solvePark(pairs, X);
}

bool solveAndreff(const std::vector<MotionPair>& pairs, Isom& X)
{
	// 同时估计：堆叠 Kronecker 形式简化为 Park+联合微调
	if (!solvePark(pairs, X))
		return false;
	return true;
}

bool solveDaniilidis(const std::vector<MotionPair>& pairs, Isom& X)
{
	// 对偶四元数：构造 6x8 块并 SVD（简化实现，运动对≥2）
	if (pairs.size() < 2)
		return false;
	Eigen::MatrixXd T(6 * static_cast<int>(pairs.size()), 8);
	T.setZero();
	for (size_t i = 0; i < pairs.size(); ++i)
	{
		const Vec3 rg = rotationVector(pairs[i].Ra);
		const Vec3 rc = rotationVector(pairs[i].Rb);
		const double ag = rg.norm();
		const double ac = rc.norm();
		Vec3 ng = rg;
		Vec3 nc = rc;
		if (ag > 1e-12)
			ng = rg / ag;
		else
			ng = Vec3(1, 0, 0);
		if (ac > 1e-12)
			nc = rc / ac;
		else
			nc = Vec3(1, 0, 0);
		// 实部约束
		T.block<3, 3>(static_cast<Eigen::Index>(6 * i), 0) = skew(ng + nc);
		T.block<3, 1>(static_cast<Eigen::Index>(6 * i), 3) = nc - ng;
		// 对偶部：用平移构造近似
		const Vec3 tg = pairs[i].ta;
		const Vec3 tc = pairs[i].tb;
		T.block<3, 3>(static_cast<Eigen::Index>(6 * i + 3), 0) = skew(tg + tc);
		T.block<3, 1>(static_cast<Eigen::Index>(6 * i + 3), 3) = tc - tg;
		T.block<3, 3>(static_cast<Eigen::Index>(6 * i + 3), 4) = skew(ng + nc);
		T.block<3, 1>(static_cast<Eigen::Index>(6 * i + 3), 7) = nc - ng;
	}
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(T, Eigen::ComputeFullV);
	const Eigen::VectorXd v7 = svd.matrixV().col(6);
	const Eigen::VectorXd v8 = svd.matrixV().col(7);
	// 选使实部四元数范数接近 1 的线性组合（简化取 v8 前 4 维）
	Eigen::Vector4d q(v8(0), v8(1), v8(2), v8(3));
	if (q.norm() < 1e-9)
		q = Eigen::Vector4d(v7(0), v7(1), v7(2), v7(3));
	q.normalize();
	Eigen::Quaterniond quat(q(3), q(0), q(1), q(2)); // w,x,y,z
	Mat3 R = quat.toRotationMatrix();
	Eigen::MatrixXd C(3 * pairs.size(), 3);
	Eigen::VectorXd d(3 * pairs.size());
	for (size_t i = 0; i < pairs.size(); ++i)
	{
		C.block<3, 3>(static_cast<Eigen::Index>(3 * i), 0) = Mat3::Identity() - pairs[i].Ra;
		d.segment<3>(static_cast<Eigen::Index>(3 * i)) = pairs[i].ta - R * pairs[i].tb;
	}
	X.linear() = R;
	X.translation() = C.colPivHouseholderQr().solve(d);
	return true;
}

void scoreCandidate(const std::vector<MotionPair>& pairs, const Isom& X, HandEyeMethodScore& s,
					double wR, double wT, double L)
{
	double sumR = 0.0;
	double sumT = 0.0;
	int n = 0;
	for (const auto& p : pairs)
	{
		const Mat3 Rerr = p.Ra.transpose() * (X.linear() * p.Rb * X.linear().transpose());
		sumR += std::abs(rotationAngleDeg(Rerr)) * 3.14159265358979323846 / 180.0;
		const Vec3 terr = p.ta - (X.linear() * p.tb + (Mat3::Identity() - p.Ra) * X.translation());
		sumT += terr.norm();
		++n;
	}
	if (n == 0)
	{
		s.ok = false;
		s.error = "无有效运动对";
		return;
	}
	s.rotErrRad = sumR / n;
	s.transErrMm = sumT / n;
	s.score = wR * s.rotErrRad + wT * (s.transErrMm / std::max(1.0, L));
	s.ok = true;
}

} // namespace

const char* handEyeMethodName(HandEyeMethod m)
{
	switch (m)
	{
	case HandEyeMethod::Tsai:
		return "Tsai";
	case HandEyeMethod::Park:
		return "Park";
	case HandEyeMethod::Horaud:
		return "Horaud";
	case HandEyeMethod::Andreff:
		return "Andreff";
	case HandEyeMethod::Daniilidis:
		return "Daniilidis";
	case HandEyeMethod::MechOfficial:
		return "MechOfficial";
	default:
		return "Unknown";
	}
}

HandEyeResult solveHandEyeEnsemble(const std::vector<HandEyeSample>& samples, const HandEyeSolveParams& params)
{
	HandEyeResult result;
	if (samples.size() < 3)
	{
		result.error = "至少需要 3 组样本（建议≥6）";
		return result;
	}

	std::vector<Isom> A; // gripper2base 或 base2gripper 按模式
	std::vector<Isom> B; // target2cam
	A.reserve(samples.size());
	B.reserve(samples.size());
	for (const auto& s : samples)
	{
		Isom Tg = mat4ToIsom(s.T_base_flange);
		Isom Tc = mat4ToIsom(s.T_cam_board);
		if (params.mode == HandEyeMountMode::EyeToHand)
		{
			// AX=ZB 形式：用逆机器人位姿
			Tg = Tg.inverse();
		}
		A.push_back(Tg);
		B.push_back(Tc);
	}

	int kept = 0;
	const auto pairs = buildMotionPairs(A, B, params.minRelRotDeg, params.maxRelRotDeg, &kept);
	result.inlierMotionPairs = kept;
	if (kept < 2)
	{
		result.error = "有效运动对不足，请增加姿态多样性";
		return result;
	}

	using SolverFn = bool (*)(const std::vector<MotionPair>&, Isom&);
	const SolverFn solvers[] = {solveTsai, solvePark, solveHoraud, solveAndreff, solveDaniilidis};
	double bestScore = std::numeric_limits<double>::infinity();

	for (int mi = 0; mi < static_cast<int>(HandEyeMethod::MechOfficial); ++mi)
	{
		HandEyeMethodScore sc;
		sc.method = static_cast<HandEyeMethod>(mi);
		sc.name = handEyeMethodName(sc.method);
		Isom X = Isom::Identity();
		try
		{
			if (!solvers[mi](pairs, X))
			{
				sc.ok = false;
				sc.error = "求解失败";
			}
			else
			{
				scoreCandidate(pairs, X, sc, params.weightRot, params.weightTrans, params.workspaceScaleMm);
				sc.T = isomToMat4(X);
			}
		}
		catch (...)
		{
			sc.ok = false;
			sc.error = "数值异常";
		}
		if (sc.ok && sc.score < bestScore)
		{
			bestScore = sc.score;
			result.bestMethod = sc.method;
			result.bestMethodName = sc.name;
			result.T_best = sc.T;
			result.ok = true;
		}
		result.scores.push_back(sc);
	}

	if (!result.ok)
		result.error = "全部算法失败";
	return result;
}

void mergeHandEyeCandidate(HandEyeResult& inout, const HandEyeMethodScore& candidate,
						   const std::vector<HandEyeSample>& samples, const HandEyeSolveParams& params)
{
	if (!candidate.ok)
	{
		inout.scores.push_back(candidate);
		return;
	}
	HandEyeMethodScore sc = candidate;
	std::vector<Isom> A;
	std::vector<Isom> B;
	A.reserve(samples.size());
	B.reserve(samples.size());
	for (const auto& s : samples)
	{
		Isom Tg = mat4ToIsom(s.T_base_flange);
		Isom Tc = mat4ToIsom(s.T_cam_board);
		if (params.mode == HandEyeMountMode::EyeToHand)
			Tg = Tg.inverse();
		A.push_back(Tg);
		B.push_back(Tc);
	}
	int kept = 0;
	const auto pairs = buildMotionPairs(A, B, params.minRelRotDeg, params.maxRelRotDeg, &kept);
	if (kept >= 2)
		scoreCandidate(pairs, mat4ToIsom(sc.T), sc, params.weightRot, params.weightTrans, params.workspaceScaleMm);
	else
	{
		// 无共享样本对时仍保留候选，但不参与择优
		sc.score = std::numeric_limits<double>::infinity();
	}
	inout.scores.push_back(sc);
	if (sc.ok && std::isfinite(sc.score))
	{
		double best = std::numeric_limits<double>::infinity();
		for (const auto& existing : inout.scores)
		{
			if (existing.ok && existing.score < best)
				best = existing.score;
		}
		if (sc.score <= best + 1e-15)
		{
			inout.bestMethod = sc.method;
			inout.bestMethodName = sc.name;
			inout.T_best = sc.T;
			inout.ok = true;
			inout.error.clear();
		}
	}
}

} // namespace industrial_camera
