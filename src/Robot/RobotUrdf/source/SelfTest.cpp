/// @file SelfTest.cpp
/// @brief FK/J/DLS 自检：最小链 fixture + 有限差分

#include "SelfTest.h"

#include "UrdfIkSolverOptions.h"
#include "UrdfNumericalIk.h"
#include "UrdfRobotLoader.h"

#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <QTextStream>
#include <cmath>
#include <sstream>

namespace UrdfRobotLoader
{
namespace
{
QString writeFixtureUrdf(QTemporaryDir& dir)
{
	const QString path = dir.filePath(QStringLiteral("two_link.urdf"));
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
	{
		return {};
	}
	QTextStream out(&f);
	out << R"(<?xml version="1.0"?>
<robot name="two_link">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="0" velocity="0"/>
  </joint>
</robot>
)";
	f.close();
	return path;
}

bool nearlyEqual(double a, double b, double tol)
{
	return std::abs(a - b) <= tol;
}
} // namespace

bool runSelfTest(std::vector<std::string>& failures)
{
	failures.clear();
	QTemporaryDir dir;
	if (!dir.isValid())
	{
		failures.push_back("temp dir");
		return false;
	}
	const QString urdf = writeFixtureUrdf(dir);
	if (urdf.isEmpty())
	{
		failures.push_back("write fixture urdf");
		return false;
	}

	clearUrdfModelCache();
	QVector<double> q;
	q << 0.3 << -0.2;
	double pos[3] = {0.0, 0.0, 0.0};
	double quat[4] = {0.0, 0.0, 0.0, 1.0};
	std::vector<double> J;
	QString err;
	if (!computeLinkPoseAndGeometricJacobian(urdf, q, QStringLiteral("link2"), pos, quat, J, true, 300.0, &err))
	{
		failures.push_back(std::string("FK/J failed: ") + err.toStdString());
		return false;
	}

	// origin: j1 抬升 100mm，j2 沿 x 200mm；q1=0.3 q2=-0.2 时位置由 FK 自洽
	const double goldenZ = 100.0;
	if (!nearlyEqual(pos[2], goldenZ, 1e-6))
	{
		std::ostringstream os;
		os << "FK z golden: got " << pos[2] << " expect " << goldenZ;
		failures.push_back(os.str());
	}

	const int n = 2;
	if (static_cast<int>(J.size()) < 6 * n)
	{
		failures.push_back("J size");
		return failures.empty();
	}
	constexpr double kEps = 1e-6;
	for (int j = 0; j < n; ++j)
	{
		QVector<double> qp = q;
		QVector<double> qm = q;
		qp[j] += kEps;
		qm[j] -= kEps;
		double pp[3] = {};
		double pm[3] = {};
		std::vector<double> Jtmp;
		if (!computeLinkPoseAndGeometricJacobian(urdf, qp, QStringLiteral("link2"), pp, nullptr, Jtmp, false, 1.0,
												 nullptr) ||
			!computeLinkPoseAndGeometricJacobian(urdf, qm, QStringLiteral("link2"), pm, nullptr, Jtmp, false, 1.0,
												 nullptr))
		{
			failures.push_back("FD FK failed");
			break;
		}
		for (int r = 0; r < 3; ++r)
		{
			const double fd = (pp[r] - pm[r]) / (2.0 * kEps);
			const double an = J[static_cast<size_t>(r * n + j)];
			const double scale = std::max(1.0, std::abs(fd));
			if (std::abs(fd - an) > 1e-3 * scale + 1e-4)
			{
				std::ostringstream os;
				os << "J fd mismatch r=" << r << " j=" << j << " fd=" << fd << " an=" << an;
				failures.push_back(os.str());
			}
		}
	}

	UrdfPoseIkTarget target{};
	target.posMm[0] = pos[0];
	target.posMm[1] = pos[1];
	target.posMm[2] = pos[2];
	target.hasOrientation = true;
	target.quatXyzw[0] = quat[0];
	target.quatXyzw[1] = quat[1];
	target.quatXyzw[2] = quat[2];
	target.quatXyzw[3] = quat[3];
	UrdfIkSolverOptions opt{};
	opt.maxIterations = 80;
	std::vector<double> seed = {0.0, 0.0};
	std::string ikFail;
	std::vector<double> qIk = solveArmPoseDampedLeastSquares(urdf, QStringLiteral("link2"), target, seed, opt, &ikFail);
	if (qIk.empty())
	{
		failures.push_back(std::string("DLS failed: ") + ikFail);
	}
	else
	{
		double pos2[3] = {};
		std::vector<double> J2;
		QVector<double> qIkQt;
		qIkQt.reserve(static_cast<int>(qIk.size()));
		for (double v : qIk)
		{
			qIkQt.push_back(v);
		}
		if (!computeLinkPoseAndGeometricJacobian(urdf, qIkQt, QStringLiteral("link2"), pos2, nullptr, J2, false, 1.0,
												 nullptr))
		{
			failures.push_back("DLS FK check failed");
		}
		else
		{
			const double dx = pos2[0] - pos[0];
			const double dy = pos2[1] - pos[1];
			const double dz = pos2[2] - pos[2];
			const double errMm = std::sqrt(dx * dx + dy * dy + dz * dz);
			if (errMm > 0.05)
			{
				std::ostringstream os;
				os << "DLS residual " << errMm;
				failures.push_back(os.str());
			}
		}
	}

	clearUrdfModelCache();
	return failures.empty();
}

} // namespace UrdfRobotLoader
