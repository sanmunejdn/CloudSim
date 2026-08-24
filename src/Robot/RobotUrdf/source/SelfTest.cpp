/// @file SelfTest.cpp
/// @brief FK/J/DLS 自检：最小链 fixture + 有限差分

#include "SelfTest.h"

#include "KinematicCoreUrdfIk.h"
#include "UrdfIkSolverOptions.h"
#include "UrdfNumericalIk.h"
#include "UrdfNumericalIkLegacy.h"
#include "UrdfRobotLoader.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
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

QString resolveIrb1100UrdfPath()
{
	const QString rel =
		QStringLiteral("resource/models/Robot/ABB/IRB 1100-4-0.58/urdf/IRB 1100-4-0.58.urdf");
	const QStringList candidates = {
		QDir::current().filePath(rel),
		QDir(QCoreApplication::applicationDirPath()).filePath(rel),
	};
	for (const QString& p : candidates)
	{
		if (QFile::exists(p))
		{
			return QFileInfo(p).absoluteFilePath();
		}
	}
	return {};
}

bool runIrb1100FkGolden(std::vector<std::string>& failures)
{
	const QString urdf = resolveIrb1100UrdfPath();
	if (urdf.isEmpty())
	{
		return true;
	}
	QStringList childLinks;
	if (!loadRevoluteJointChildLinksInOrder(urdf, childLinks, nullptr) || childLinks.isEmpty())
	{
		failures.push_back("IRB1100: load child links failed");
		return false;
	}
	const QString flangeLink = childLinks.back();
	const auto checkQ = [&](const QVector<double>& q, const char* label) -> bool {
		double legacyPos[3] = {};
		std::vector<double> J;
		if (!computeLinkPoseAndGeometricJacobian(urdf, q, flangeLink, legacyPos, nullptr, J, false, 1.0, nullptr))
		{
			failures.push_back(std::string("IRB1100 legacy FK failed: ") + label);
			return false;
		}
		QHash<QString, osg::Matrixd> coreLinkWorld;
		if (!computeLinkWorldMatrices(urdf, q, coreLinkWorld, nullptr) || !coreLinkWorld.contains(flangeLink))
		{
			failures.push_back(std::string("IRB1100 core FK failed: ") + label);
			return false;
		}
		const osg::Vec3d corePos = coreLinkWorld.value(flangeLink).getTrans();
		const double dx = legacyPos[0] - corePos.x();
		const double dy = legacyPos[1] - corePos.y();
		const double dz = legacyPos[2] - corePos.z();
		const double errMm = std::sqrt(dx * dx + dy * dy + dz * dz);
		if (errMm > 0.1)
		{
			std::ostringstream os;
			os << "IRB1100 FK delta " << errMm << " mm at " << label;
			failures.push_back(os.str());
			return false;
		}
		return true;
	};

	QVector<double> q0(6, 0.0);
	if (!checkQ(q0, "zero"))
	{
		return false;
	}
	QVector<double> qRand;
	qRand << 0.12 << -0.34 << 0.56 << -0.21 << 0.45 << -0.08;
	return checkQ(qRand, "random");
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

	QHash<QString, osg::Matrixd> linkWorldGraph;
	QString fkErr;
	if (!computeLinkWorldMatrices(urdf, q, linkWorldGraph, &fkErr))
	{
		failures.push_back(std::string("graph FK: ") + fkErr.toStdString());
	}
	else if (!linkWorldGraph.contains(QStringLiteral("link2")))
	{
		failures.push_back("graph FK missing link2");
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
	UrdfPoseIkTarget posOnly{};
	posOnly.posMm[0] = pos[0];
	posOnly.posMm[1] = pos[1];
	posOnly.posMm[2] = pos[2];
	posOnly.hasOrientation = false;
	std::vector<double> qCore = solveArmPoseViaKinematicCore(urdf, QStringLiteral("link2"), posOnly, seed, opt, &ikFail);
	if (qCore.empty())
	{
		failures.push_back(std::string("KinematicCore IK failed: ") + ikFail);
	}

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

	(void)runIrb1100FkGolden(failures);

	clearUrdfModelCache();
	return failures.empty();
}

} // namespace UrdfRobotLoader
