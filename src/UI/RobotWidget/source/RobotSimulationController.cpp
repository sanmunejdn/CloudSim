/// @file RobotSimulationController.cpp
/// @brief 机器人仿真编排（URDF/程序/TCP 示教）

#include "RobotSimulationController.h"

#include "../../OsgWidgetCore/inc/ObjectGizmoFrame.h"
#include "../../OsgWidgetCore/inc/OsgScene.h"
#include "CoreTypes.h"
#include "FeaturePickTransform.h"
#include "FeatureTrajectoryPageWidget.h"
#include "IRobotMainWindowHost.h"
#include "IRobotMotionClient.h"
#include "IRobotOsgViewHost.h"
#include "InstructionProgramDocument.h"
#include "InstructionProgramTreeWidget.h"
#include "PlanResultCache.h"
#include "ProgramEditService.h"
#include "RawTrajectory.h"
#include "RobotExternalAxisSceneApply.h"
#include "RobotAxisControlWidget.h"
#include "RobotCollisionSettingsWidget.h"
#include "MotionPathPlanDialog.h"
#include "RobotCanonicalProgramExport.h"
#include "BrandProgramExportDialog.h"
#include "PythonScriptCaller.h"
#include "RobotFrameSettingsWidget.h"
#include "RobotExternalAxisSettingsWidget.h"
#include "RobotCommPageWidget.h"

#include <json.hpp>
#include "RobotInstructionPlanningHelpers.h"
#include "RobotInstructionProgram.h"
#include "Adapters.h"
#include "RobotInstructionTransform.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotOsgUiTypes.h"
#include "RobotProgramExport.h"
#include "RobotExternalAxes.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "KinematicModelApply.h"
#include "KinematicModelRegistry.h"
#include "BackendTypeIds.h"
#include "AxisControlTargetService.h"
#include "CustomDeviceSimService.h"
#include "DeviceCommandPageWidget.h"
#include "DevicePoseMotionPlayer.h"
#include "DevicePoseSignalDriver.h"
#include "DockNavigationService.h"
#include "IoSignalNetworkService.h"
#include "RobotSimulationDockWidget.h"
#include "BackendCollisionSync.h"
#include "RobotPathPlanning.h"
#include "CollisionWorld.h"
#include "MeshBackendData.h"
#include "BrepBackendData.h"
#include "RobotSceneKinematics.h"
#include "RobotSimulationDockWidget.h"
#include "RobotSimulationMath.h"
#include "RobotTeachIk.h"
#include "RobotJointWrap.h"
#include "RunLogger.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryEditPageWidget.h"
#include "TrajectoryEditSession.h"
#include "TrajectoryGenerationPageWidget.h"
#include "UrdfRobotLoader.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QGuiApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QPointer>
#include <QSet>
#include <QSignalBlocker>
#include <QTemporaryFile>
#include <QTimer>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <Adapters.h>
#include <BackendDataBase.h>
#include <BackendDataManager.h>
#include "IRobotBackendPoseSink.h"
#include <ToolKinematics.h>
#include <osg/Matrixd>

using namespace RobotSimulation;

namespace
{
constexpr double kTaughtReuseResidualMm = 1.0;
/// 示教/缓存复用姿态门限（过严会导致「预览到、Run 不到」）
constexpr double kMaxTaughtOrientReuseDeg = 15.0;
/// 新鲜 IK 仅拦近翻转；联立求解按位置选优，5° 会误杀可用解
constexpr double kMaxFreshIkOrientDeg = 45.0;
constexpr double kFreshIkPositionRejectMm = 3.0;
/// 随机兜底候选的连续性硬门：与链式种子的未折圈逐关节最大偏差（执行器真实行程，±2π 必淘汰）
/// 只约束随机兜底；确定性种子 / 示教 CSV 不受此限
constexpr double kFallbackConsistencyRad = 2.6;
/// 收集-择优的提前收口：最佳候选已足够近时不再消耗时间盒
constexpr double kFallbackNearEnoughRad = 0.35;
constexpr double kPi = 3.14159265358979323846;

bool isTaughtOrCacheReuseAcceptable(const double residualMm, const double orientDeg)
{
	return residualMm >= 0.0 && residualMm <= kTaughtReuseResidualMm && orientDeg >= 0.0 &&
		   orientDeg <= kMaxTaughtOrientReuseDeg;
}

bool isFreshIkSolutionAcceptable(const double residualMm, const double orientDeg)
{
	if (residualMm < 0.0 || residualMm > kFreshIkPositionRejectMm)
	{
		return false;
	}
	if (orientDeg < 0.0)
	{
		return true;
	}
	return orientDeg <= kMaxFreshIkOrientDeg;
}

/// 确定性种子全败时的随机重启兜底：固定 RNG 序列保证可复现；同步/Worker 两条规划路径共用。
/// 前半沿用 ±π 均匀采样（序列与已验证行为逐位一致）；后半在 URDF 限位内逐关节采样（TRAC-IK 方式），
/// 覆盖宽限位关节的远端分支，同时避免越限种子被钳制到边界后重复浪费。
/// 收集-择优与时间盒在 tryPass 的 collectNearest 模式内：盒内收集全部过门候选，选距链式种子最近者
template <typename TryPass>
bool tryRandomRestartPass(TryPass&& tryPass, const int nj, const QVector<double>& lowerRad,
						  const QVector<double>& upperRad, const qint64 budgetMs)
{
	constexpr int kUniformSeeds = 8;
	constexpr int kLimitSeeds = 8;
	std::mt19937 rng(80591);
	std::uniform_real_distribution<double> uniFull(-kPi, kPi);
	const bool hasLimits = lowerRad.size() == nj && upperRad.size() == nj;
	QVector<QVector<double>> randomSeeds;
	randomSeeds.reserve(kUniformSeeds + kLimitSeeds);
	for (int k = 0; k < kUniformSeeds + kLimitSeeds; ++k)
	{
		const bool useLimits = hasLimits && k >= kUniformSeeds;
		QVector<double> rs(nj);
		for (int j = 0; j < nj; ++j)
		{
			if (useLimits && lowerRad[j] < upperRad[j])
			{
				std::uniform_real_distribution<double> uniLimit(lowerRad[j], upperRad[j]);
				rs[j] = uniLimit(rng);
			}
			else
			{
				rs[j] = uniFull(rng);
			}
		}
		randomSeeds.push_back(rs);
	}
	return tryPass(randomSeeds, false, true, budgetMs);
}

double trapezoidTravelDurationSec(const double distance, const double vMax, const double accel)
{
	const double d = std::abs(distance);
	if (d < 1e-9)
	{
		return 0.05;
	}
	const double v = vMax;
	const double a = accel;
	const double tAcc = v / a;
	const double dAcc = 0.5 * a * tAcc * tAcc;
	if (d <= 2.0 * dAcc)
	{
		return std::max(0.05, 2.0 * std::sqrt(d / a));
	}
	return std::max(0.05, 2.0 * tAcc + (d - 2.0 * dAcc) / v);
}

QVector<double> enabledValuesFromFullQ(const RobotExternal::RobotExternalAxisConfigSet& set,
									   const std::vector<double>& fullQ)
{
	QVector<double> out;
	const std::vector<int> idxs = RobotExternal::enabledExternalAxisIndices(set);
	out.reserve(static_cast<int>(idxs.size()));
	for (const int idx : idxs)
	{
		double v = 0.0;
		if (idx >= 0 && idx < static_cast<int>(fullQ.size()))
		{
			v = fullQ[static_cast<size_t>(idx)];
		}
		else if (idx >= 0 && idx < static_cast<int>(set.axes.size()))
		{
			v = set.axes[static_cast<size_t>(idx)].home;
		}
		out.push_back(v);
	}
	return out;
}

std::vector<double> fullQFromEnabledValues(const RobotExternal::RobotExternalAxisConfigSet& set,
										   const QVector<double>& enabledValues)
{
	std::vector<double> full(set.axes.size(), 0.0);
	for (size_t i = 0; i < set.axes.size(); ++i)
	{
		full[i] = set.axes[i].home;
	}
	const std::vector<int> idxs = RobotExternal::enabledExternalAxisIndices(set);
	for (int e = 0; e < static_cast<int>(idxs.size()) && e < enabledValues.size(); ++e)
	{
		const int idx = idxs[static_cast<size_t>(e)];
		if (idx >= 0 && idx < static_cast<int>(full.size()))
		{
			full[static_cast<size_t>(idx)] = enabledValues[e];
		}
	}
	return full;
}

double firstRobotBaseEnabledQ(const RobotExternal::RobotExternalAxisConfigSet& set, const std::vector<double>& qs,
							  const double fallback)
{
	const std::vector<int> idxs = RobotExternal::enabledExternalAxisIndices(set);
	for (const int idx : idxs)
	{
		if (idx >= 0 && idx < static_cast<int>(set.axes.size()) &&
			set.axes[static_cast<size_t>(idx)].attachment == RobotExternal::RobotExternalAttachment::RobotBase &&
			idx < static_cast<int>(qs.size()))
		{
			return qs[static_cast<size_t>(idx)];
		}
	}
	for (const int idx : idxs)
	{
		if (idx >= 0 && idx < static_cast<int>(qs.size()))
		{
			return qs[static_cast<size_t>(idx)];
		}
	}
	return qs.empty() ? fallback : qs.front();
}

std::vector<double> expandScalarExternalAxisQ(const RobotExternal::RobotExternalAxisConfigSet& set, const double qScalar)
{
	std::vector<double> full(set.axes.size(), 0.0);
	for (size_t i = 0; i < set.axes.size(); ++i)
	{
		full[i] = set.axes[i].home;
	}
	const std::vector<int> idxs = RobotExternal::enabledExternalAxisIndices(set);
	bool wrote = false;
	for (const int idx : idxs)
	{
		if (idx >= 0 && idx < static_cast<int>(set.axes.size()) &&
			set.axes[static_cast<size_t>(idx)].attachment == RobotExternal::RobotExternalAttachment::RobotBase)
		{
			full[static_cast<size_t>(idx)] = qScalar;
			wrote = true;
			break;
		}
	}
	if (!wrote && !idxs.empty())
	{
		const int idx = idxs.front();
		if (idx >= 0 && idx < static_cast<int>(full.size()))
		{
			full[static_cast<size_t>(idx)] = qScalar;
		}
	}
	return full;
}

std::vector<double> padExternalAxisQToConfig(const RobotExternal::RobotExternalAxisConfigSet& set,
											 const std::vector<double>& qs)
{
	std::vector<double> full(set.axes.size(), 0.0);
	for (size_t i = 0; i < set.axes.size(); ++i)
	{
		full[i] = set.axes[i].home;
	}
	for (size_t i = 0; i < full.size() && i < qs.size(); ++i)
	{
		full[i] = qs[i];
	}
	return full;
}

QVector<double> toQVector(const std::vector<double>& v)
{
	QVector<double> out;
	out.reserve(static_cast<int>(v.size()));
	for (double x : v)
	{
		out.push_back(x);
	}
	return out;
}

std::vector<double> toStdVector(const QVector<double>& v)
{
	std::vector<double> out;
	out.reserve(static_cast<size_t>(v.size()));
	for (double x : v)
	{
		out.push_back(x);
	}
	return out;
}

double externalAxisTravelDurationSec(const RobotExternal::RobotExternalAxisConfigSet& set,
									 const std::vector<double>& fromQ, const std::vector<double>& toQ)
{
	constexpr double kTranslateVMmPerSec = 250.0;
	constexpr double kTranslateAMmPerSec2 = 600.0;
	constexpr double kRotateVRadPerSec = 1.0;
	constexpr double kRotateARadPerSec2 = 2.4;
	double maxT = 0.05;
	for (size_t i = 0; i < set.axes.size(); ++i)
	{
		if (!set.axes[i].enabled)
		{
			continue;
		}
		const double from = i < fromQ.size() ? fromQ[i] : set.axes[i].home;
		const double to = i < toQ.size() ? toQ[i] : set.axes[i].home;
		const double d = to - from;
		if (set.axes[i].motionType == RobotExternal::RobotExternalMotionType::Rotate)
		{
			maxT = std::max(maxT, trapezoidTravelDurationSec(d, kRotateVRadPerSec, kRotateARadPerSec2));
		}
		else
		{
			maxT = std::max(maxT, trapezoidTravelDurationSec(d, kTranslateVMmPerSec, kTranslateAMmPerSec2));
		}
	}
	return maxT;
}

void fillPlanExternalAxisFromInstructionExt(RobotInstruction::PlanResult& plan,
											const std::unordered_map<std::string, std::string>& ext,
											const RobotExternal::RobotExternalAxisConfigSet* setOpt = nullptr)
{
	const auto itCsv = ext.find(RobotExternal::kExtContextExternalAxisQCsv);
	if (itCsv != ext.end() && !itCsv->second.empty())
	{
		plan.externalAxisQs = RobotExternal::parseExternalAxisQCsv(itCsv->second);
		if (setOpt)
		{
			plan.externalAxisQs = padExternalAxisQToConfig(*setOpt, plan.externalAxisQs);
		}
		plan.hasExternalAxisQ = !plan.externalAxisQs.empty();
		if (plan.hasExternalAxisQ)
		{
			plan.externalAxisQ =
				setOpt ? firstRobotBaseEnabledQ(*setOpt, plan.externalAxisQs, plan.externalAxisQs.front())
					   : plan.externalAxisQs.front();
		}
		return;
	}
	const auto itQ = ext.find(RobotExternal::kExtContextExternalAxisQMm);
	if (itQ == ext.end() || itQ->second.empty())
	{
		return;
	}
	bool ok = false;
	const double qe = QString::fromStdString(itQ->second).toDouble(&ok);
	if (!ok)
	{
		return;
	}
	plan.hasExternalAxisQ = true;
	plan.externalAxisQ = qe;
	if (setOpt && !setOpt->axes.empty())
	{
		plan.externalAxisQs = expandScalarExternalAxisQ(*setOpt, qe);
	}
	else
	{
		plan.externalAxisQs = {qe};
	}
}

enum class CoordinateFrameChangeKind
{
	StructuralOnly,
	ActiveToolChanged,
	ToolGeometryChanged,
};

const RobotCoordinate::RobotToolFrame* findToolFrameByIdInSet(const RobotCoordinate::RobotCoordinateFrameSet& set,
															  const std::string& id)
{
	for (const RobotCoordinate::RobotToolFrame& tf : set.toolFrames)
	{
		if (tf.id == id)
		{
			return &tf;
		}
	}
	return nullptr;
}

bool toolFrameGeometryMatches(const RobotCoordinate::RobotToolFrame& a, const RobotCoordinate::RobotToolFrame& b)
{
	return RobotCoordinate::encodeMat4Csv(RobotCoordinate::frameToMat4(a.T_flange_tool)) ==
			   RobotCoordinate::encodeMat4Csv(RobotCoordinate::frameToMat4(b.T_flange_tool)) &&
		   a.flangeLinkName == b.flangeLinkName;
}

bool rigidFrameMatches(const RobotCoordinate::RobotRigidFrame& a, const RobotCoordinate::RobotRigidFrame& b)
{
	for (int i = 0; i < 3; ++i)
	{
		if (std::abs(a.positionMm[i] - b.positionMm[i]) > 1e-6 || std::abs(a.eulerDeg[i] - b.eulerDeg[i]) > 1e-6)
		{
			return false;
		}
	}
	return true;
}

bool toolFrameEntryMatches(const RobotCoordinate::RobotToolFrame& a, const RobotCoordinate::RobotToolFrame& b)
{
	return a.id == b.id && a.name == b.name && a.showInScene == b.showInScene && toolFrameGeometryMatches(a, b);
}

bool userFrameEntryMatches(const RobotCoordinate::RobotUserFrame& a, const RobotCoordinate::RobotUserFrame& b)
{
	return a.id == b.id && a.name == b.name && a.showInScene == b.showInScene &&
		   rigidFrameMatches(a.T_base_user, b.T_base_user);
}

bool coordinateFrameSetEquals(const RobotCoordinate::RobotCoordinateFrameSet& a,
							  const RobotCoordinate::RobotCoordinateFrameSet& b)
{
	if (a.flangeLinkName != b.flangeLinkName || a.activeToolFrameId != b.activeToolFrameId ||
		a.activeUserFrameId != b.activeUserFrameId || a.showToolFrameInScene != b.showToolFrameInScene ||
		a.showUserFramesInScene != b.showUserFramesInScene || a.toolFrames.size() != b.toolFrames.size() ||
		a.userFrames.size() != b.userFrames.size())
	{
		return false;
	}
	for (size_t i = 0; i < a.toolFrames.size(); ++i)
	{
		if (!toolFrameEntryMatches(a.toolFrames[i], b.toolFrames[i]))
		{
			return false;
		}
	}
	for (size_t i = 0; i < a.userFrames.size(); ++i)
	{
		if (!userFrameEntryMatches(a.userFrames[i], b.userFrames[i]))
		{
			return false;
		}
	}
	return true;
}

bool coordinateFrameSetPlanningEquals(const RobotCoordinate::RobotCoordinateFrameSet& a,
									  const RobotCoordinate::RobotCoordinateFrameSet& b)
{
	RobotCoordinate::RobotCoordinateFrameSet aa = a;
	RobotCoordinate::RobotCoordinateFrameSet bb = b;
	aa.showToolFrameInScene = bb.showToolFrameInScene;
	aa.showUserFramesInScene = bb.showUserFramesInScene;
	for (size_t i = 0; i < aa.toolFrames.size() && i < bb.toolFrames.size(); ++i)
	{
		aa.toolFrames[i].showInScene = bb.toolFrames[i].showInScene;
	}
	for (size_t i = 0; i < aa.userFrames.size() && i < bb.userFrames.size(); ++i)
	{
		aa.userFrames[i].showInScene = bb.userFrames[i].showInScene;
	}
	return coordinateFrameSetEquals(aa, bb);
}

CoordinateFrameChangeKind classifyCoordinateFrameChange(const RobotCoordinate::RobotCoordinateFrameSet& oldFrames,
														const RobotCoordinate::RobotCoordinateFrameSet& newFrames)
{
	if (oldFrames.activeToolFrameId != newFrames.activeToolFrameId)
	{
		return CoordinateFrameChangeKind::ActiveToolChanged;
	}
	const RobotCoordinate::RobotToolFrame* oldActive = findToolFrameByIdInSet(oldFrames, oldFrames.activeToolFrameId);
	const RobotCoordinate::RobotToolFrame* newActive = findToolFrameByIdInSet(newFrames, newFrames.activeToolFrameId);
	if (oldActive && newActive && !toolFrameGeometryMatches(*oldActive, *newActive))
	{
		return CoordinateFrameChangeKind::ToolGeometryChanged;
	}
	for (const RobotCoordinate::RobotToolFrame& nt : newFrames.toolFrames)
	{
		const RobotCoordinate::RobotToolFrame* ot = findToolFrameByIdInSet(oldFrames, nt.id);
		if (ot && !toolFrameGeometryMatches(*ot, nt))
		{
			return CoordinateFrameChangeKind::ToolGeometryChanged;
		}
	}
	return CoordinateFrameChangeKind::StructuralOnly;
}

int changedJointCount(const QVector<double>& a, const QVector<double>& b, const double eps = 1e-9)
{
	const int n = std::min(a.size(), b.size());
	int changed = 0;
	for (int i = 0; i < n; ++i)
	{
		if (std::abs(a[i] - b[i]) > eps)
		{
			++changed;
		}
	}
	return changed;
}

void wrapJointAnglesTowardSeed(QVector<double>& q, const QVector<double>& seed)
{
	if (q.size() != seed.size())
	{
		return;
	}
	for (int j = 0; j < q.size(); ++j)
	{
		double d = q[j] - seed[j];
		while (d > kPi)
		{
			q[j] -= 2.0 * kPi;
			d = q[j] - seed[j];
		}
		while (d < -kPi)
		{
			q[j] += 2.0 * kPi;
			d = q[j] - seed[j];
		}
	}
}

QVector<double> clampJointStepFromPrevious(const QVector<double>& target, const QVector<double>& previous,
										   const double maxStepRad)
{
	QVector<double> out = target;
	if (previous.size() != target.size() || maxStepRad <= 0.0)
	{
		return out;
	}
	for (int j = 0; j < target.size(); ++j)
	{
		const double d = target[j] - previous[j];
		if (std::abs(d) > maxStepRad)
		{
			out[j] = previous[j] + std::copysign(maxStepRad, d);
		}
	}
	return out;
}

double maxJointDeltaRad(const QVector<double>& a, const QVector<double>& b)
{
	if (a.size() != b.size())
	{
		return 0.0;
	}
	double m = 0.0;
	for (int j = 0; j < a.size(); ++j)
	{
		m = std::max(m, std::abs(a[j] - b[j]));
	}
	return m;
}

bool instructionTcpWorldMat4FromTaughtJoints(IRobotDocumentHost* doc, int instIdx, const RobotInstruction::Base& ins,
											 const QVector<double>& taughtQ, osg::Matrixd& outTcpWorld);

double targetResidualMmForInstruction(const QString& urdfPath, const QVector<double>& jointQ,
									  const RobotCoordinate::RobotCoordinateFrameSet& frames,
									  const QString& fallbackFlangeLink, const RobotInstruction::Base& ins)
{
	engine::RigidTransform target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(ins, target))
	{
		return -1.0;
	}
	BackendMat4 fkTargetMat = BackendMat4::identity();
	QString resolvedFlangeLink;
	if (!RobotSimulationMath::targetInBaseFromUrdfFlangeFk(urdfPath, jointQ, frames, fallbackFlangeLink, fkTargetMat,
														   &ins, &resolvedFlangeLink))
	{
		return -1.0;
	}
	const engine::RigidTransform fkTarget = RobotCoordinate::rigidTransformFromBackendMat4(fkTargetMat);
	Eigen::Vector3d a = target.translationMm();
	Eigen::Vector3d b = fkTarget.translationMm();
	// 启用外轴时：TCP_eff = FK + q_e * axis
	const auto& ext = ins.extensionProperties();
	const auto itQ = ext.find(RobotExternal::kExtContextExternalAxisQMm);
	if (itQ != ext.end() && !itQ->second.empty())
	{
		const double qe = QString::fromStdString(itQ->second).toDouble();
		double axis[3] = {1.0, 0.0, 0.0};
		const auto itDir = ext.find(RobotExternal::kExtContextExternalAxisDir);
		if (itDir != ext.end() && !itDir->second.empty())
		{
			const QStringList parts = QString::fromStdString(itDir->second).split(QLatin1Char(','));
			if (parts.size() >= 3)
			{
				axis[0] = parts[0].toDouble();
				axis[1] = parts[1].toDouble();
				axis[2] = parts[2].toDouble();
			}
		}
		b += Eigen::Vector3d(qe * axis[0], qe * axis[1], qe * axis[2]);
	}
	return (a - b).norm();
}

void fillWorkpieceIkFrameContext(RobotInstruction::Controller& ctrl, IRobotDocumentHost* doc, const int instIdx)
{
	ctrl.clearWorkpieceIkFrameContext();
	if (!doc || instIdx < 0)
	{
		return;
	}
	const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instIdx);
	if (!RobotExternal::hasEnabledWorkpieceExternalAxes(set))
	{
		return;
	}
	const std::string boundId = RobotExternal::primaryWorkpieceBackendId(set);
	if (boundId.empty())
	{
		return;
	}
	const QString boundQ = QString::fromStdString(boundId);
	const QString workFrameQ = QString::fromStdString(RobotExternal::resolveWorkingFrameId(set));
	if (IRobotBackendPoseSink* sink = doc->poseSink())
	{
		cloudsim::core::Mat4 curBound = cloudsim::core::PlanContextDto::identityMat4();
		if (sink->getBackendRootWorldMatrix(boundId, curBound))
		{
			const std::vector<double> qs = doc->robotExternalAxisQ(instIdx);
			cloudsim::core::Mat4 w0Cand = curBound;
			RobotExternal::unbakeWorkpiecePlacementExternalAxis(curBound.data(), set, boundId, qs, w0Cand.data());
			doc->ensureWorkpieceExternalBasePlacement(instIdx, boundQ, w0Cand);
		}
		if (!workFrameQ.isEmpty() && workFrameQ != boundQ)
		{
			cloudsim::core::Mat4 workWorld = cloudsim::core::PlanContextDto::identityMat4();
			if (sink->getBackendRootWorldMatrix(workFrameQ.toStdString(), workWorld))
			{
				doc->ensureWorkpieceWorkingFrameOffset(instIdx, boundQ, workFrameQ, workWorld);
			}
		}
		else
		{
			doc->ensureWorkpieceWorkingFrameOffset(instIdx, boundQ, boundQ,
												  cloudsim::core::PlanContextDto::identityMat4());
		}
	}

	RobotInstruction::Controller::WorkpieceIkFrameContext ctx;
	ctx.valid = true;
	ctx.boundBackendId = boundId;
	const cloudsim::core::Mat4 p0 = doc->robotBasePlacementWorldForInstance(instIdx);
	const cloudsim::core::Mat4 w0 = doc->workpieceExternalBasePlacement(instIdx, boundQ);
	const cloudsim::core::Mat4 offset = doc->workpieceWorkingFrameOffset(instIdx, boundQ);
	for (int i = 0; i < 16; ++i)
	{
		ctx.p0World[static_cast<size_t>(i)] = p0[static_cast<size_t>(i)];
		ctx.w0World[static_cast<size_t>(i)] = w0[static_cast<size_t>(i)];
		ctx.offsetW0Local[static_cast<size_t>(i)] = offset[static_cast<size_t>(i)];
	}
	ctrl.setWorkpieceIkFrameContext(ctx);
}

void syncInstructionControllerExternalAxes(RobotInstruction::Controller& ctrl, IRobotDocumentHost* doc,
										   const int instIdx)
{
	if (!doc || instIdx < 0)
	{
		ctrl.clearExternalAxes();
		ctrl.clearWorkpieceIkFrameContext();
		return;
	}
	ctrl.setExternalAxes(doc->robotExternalAxesForInstance(instIdx));
	fillWorkpieceIkFrameContext(ctrl, doc, instIdx);
}

void writeExternalAxisPlanToInstruction(RobotInstruction::Base& ins, const RobotInstruction::PlanResult& plan,
										IRobotDocumentHost* doc, const int instIdx)
{
	if (!plan.hasExternalAxisQ)
	{
		// 计划无外轴结果时保留指令已有示教/求解值，避免静默擦除
		return;
	}
	const RobotExternal::RobotExternalAxisConfigSet* setPtr = nullptr;
	if (doc && instIdx >= 0)
	{
		setPtr = &doc->robotExternalAxesForInstance(instIdx);
	}
	double qScalar = plan.externalAxisQ;
	if (!plan.externalAxisQs.empty())
	{
		ins.setExtensionProperty(RobotExternal::kExtContextExternalAxisQCsv,
								 RobotExternal::encodeExternalAxisQCsv(plan.externalAxisQs));
		if (setPtr)
		{
			qScalar = firstRobotBaseEnabledQ(*setPtr, plan.externalAxisQs, plan.externalAxisQ);
		}
		else
		{
			qScalar = plan.externalAxisQs.front();
		}
	}
	else
	{
		ins.eraseExtensionProperty(RobotExternal::kExtContextExternalAxisQCsv);
	}
	ins.setExtensionProperty(RobotExternal::kExtContextExternalAxisQMm, QString::number(qScalar, 'g', 12).toStdString());
	if (!setPtr)
	{
		return;
	}
	if (const RobotExternal::RobotExternalAxisConfig* rail = RobotExternal::firstEnabledExternalAxis(*setPtr))
	{
		const QString dir = QStringLiteral("%1,%2,%3")
								.arg(rail->axis[0], 0, 'g', 12)
								.arg(rail->axis[1], 0, 'g', 12)
								.arg(rail->axis[2], 0, 'g', 12);
		ins.setExtensionProperty(RobotExternal::kExtContextExternalAxisDir, dir.toStdString());
	}
}

double targetOrientationResidualDegForInstruction(const QString& urdfPath, const QVector<double>& jointQ,
												  const RobotCoordinate::RobotCoordinateFrameSet& frames,
												  const QString& fallbackFlangeLink, const RobotInstruction::Base& ins)
{
	engine::RigidTransform target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(ins, target))
	{
		return -1.0;
	}
	BackendMat4 fkTargetMat = BackendMat4::identity();
	if (!RobotSimulationMath::targetInBaseFromUrdfFlangeFk(urdfPath, jointQ, frames, fallbackFlangeLink, fkTargetMat,
														   &ins, nullptr))
	{
		return -1.0;
	}
	const engine::RigidTransform fkTarget = RobotCoordinate::rigidTransformFromBackendMat4(fkTargetMat);
	const Eigen::Quaterniond qErr = target.rotation().inverse() * fkTarget.rotation();
	const double w = std::clamp(std::abs(qErr.w()), 0.0, 1.0);
	return 2.0 * std::acos(w) * (180.0 / kPi);
}

QVector<double> clampJointAnglesToInstanceLimits(IRobotDocumentHost* doc, const int instIdx,
												 const QVector<double>& jointAnglesRad)
{
	QVector<double> out = jointAnglesRad;
	if (!doc || instIdx < 0)
	{
		return out;
	}
	QVector<double> lower;
	QVector<double> upper;
	doc->robotJointLimitsForInstance(instIdx, lower, upper);
	const int n = std::min(out.size(), std::min(lower.size(), upper.size()));
	for (int j = 0; j < n; ++j)
	{
		out[j] = std::clamp(out[j], lower[j], upper[j]);
	}
	return out;
}

/// Lookahead/worker 入库前：R1 + 轨迹对齐；失败则标记不可缓存
bool prepareWorkerPlanForCache(RobotInstruction::PlanResult& plan, const QVector<double>& chainSeedQ,
							   const QVector<double>& limLo, const QVector<double>& limHi)
{
	if (!plan.ok || plan.jointTargetsRad.empty())
	{
		return false;
	}
	if (chainSeedQ.size() != static_cast<int>(plan.jointTargetsRad.size()))
	{
		return true;
	}
	std::vector<double> refQ(chainSeedQ.begin(), chainSeedQ.end());
	return applyJointWrapToPlan(plan, refQ, limLo, limHi);
}

double maxUnfoldedJointAbsDelta(const QVector<double>& a, const QVector<double>& b)
{
	if (a.size() != b.size())
	{
		return 1e30;
	}
	double maxAbs = 0.0;
	for (int j = 0; j < a.size(); ++j)
	{
		maxAbs = std::max(maxAbs, std::abs(a[j] - b[j]));
	}
	return maxAbs;
}

/// 折圈后逐关节最大偏差：用于连续性硬门（区分「分支跨越」与「整圈跳转」）
/// 分支跨越（如手腕翻转）折圈后差值小，应通过；整圈跳转折圈后差为 0，也应通过
/// 排序仍用未折圈 SSE，优先选物理行程短的解
double maxFoldedJointAbsDelta(const QVector<double>& a, const QVector<double>& b)
{
	if (a.size() != b.size())
	{
		return 1e30;
	}
	constexpr double kTwoPi = 6.283185307179586;
	double maxAbs = 0.0;
	for (int j = 0; j < a.size(); ++j)
	{
		const double folded = a[j] - kTwoPi * std::round((a[j] - b[j]) / kTwoPi);
		maxAbs = std::max(maxAbs, std::abs(folded - b[j]));
	}
	return maxAbs;
}

double unfoldedJointSse(const QVector<double>& a, const QVector<double>& b)
{
	if (a.size() != b.size())
	{
		return 1e30;
	}
	double sse = 0.0;
	for (int j = 0; j < a.size(); ++j)
	{
		const double d = a[j] - b[j];
		sse += d * d;
	}
	return sse;
}
} // namespace

namespace InstructionPoseDiagState
{
void requestRefresh() {}
bool shouldLog(const std::string&)
{
	return false;
}
} // namespace InstructionPoseDiagState

RobotSimulationController::RobotSimulationController(QObject* parent)
	: QObject(parent), m_collisionWorld(std::make_unique<collision::CollisionWorld>())
{
	m_ioNetwork = new IoSignalNetworkService(this);
}

RobotSimulationController::~RobotSimulationController()
{
	if (m_robotCommPollTimer)
	{
		m_robotCommPollTimer->stop();
	}
	if (m_robotCommClient)
	{
		m_robotCommClient->disconnectRobot();
		m_robotCommClient->disconnectBridge();
	}
}

QJsonObject RobotSimulationController::ioSignalNetworkToJson() const
{
	return m_ioNetwork ? m_ioNetwork->toProjectJson() : QJsonObject();
}

bool RobotSimulationController::ioSignalNetworkFromJson(const QJsonObject& root, QString* outError)
{
	if (!m_ioNetwork)
	{
		return false;
	}
	if (!m_ioNetwork->fromProjectJson(root, outError))
	{
		return false;
	}
	if (m_host && m_host->document())
	{
		m_ioNetwork->syncOwnersFromDocument(m_host->document());
	}
	return true;
}

void RobotSimulationController::setIoUiOwnerId(const QString& ownerId)
{
	m_ioUiOwnerId = ownerId;
}

NamedSignalIoSink* RobotSimulationController::simulationIoSinkForOwner(const QString& ownerId)
{
	return m_ioNetwork ? m_ioNetwork->sink(ownerId) : nullptr;
}

NamedSignalIoSink* RobotSimulationController::simulationIoSink()
{
	return simulationIoSinkForOwner(m_ioUiOwnerId);
}

RobotIo::NamedSignalTable* RobotSimulationController::namedSignalTableForOwner(const QString& ownerId)
{
	return m_ioNetwork ? m_ioNetwork->table(ownerId) : nullptr;
}

const RobotIo::NamedSignalTable* RobotSimulationController::namedSignalTableForOwner(const QString& ownerId) const
{
	return m_ioNetwork ? m_ioNetwork->table(ownerId) : nullptr;
}

RobotIo::NamedSignalTable& RobotSimulationController::namedSignalTable()
{
	if (RobotIo::NamedSignalTable* t = namedSignalTableForOwner(m_ioUiOwnerId))
	{
		return *t;
	}
	static RobotIo::NamedSignalTable sEmpty;
	return sEmpty;
}

const RobotIo::NamedSignalTable& RobotSimulationController::namedSignalTable() const
{
	if (m_ioNetwork)
	{
		if (const RobotIo::NamedSignalTable* t = m_ioNetwork->table(m_ioUiOwnerId))
		{
			return *t;
		}
	}
	static RobotIo::NamedSignalTable sEmpty;
	return sEmpty;
}

QStringList RobotSimulationController::ioSignalNamesForOwner(const QString& ownerId,
															 const RobotIo::SignalKind kind) const
{
	QStringList out;
	if (!m_ioNetwork)
	{
		return out;
	}
	const RobotIo::NamedSignalTable* table = m_ioNetwork->table(ownerId);
	if (!table)
	{
		return out;
	}
	for (const RobotIo::SignalDef& s : table->entries())
	{
		if (s.kind == kind && !s.name.empty())
		{
			out << QString::fromStdString(s.name);
		}
	}
	return out;
}

QStringList RobotSimulationController::ioSignalNames(const RobotIo::SignalKind kind) const
{
	return ioSignalNamesForOwner(m_ioUiOwnerId, kind);
}

void RobotSimulationController::setIoSinkBackend(const RobotIoSinkBackend backend)
{
	if (m_ioNetwork)
	{
		m_ioNetwork->setIoSinkBackend(backend);
	}
}

void RobotSimulationController::syncIoOwnersFromDocument()
{
	if (m_ioNetwork && m_host)
	{
		m_ioNetwork->syncOwnersFromDocument(m_host->document());
	}
}

void RobotSimulationController::flushDeviceIoTablesToDocument()
{
	if (m_ioNetwork && m_host)
	{
		m_ioNetwork->flushDeviceTablesToDocument(m_host->document());
	}
}


void RobotSimulationController::setHost(IRobotMainWindowHost* host)
{
	m_host = host;
	if (m_customDeviceSim)
	{
		m_customDeviceSim->setHost(host);
	}
	if (m_axisControlTargets)
	{
		m_axisControlTargets->setHost(host);
	}
}

void RobotSimulationController::initializePlanners()
{
	m_instructionController.buildDefaultPlanners();
}

void RobotSimulationController::createSimulationDock(QWidget* parentForTabs)
{
	m_simulationDock = new RobotSimulationDockWidget(parentForTabs);
	m_programEditService = new ProgramEditService(this);
	m_trajectoryEditSession = new TrajectoryEditSession(this);
	m_customDeviceSim = new CustomDeviceSimService(this);
	m_dockNavigation = new DockNavigationService(this);
	m_dockNavigation->setDock(m_simulationDock);
	m_axisControlTargets = new AxisControlTargetService(this);
	m_axisControlTargets->setSimulationController(this);
	connect(m_axisControlTargets, &AxisControlTargetService::catalogChanged, this,
			&RobotSimulationController::customDeviceCatalogChanged);
}

DevicePoseMotionPlayer* RobotSimulationController::devicePoseMotionPlayer() const
{
	return m_customDeviceSim ? m_customDeviceSim->motionPlayer() : nullptr;
}

DevicePoseSignalDriver* RobotSimulationController::devicePoseSignalDriver() const
{
	return m_customDeviceSim ? m_customDeviceSim->signalDriver() : nullptr;
}

void RobotSimulationController::wireSimulationSignals()
{
	if (!m_host || !m_simulationDock)
	{
		return;
	}
	auto* cmd = m_simulationDock->commandPage();
	auto* axis = m_simulationDock->axisPage();
	auto* frame = m_simulationDock->framePage();
	auto* traj = m_simulationDock->trajectoryEditPage();
	if (m_host && m_host->document())
	{
		m_programEditService->bindStore(&m_host->document()->robotProgramStore());
	}
	m_trajectoryEditSession->bindEditService(m_programEditService);
	m_trajectoryEditSession->bindSimulationController(this);
	if (traj)
	{
		traj->bindEditService(m_programEditService);
		traj->bindSession(m_trajectoryEditSession);
		traj->bindCommandPage(cmd);
		traj->bindHost(m_host);
		traj->bindSimulationController(this);
	}
	if (TrajectoryGenerationPageWidget* gen = m_simulationDock->trajectoryGenerationPage())
	{
		gen->bindSession(m_trajectoryEditSession);
		gen->bindSimulationController(this);
		if (m_host && m_host->document())
		{
			gen->bindStore(&m_host->document()->robotProgramStore());
		}
		gen->bindEditService(m_programEditService);
		gen->bindCommandPage(cmd);
	}
	if (cmd && cmd->instructionTree())
	{
		cmd->instructionTree()->setGroupVisibilityQuery([this](const std::string& groupId)
														{ return isInstructionGroupVisible(groupId); });
		connect(cmd->instructionTree(), &InstructionProgramTreeWidget::groupVisibilityChangeRequested, this,
				&RobotSimulationController::onInstructionGroupVisibilityChangeRequested);
	}
	if (cmd)
	{
		cmd->setProgramEditService(m_programEditService);
		connect(cmd, &SimulationCommandWidget::runRequested, this, &RobotSimulationController::onSimulationRunRequested);
		connect(cmd, &SimulationCommandWidget::stopRequested, this, &RobotSimulationController::onSimulationStopRequested);
		connect(cmd, &SimulationCommandWidget::exportProgramRequested, this,
				&RobotSimulationController::onSimulationExportRequested);
		connect(cmd, &SimulationCommandWidget::playbackRateChanged, this,
				[this](const double rate) { m_programExecutor.setPlaybackRate(rate); });
		m_programExecutor.setPlaybackRate(cmd->playbackRate());
		connect(cmd, &SimulationCommandWidget::ikSeedPolicyChanged, this,
				[this](const int policy)
				{
					m_ikSeedPolicy = (policy == 1) ? RobotInstruction::IkSeedPolicy::FromCurrentPose
												   : RobotInstruction::IkSeedPolicy::FromInstruction;
					m_ikSeedInstructionId.clear();
					m_planResultCache.invalidateAll();
				});
	}
	connect(cmd, &SimulationCommandWidget::addInstructionRequested, this,
			&RobotSimulationController::onSimulationAddInstructionRequested);
	connect(cmd, &SimulationCommandWidget::instructionSelectionChanged, this,
			&RobotSimulationController::onSimulationInstructionSelectionChanged);
	connect(cmd, &SimulationCommandWidget::robotSelectionChanged, this,
			&RobotSimulationController::onSimulationRobotSelectionChanged);
	connect(cmd, &SimulationCommandWidget::tcpDragTeachModeChanged, this,
			&RobotSimulationController::onSimulationTcpDragTeachModeChanged);
	connect(cmd, &SimulationCommandWidget::instructionWaypointPickModeChanged, this,
			&RobotSimulationController::onInstructionWaypointPickModeChanged);
	// 启动时可能尚无 DocumentPage；进入拾取模式时会再绑到当前 OsgWidget
	bindInstructionWaypointPickCallbacks();
	connect(axis, &RobotAxisControlWidget::allJointAnglesChanged, this,
			&RobotSimulationController::onRobotAxisJointAnglesChanged);
	connect(axis, &RobotAxisControlWidget::externalAxisValuesChanged, this,
			&RobotSimulationController::onRobotAxisExternalValuesChanged);
	connect(axis, &RobotAxisControlWidget::controlTargetChanged, this,
			&RobotSimulationController::onAxisControlTargetChanged);
	connect(axis, &RobotAxisControlWidget::reachableWorkspaceToggled, this,
			&RobotSimulationController::onReachableWorkspaceToggled);
	connect(axis, &RobotAxisControlWidget::reachableWorkspaceDensityChanged, this,
			&RobotSimulationController::onReachableWorkspaceDensityChanged);
	connect(frame, &RobotFrameSettingsWidget::framesChanged, this,
			&RobotSimulationController::onRobotCoordinateFramesChanged);
	connect(frame, &RobotFrameSettingsWidget::captureToolFromTcpRequested, this,
			&RobotSimulationController::onCaptureToolFrameFromTcp);
	connect(frame, &RobotFrameSettingsWidget::captureUserFrameFromTcpRequested, this,
			&RobotSimulationController::onCaptureUserFrameFromTcp);
	connect(frame, &RobotFrameSettingsWidget::resetToolFrameRequested, this,
			&RobotSimulationController::onResetToolFrame);
	if (RobotExternalAxisSettingsWidget* ext = m_simulationDock->externalAxisPage())
	{
		connect(ext, &RobotExternalAxisSettingsWidget::externalAxesChanged, this,
				&RobotSimulationController::onRobotExternalAxesChanged);
	}
	if (RobotCollisionSettingsWidget* col = m_simulationDock->collisionPage())
	{
		connect(col, &RobotCollisionSettingsWidget::settingsChanged, this,
				&RobotSimulationController::onRobotCollisionSettingsChanged);
		connect(col, &RobotCollisionSettingsWidget::planRequested, this,
				&RobotSimulationController::onMotionPathPlanRequested);
		connect(col, &RobotCollisionSettingsWidget::clearPreviewRequested, this,
				&RobotSimulationController::onMotionPathPreviewClearRequested);
		connect(col, &RobotCollisionSettingsWidget::confirmTrajectoryRequested, this,
				&RobotSimulationController::onMotionPathConfirmTrajectoryRequested);
		connect(col, &RobotCollisionSettingsWidget::refreshSceneObjectsRequested, this,
				&RobotSimulationController::refreshCollisionPageSceneObjects);
	}
	connect(m_trajectoryEditSession, &TrajectoryEditSession::pathPlanBound, this,
			[this](const std::string&) { refreshPathPlanPreviewForActiveTab(); });
	connect(m_trajectoryEditSession, &TrajectoryEditSession::rawTrajectoryChanged, this,
			[this]() { refreshPathPlanPreviewForActiveTab(); });
	if (QTabWidget* tabs = m_simulationDock->robotTabWidget())
	{
		connect(tabs, &QTabWidget::currentChanged, this, &RobotSimulationController::onSimulationDockTabChanged);
	}
	if (QTabWidget* deviceTabs = m_simulationDock->deviceTabWidget())
	{
		connect(deviceTabs, &QTabWidget::currentChanged, this, &RobotSimulationController::onSimulationDockTabChanged);
	}
	connect(m_simulationDock, &RobotSimulationDockWidget::dockModeChanged, this,
			[this](SimulationDockMode mode)
			{
				refreshPathPlanPreviewForActiveTab();
				syncAxisControlTargetForDockMode(mode);
			});
	if (m_programEditService)
	{
		connect(m_programEditService, &ProgramEditService::revisionChanged, this,
				[this](int)
				{
					m_planResultCache.invalidateAll();
					invalidateChainSeedRollCache();
				});
	}
	if (RobotCommPageWidget* comm = m_simulationDock->robotCommPage())
	{
		if (m_host)
		{
			comm->setUseChinese(m_host->useChinese());
		}
		connect(comm, &RobotCommPageWidget::connectRequested, this,
				&RobotSimulationController::onRobotCommConnectRequested);
		connect(comm, &RobotCommPageWidget::disconnectRequested, this,
				&RobotSimulationController::onRobotCommDisconnectRequested);
		connect(comm, &RobotCommPageWidget::mirrorToggled, this,
				&RobotSimulationController::onRobotCommMirrorToggled);
		connect(comm, &RobotCommPageWidget::pollIntervalChanged, this,
				&RobotSimulationController::onRobotCommPollIntervalChanged);
		if (!m_robotCommPollTimer)
		{
			m_robotCommPollTimer = new QTimer(this);
			m_robotCommPollTimer->setTimerType(Qt::CoarseTimer);
			connect(m_robotCommPollTimer, &QTimer::timeout, this, &RobotSimulationController::onRobotCommPollTick);
		}
		m_robotCommPollTimer->setInterval(comm->pollIntervalMs());
	}

	if (m_axisControlTargets)
	{
		m_axisControlTargets->setHost(m_host);
	}
	if (m_customDeviceSim)
	{
		m_customDeviceSim->wire(m_host, m_ioNetwork,
								m_simulationDock ? m_simulationDock->deviceCommandPage() : nullptr);
	}
	if (DeviceCommandPageWidget* deviceCmd = m_simulationDock->deviceCommandPage())
	{
		connect(this, &RobotSimulationController::customDeviceCatalogChanged, deviceCmd,
				&DeviceCommandPageWidget::refreshDevices);
	}
}

void RobotSimulationController::attachPlaybackTimer(QTimer* externalTimer)
{
	m_playbackTimer = externalTimer;
	m_ownsPlaybackTimer = false;
	if (m_playbackTimer)
	{
		// CoarseTimer：回放不必 Precise，省唤醒开销
		m_playbackTimer->setTimerType(Qt::CoarseTimer);
		connect(m_playbackTimer, &QTimer::timeout, this, &RobotSimulationController::onRobotSimulationTick);
	}
}

bool RobotSimulationController::isPlaybackUiInteractionBusy()
{
	return QGuiApplication::mouseButtons() != Qt::NoButton || QApplication::activePopupWidget() != nullptr ||
		   QApplication::activeModalWidget() != nullptr;
}

void RobotSimulationController::invalidateChainSeedRollCache()
{
	m_chainSeedRollFingerprint.clear();
	m_chainSeedEndJointsByIndex.clear();
}

void RobotSimulationController::onUrdfImportRequested(const QString& urdfPath)
{
	if (m_host)
	{
		m_host->registerUrdfRobot(urdfPath, false);
	}
}
void RobotSimulationController::syncRobotKinematicsAfterPoseEdit(const QString& backendId)
{
	if (backendId.isEmpty() || !m_host)
	{
		return;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc)
	{
		return;
	}
	BackendDataManager* mgr = doc->robotBackendManagerForKinematics();
	if (!mgr)
	{
		return;
	}
	const std::shared_ptr<BackendDataBase> data = mgr->getData(backendId.toStdString());
	syncRobotKinematicsAfterPoseEdit(data);
}

void RobotSimulationController::syncRobotKinematicsAfterPoseEdit(const std::shared_ptr<BackendDataBase>& data)
{
	if (!data || !data->hasPoseProperty())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg)
	{
		return;
	}
	const QString backendId = QString::fromStdString(data->id());
	bool isSceneRoot = false;
	const int instIdx = doc->robotInstanceIndexForPerLinkBackend(backendId, &isSceneRoot);
	if (instIdx < 0)
	{
		return;
	}
	cloudsim::core::RobotPerLinkKinematicsSliceDto dto;
	if (!doc->robotPerLinkKinematicsForInstance(instIdx, dto))
	{
		return;
	}
	RobotPerLinkKinematicsSlice slice = RobotSceneKinematics::robotPerLinkSliceFromDto(dto);
	const BackendVec3 p = data->pose();
	const BackendVec3 r = data->rotation();
	const osg::Quat q = engine::eulerDegToQuat(r.x, r.y, r.z);
	const osg::Matrixd placement = ObjectGizmoFrame::outerLocalMatrix(
		osg::Vec3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)), q);

	if (isSceneRoot)
	{
		doc->setRobotBasePlacementWorldForInstance(instIdx, RobotSimulationMath::coreMat4FromOsgMatrix(placement));
		slice.robotBasePlacementWorld = placement;
		const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
		QVector<double> localAngles(nj, 0.0);
		int offset = 0;
		for (int i = 0; i < instIdx; ++i)
		{
			offset += doc->robotRevoluteJointCountForInstance(i);
		}
		const int totalJoints = doc->robotRevoluteJointNames().size();
		if (m_aggregatedJointAnglesRad.size() != totalJoints)
		{
			m_aggregatedJointAnglesRad.resize(totalJoints);
		}
		if (m_aggregatedJointAnglesRad.size() >= offset + nj)
		{
			for (int j = 0; j < nj; ++j)
			{
				localAngles[j] = m_aggregatedJointAnglesRad[offset + j];
			}
		}
		else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
		{
			localAngles = m_host->robotAxisControlPage()->jointAnglesRad();
		}
		if (RobotSceneKinematics::applyPerLinkRobotBasePlacement(doc->poseSink(), doc->backend(), slice, localAngles,
																 placement))
		{
			osg->requestRedraw();
		}
		m_host->invalidateInstructionPropertyCache();
		refreshInstructionPoseAxes();
		return;
	}

	osg::Matrixd world;
	if (RobotSimulationMath::getBackendRootWorldMatrixOsg(osg, data->id(), world))
	{
		doc->updateRobotLinkOuterBindFromWorld(instIdx, backendId, RobotSimulationMath::coreMat4FromOsgMatrix(world));
	}
}

void RobotSimulationController::stopRobotSimulation(const bool applyLastPoseToScene)
{
	cancelArcTeach();
	const QVector<double> lastJointAngles = m_programExecutor.jointAnglesRad();
	m_programExecutor.stop();
	if (m_playbackTimer)
	{
		m_playbackTimer->stop();
	}
	m_currentRunMotions.clear();
	m_lookaheadPendingJobs = 0;
	m_lookaheadInFlight.clear();
	m_lookaheadFailedFingerprints.clear();
	m_playbackHoldTimer.invalidate();
	m_lastHighlightedInstructionId.clear();
	m_playbackMotionIndex = 0;
	m_playbackRollingSeedQ.clear();
	m_playbackProgramStartQ.clear();
	m_playbackSegmentExternalAxisStart.clear();
	m_playbackExtInterpMotion = nullptr;
	m_playbackOverlayHighlight.reset();
	m_waypointLabelOverlayFingerprint.clear();
	if (SimulationCommandWidget* cmd = m_host ? m_host->simulationCommandPage() : nullptr)
	{
		std::shared_ptr<RobotInstruction::Base> sel;
		if (InstructionProgramTreeWidget* tree = cmd->instructionTree())
		{
			sel = tree->selectedInstruction();
		}
		refreshPlaybackPathOverlays(sel);
		cmd->setSimulationRunning(false);
	}
	else
	{
		refreshPlaybackPathOverlays(nullptr);
	}
	if (m_simulationDock && m_simulationDock->trajectoryEditPage())
	{
		m_simulationDock->trajectoryEditPage()->setReadOnly(false);
	}
	if (applyLastPoseToScene && m_host->robotAxisControlPage())
	{
		m_host->robotAxisControlPage()->setInteractionEnabled(true);
		IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
		const int instIdx =
			m_host->simulationCommandPage() ? m_host->simulationCommandPage()->currentRobotInstanceIndex() : 0;
		if (doc && doc->hasRobotSimulationContext() && instIdx >= 0 && !lastJointAngles.isEmpty())
		{
			const int offset = doc->robotJointOffsetInAggregatedVector(instIdx);
			const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
			if (nj > 0 && m_host->robotAxisControlPage()->jointCount() == nj && offset + nj <= lastJointAngles.size())
			{
				const QVector<double> local = lastJointAngles.mid(offset, nj);
				m_host->robotAxisControlPage()->setJointAnglesRad(local);
				if (m_aggregatedJointAnglesRad.size() == doc->robotRevoluteJointNames().size())
				{
					(void)doc->applyJointAnglesRad(instIdx, local, m_aggregatedJointAnglesRad);
				}
			}
		}
	}
	else if (m_host && m_host->robotAxisControlPage())
	{
		m_host->robotAxisControlPage()->setInteractionEnabled(true);
	}
	refreshInstructionPoseAxes();
	refreshRobotCoordinateFrameOverlays();
}

void RobotSimulationController::forgetDocumentJointUiState(const QString& documentId)
{
	if (documentId.isEmpty())
	{
		return;
	}
	m_jointUiByDocumentId.remove(documentId);
	if (m_boundDocumentId == documentId)
	{
		m_boundDocumentId.clear();
		m_aggregatedJointAnglesRad.clear();
		m_syncedRobotSceneBackendIds.clear();
	}
}

void RobotSimulationController::bindJointUiToDocument(IRobotDocumentHost* doc)
{
	const QString documentId = doc ? doc->documentId() : QString();
	if (documentId == m_boundDocumentId)
	{
		return;
	}
	if (!m_boundDocumentId.isEmpty())
	{
		DocumentJointUiState& stash = m_jointUiByDocumentId[m_boundDocumentId];
		stash.aggregatedJointAnglesRad = m_aggregatedJointAnglesRad;
		stash.syncedRobotSceneBackendIds = m_syncedRobotSceneBackendIds;
	}
	m_boundDocumentId = documentId;
	if (documentId.isEmpty())
	{
		m_aggregatedJointAnglesRad.clear();
		m_syncedRobotSceneBackendIds.clear();
		return;
	}
	const auto it = m_jointUiByDocumentId.constFind(documentId);
	if (it != m_jointUiByDocumentId.cend())
	{
		m_aggregatedJointAnglesRad = it->aggregatedJointAnglesRad;
		m_syncedRobotSceneBackendIds = it->syncedRobotSceneBackendIds;
		return;
	}
	m_aggregatedJointAnglesRad.clear();
	m_syncedRobotSceneBackendIds.clear();
	if (!doc || !doc->hasRobotSimulationContext())
	{
		return;
	}
	resyncAggregatedJointAnglesFromDocument(doc);
}

void RobotSimulationController::resyncAggregatedJointAnglesFromDocument(IRobotDocumentHost* doc)
{
	if (!doc || !doc->hasRobotSimulationContext())
	{
		m_aggregatedJointAnglesRad.clear();
		m_syncedRobotSceneBackendIds.clear();
		return;
	}

	const int n = doc->robotKinematicInstanceCount();
	QStringList backendIds;
	backendIds.reserve(n);
	for (int i = 0; i < n; ++i)
	{
		backendIds.append(doc->robotSceneBackendIdForInstance(i));
	}

	QHash<QString, QVector<double>> preservedByRoot;
	{
		int oldOff = 0;
		bool layoutTrusted = true;
		for (const QString& oldId : m_syncedRobotSceneBackendIds)
		{
			int oldInst = -1;
			for (int i = 0; i < n; ++i)
			{
				if (backendIds[i] == oldId)
				{
					oldInst = i;
					break;
				}
			}
			if (oldInst < 0)
			{
				// 中间删机后旧槽位长度未知，放弃前缀映射，改走文档局部角
				layoutTrusted = false;
				break;
			}
			const int nj = doc->robotRevoluteJointCountForInstance(oldInst);
			if (nj <= 0 || oldOff + nj > m_aggregatedJointAnglesRad.size())
			{
				layoutTrusted = false;
				break;
			}
			preservedByRoot.insert(oldId, m_aggregatedJointAnglesRad.mid(oldOff, nj));
			oldOff += nj;
		}
		if (!layoutTrusted)
		{
			preservedByRoot.clear();
		}
	}

	const int total = doc->robotRevoluteJointNames().size();
	QVector<double> next(total, 0.0);
	for (int i = 0; i < n; ++i)
	{
		const QString& root = backendIds[i];
		const int offset = doc->robotJointOffsetInAggregatedVector(i);
		const int nj = doc->robotRevoluteJointCountForInstance(i);
		if (nj <= 0 || offset + nj > total)
		{
			continue;
		}
		const auto pit = preservedByRoot.constFind(root);
		if (pit != preservedByRoot.cend() && pit->size() == nj)
		{
			for (int j = 0; j < nj; ++j)
			{
				next[offset + j] = (*pit)[j];
			}
			continue;
		}
		QVector<double> local;
		if (!doc->robotLocalJointAnglesForSceneRoot(root, local))
		{
			continue;
		}
		for (int j = 0; j < nj && j < local.size(); ++j)
		{
			next[offset + j] = local[j];
		}
	}

	m_aggregatedJointAnglesRad = std::move(next);
	m_syncedRobotSceneBackendIds = backendIds;
	if (!m_boundDocumentId.isEmpty())
	{
		DocumentJointUiState& stash = m_jointUiByDocumentId[m_boundDocumentId];
		stash.aggregatedJointAnglesRad = m_aggregatedJointAnglesRad;
		stash.syncedRobotSceneBackendIds = m_syncedRobotSceneBackendIds;
	}
}

void RobotSimulationController::refreshSimulationJointListFromCurrentDoc()
{
	if (!m_host->simulationCommandPage() || !m_host->robotAxisControlPage())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	bindJointUiToDocument(doc);
	if (doc)
	{
		m_host->simulationCommandPage()->setProgramStore(&doc->robotProgramStore());
	}
	if (m_programEditService && doc)
	{
		m_programEditService->bindStore(&doc->robotProgramStore());
	}
	if (m_trajectoryEditSession && doc)
	{
		m_trajectoryEditSession->bindStore(&doc->robotProgramStore());
	}
	if (m_simulationDock && m_simulationDock->trajectoryEditPage() && doc)
	{
		m_simulationDock->trajectoryEditPage()->bindStore(&doc->robotProgramStore());
		m_simulationDock->trajectoryEditPage()->refreshProgramAndGroupCombos();
	}
	if (m_simulationDock && m_simulationDock->trajectoryGenerationPage() && m_host)
	{
		auto* genPage = m_simulationDock->trajectoryGenerationPage();
		IRobotMainWindowHost* host = m_host;
		genPage->bindSession(m_trajectoryEditSession);
		genPage->bindSimulationController(this);
		if (doc)
		{
			genPage->bindStore(&doc->robotProgramStore());
		}
		genPage->bindEditService(m_programEditService);
		if (m_simulationDock->commandPage())
		{
			genPage->bindCommandPage(m_simulationDock->commandPage());
		}
		genPage->setStepPathResolver(
			[host](const QString& backendId) -> QString
			{
				IRobotDocumentHost* liveDoc = host ? host->document() : nullptr;
				return liveDoc ? liveDoc->meshBackendStepSourcePath(backendId) : QString();
			});
		genPage->bindHost(m_host);
	}
	if (m_simulationDock && m_simulationDock->trajectoryEditPage() && m_host)
	{
		m_simulationDock->trajectoryEditPage()->bindHost(m_host);
	}
	if (doc && doc->hasRobotSimulationContext())
	{
		QStringList labels;
		QStringList backendIds;
		const int n = doc->robotKinematicInstanceCount();
		for (int i = 0; i < n; ++i)
		{
			labels.append(doc->robotDisplayLabelForInstance(i));
			backendIds.append(doc->robotSceneBackendIdForInstance(i));
		}
		// 聚合角须先按 sceneBackendId 对齐；setRobotInstances 会立刻触发选中回调读聚合向量
		resyncAggregatedJointAnglesFromDocument(doc);
		m_host->simulationCommandPage()->setRobotInstances(labels, backendIds);
		refreshAxisControlTargets();
		if (m_customDeviceSim)
		{
			m_customDeviceSim->resetEdgeState();
		}
		if (m_simulationDock && m_simulationDock->deviceCommandPage())
		{
			m_simulationDock->deviceCommandPage()->refreshDevices();
		}

		const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
								? m_host->simulationCommandPage()->currentRobotInstanceIndex()
								: 0;
		m_host->simulationCommandPage()->setRevoluteJointNames(doc->robotRevoluteJointNamesForInstance(instIdx));

		QStringList tcpLinks;
		QString preferredTcp;
		cloudsim::core::RobotPerLinkKinematicsSliceDto plSlice;
		if (doc->robotPerLinkKinematicsForInstance(instIdx, plSlice) && !plSlice.linkNameToBackendId.isEmpty())
		{
			tcpLinks = plSlice.linkNameToBackendId.keys();
			tcpLinks.sort();
			const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
			preferredTcp = QString::fromStdString(frames.flangeLinkName);
			if (preferredTcp.isEmpty() && !tcpLinks.isEmpty())
			{
				preferredTcp = tcpLinks.back();
			}
		}
		else
		{
			const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
			(void)UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferredTcp, nullptr);
			QStringList childLinks;
			(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
			QSet<QString> uniq;
			if (!preferredTcp.isEmpty())
			{
				uniq.insert(preferredTcp);
				tcpLinks.push_back(preferredTcp);
			}
			for (const QString& l : childLinks)
			{
				if (l.isEmpty() || uniq.contains(l))
				{
					continue;
				}
				uniq.insert(l);
				tcpLinks.push_back(l);
			}
		}
		m_host->simulationCommandPage()->setTcpLinkOptions(tcpLinks, preferredTcp);

		QVector<double> lower;
		QVector<double> upper;
		doc->robotJointLimitsForInstance(instIdx, lower, upper);
		const QStringList jn = doc->robotRevoluteJointNamesForInstance(instIdx);
		if (!jn.isEmpty() && lower.size() == jn.size() && upper.size() == jn.size())
		{
			m_host->robotAxisControlPage()->setJoints(jn, lower, upper);
		}
		else
		{
			m_host->robotAxisControlPage()->clearJoints();
		}
		syncRobotAxisControlExternalAxes(instIdx);

		// 切文档只同步轴控 UI；场景位姿由各 DocumentPage 自持，勿每次重推 FK
		{
			const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
			const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
			if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
			{
				const QVector<double> local = m_aggregatedJointAnglesRad.mid(jointOffset, nj);
				QSignalBlocker blocker(m_host->robotAxisControlPage());
				m_host->robotAxisControlPage()->setJointAnglesRad(local);
			}
		}
		captureMotionPreviewProgramStartJoints();
		syncRobotFrameSettingsFromDocument(instIdx);
		syncRobotExternalAxisSettingsFromDocument(instIdx);
		refreshRobotCoordinateFrameOverlays();
	}
	else
	{
		// 删机后无仿真上下文：关掉拖动示教，避免叠加罗盘残留
		if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode())
		{
			onSimulationTcpDragTeachModeChanged(false);
			m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		}
		else if (IRobotOsgViewHost* osg = m_host->osgView())
		{
			if (osg->isTcpDragTeachActive())
			{
				osg->endTcpDragTeach();
			}
		}
		m_host->simulationCommandPage()->setRobotInstances(QStringList(), QStringList());
		m_host->simulationCommandPage()->setRevoluteJointNames(QStringList());
		m_host->simulationCommandPage()->setTcpLinkOptions(QStringList(), QString());
		m_host->robotAxisControlPage()->clearJoints();
		refreshAxisControlTargets();
		// 保留聚合角与 m_syncedRobotSceneBackendIds：空文档切回原机时可对齐；无交集的新机在上方清零
		m_motionPreviewProgramStartJointRad.clear();
	}
}

void RobotSimulationController::restoreAggregatedJointStateAfterProjectLoad(const QVector<double>& allJointAnglesRad)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int total = doc->robotRevoluteJointNames().size();
	if (allJointAnglesRad.size() != total)
	{
		return;
	}
	m_aggregatedJointAnglesRad = allJointAnglesRad;
	{
		QStringList ids;
		const int n = doc->robotKinematicInstanceCount();
		ids.reserve(n);
		for (int i = 0; i < n; ++i)
		{
			ids.append(doc->robotSceneBackendIdForInstance(i));
		}
		m_syncedRobotSceneBackendIds = ids;
	}
	m_boundDocumentId = doc->documentId();
	if (!m_boundDocumentId.isEmpty())
	{
		DocumentJointUiState& stash = m_jointUiByDocumentId[m_boundDocumentId];
		stash.aggregatedJointAnglesRad = m_aggregatedJointAnglesRad;
		stash.syncedRobotSceneBackendIds = m_syncedRobotSceneBackendIds;
	}
	const int instIdx =
		m_host->simulationCommandPage() && m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
			? m_host->simulationCommandPage()->currentRobotInstanceIndex()
			: 0;
	if (instIdx < 0)
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (m_host->robotAxisControlPage() && nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
	{
		QVector<double> local(nj);
		for (int j = 0; j < nj; ++j)
		{
			local[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
		QSignalBlocker blocker(m_host->robotAxisControlPage());
		m_host->robotAxisControlPage()->setJointAnglesRad(local);
	}
	captureMotionPreviewProgramStartJoints();
	refreshInstructionPoseAxes();
}

void RobotSimulationController::applyProgramStartPoseAfterProjectLoad()
{
	QPointer<RobotSimulationController> guard(this);
	QTimer::singleShot(0, this,
					   [guard]()
					   {
						   if (!guard)
						   {
							   return;
						   }
						   guard->applyProgramStartPoseAfterProjectLoadImpl();
					   });
}

void RobotSimulationController::finishProgramStartPoseAfterProjectLoad(const int instIdx,
																	   const QVector<double> startJointQ)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	if (!doc || !poseSink || instIdx < 0 || !m_host || !m_host->simulationCommandPage())
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (startJointQ.size() == nj)
	{
		(void)doc->applyJointAnglesRad(instIdx, startJointQ, m_aggregatedJointAnglesRad);
		captureMotionPreviewProgramStartJoints();
	}
	refreshInstructionPoseAxes(false);
}

void RobotSimulationController::applyProgramStartPoseAfterProjectLoadImpl()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	if (!doc || !poseSink || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	if (instIdx < 0)
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (nj <= 0)
	{
		return;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	QVector<double> startQForScene;
	if (!motions.empty() && motions.front())
	{
		const QVector<double> startQ = RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*motions.front());
		if (startQ.size() == nj)
		{
			const QStringList jnamesAll = doc->robotRevoluteJointNames();
			if (m_aggregatedJointAnglesRad.size() != jnamesAll.size())
			{
				resyncAggregatedJointAnglesFromDocument(doc);
			}
			for (int j = 0; j < nj && jointOffset + j < m_aggregatedJointAnglesRad.size(); ++j)
			{
				m_aggregatedJointAnglesRad[jointOffset + j] = startQ[j];
			}
			if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
			{
				QSignalBlocker blocker(m_host->robotAxisControlPage());
				m_host->robotAxisControlPage()->setJointAnglesRad(startQ);
			}
			startQForScene = startQ;
		}
	}
	osg::Matrixd robotBaseWorldAtLoad;
	robotBaseWorldAtLoad.makeIdentity();
	IRobotOsgViewHost* loadOsg = m_host ? m_host->osgView() : nullptr;
	(void)RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, loadOsg, instIdx, robotBaseWorldAtLoad);
	for (const auto& ins : program)
	{
		if (!ins || !RobotInstruction::isMotionWaypointType(ins->type()))
		{
			continue;
		}
		const auto& ext = ins->extensionProperties();
		osg::Matrixd savedWorld;
		savedWorld.makeIdentity();
		bool hasSavedWorld = false;
		const auto itSavedWorld = ext.find("render.tcpWorldMat4");
		if (itSavedWorld != ext.end() && !itSavedWorld->second.empty())
		{
			hasSavedWorld = RobotSimulationMath::decodeMatrix4Csv(itSavedWorld->second, savedWorld);
		}
		osg::Matrixd tcpLocalForRecompute;
		tcpLocalForRecompute.makeIdentity();
		engine::RigidTransform targetForRecompute{};
		if (RobotInstruction::readTargetTransformFromInstruction(*ins, targetForRecompute))
		{
			tcpLocalForRecompute = engine::osgMatrixFromRigidTransform(targetForRecompute);
		}
		const auto itSavedLocal = ext.find("render.tcpLocalMat4");
		if (itSavedLocal != ext.end() && !itSavedLocal->second.empty())
		{
			osg::Matrixd savedLocal;
			if (RobotSimulationMath::decodeMatrix4Csv(itSavedLocal->second, savedLocal))
			{
				tcpLocalForRecompute = savedLocal;
			}
		}
		if (hasSavedWorld)
		{
			// 已有世界系 TCP 缓存则跳过；避免用加载时 robotBaseWorld 再乘一次
			continue;
		}
		if (itSavedLocal != ext.end() && !itSavedLocal->second.empty())
		{
			osg::Matrixd savedLocal;
			if (RobotSimulationMath::decodeMatrix4Csv(itSavedLocal->second, savedLocal))
			{
				ins->setExtensionProperty("render.tcpWorldMat4",
										  RobotSimulationMath::encodeMatrix4Csv(savedLocal * robotBaseWorldAtLoad));
				continue;
			}
		}
		syncInstructionRenderMatricesFromPose(ins);
	}
	QPointer<RobotSimulationController> guard(this);
	const QVector<double> startQCopy = startQForScene;
	QTimer::singleShot(0, this,
					   [guard, instIdx, startQCopy]()
					   {
						   if (!guard)
						   {
							   return;
						   }
						   guard->finishProgramStartPoseAfterProjectLoad(instIdx, startQCopy);
					   });
}

void RobotSimulationController::captureMotionPreviewProgramStartJoints()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		m_motionPreviewProgramStartJointRad.clear();
		return;
	}
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	m_motionPreviewProgramStartJointRad = QVector<double>(jnamesAll.size(), 0.0);
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
	{
		for (int j = 0; j < nj && jointOffset + j < m_motionPreviewProgramStartJointRad.size(); ++j)
		{
			m_motionPreviewProgramStartJointRad[jointOffset + j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
	}
	else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		const QVector<double> local = m_host->robotAxisControlPage()->jointAnglesRad();
		for (int j = 0; j < nj && jointOffset + j < m_motionPreviewProgramStartJointRad.size(); ++j)
		{
			m_motionPreviewProgramStartJointRad[jointOffset + j] = local[j];
		}
	}
}

QVector<double> RobotSimulationController::localJointAnglesForInstance(const int instIdx) const
{
	QVector<double> out;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || instIdx < 0)
	{
		return out;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	out.resize(nj);
	if (m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
	{
		for (int j = 0; j < nj; ++j)
		{
			out[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
	}
	else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		out = m_host->robotAxisControlPage()->jointAnglesRad();
	}
	return out;
}

int RobotSimulationController::axisControlActiveRobotInstanceIndex() const
{
	RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!axis || !doc)
	{
		return -1;
	}
	const AxisControlTargetItem target = axis->currentControlTarget();
	if (target.kind == AxisControlTargetKind::CustomDevice)
	{
		return -1;
	}
	if (target.robotInstanceIndex >= 0)
	{
		return target.robotInstanceIndex;
	}
	if (!target.id.isEmpty())
	{
		const int idx = doc->robotInstanceIndexForSceneBackendId(target.id);
		if (idx >= 0)
		{
			return idx;
		}
	}
	if (m_host->simulationCommandPage())
	{
		return m_host->simulationCommandPage()->currentRobotInstanceIndex();
	}
	return -1;
}

QVector<double> RobotSimulationController::motionPreviewProgramStartJointsLocal(const int nj,
																				const int jointOffset) const
{
	QVector<double> rollingQ(nj, 0.0);
	if (nj <= 0)
	{
		return rollingQ;
	}
	bool fromFrozen = false;
	if (m_motionPreviewProgramStartJointRad.size() > jointOffset)
	{
		for (int j = 0; j < nj && jointOffset + j < m_motionPreviewProgramStartJointRad.size(); ++j)
		{
			rollingQ[j] = m_motionPreviewProgramStartJointRad[jointOffset + j];
		}
		fromFrozen = true;
	}
	const bool axisOk =
		m_host && m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj;
	if (axisOk)
	{
		const QVector<double> axisQ = m_host->robotAxisControlPage()->jointAnglesRad();
		bool frozenNearZero = true;
		for (int j = 0; j < nj; ++j)
		{
			if (std::abs(rollingQ[j]) > 1e-6)
			{
				frozenNearZero = false;
				break;
			}
		}
		bool axisMeaningful = false;
		for (int j = 0; j < axisQ.size(); ++j)
		{
			if (std::abs(axisQ[j]) > 1e-3)
			{
				axisMeaningful = true;
				break;
			}
		}
		// 导入瞬间捕获的全零起点会毒害 IK；轴控已有位姿时改用轴控
		if ((!fromFrozen || (frozenNearZero && axisMeaningful)) && axisQ.size() == nj)
		{
			return axisQ;
		}
	}
	if (fromFrozen)
	{
		return rollingQ;
	}
	return rollingQ;
}

bool RobotSimulationController::buildChainSeedJointRadForInstruction(
	const std::shared_ptr<RobotInstruction::Base>& instruction, QVector<double>& outChainSeed,
	int* outTargetMotionIndex, bool* outChainReliable)
{
	outChainSeed.clear();
	if (outChainReliable)
	{
		*outChainReliable = true;
	}
	if (outTargetMotionIndex)
	{
		*outTargetMotionIndex = -1;
	}
	if (!instruction || !m_host || !m_host->simulationCommandPage())
	{
		return false;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc || !doc->hasRobotSimulationContext())
	{
		return false;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return false;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return false;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return false;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	const std::string targetId = instruction->id();
	int targetMotionIndex = -1;
	for (size_t i = 0; i < motions.size(); ++i)
	{
		if (motions[i] && motions[i]->id() == targetId)
		{
			targetMotionIndex = static_cast<int>(i);
			break;
		}
	}
	if (targetMotionIndex < 0)
	{
		return false;
	}
	if (outTargetMotionIndex)
	{
		*outTargetMotionIndex = targetMotionIndex;
	}
	const QString defaultTcpLinkName =
		RobotSimulationMath::defaultTcpLinkNameForUrdf(urdfPath, m_host->simulationCommandPage()->selectedTcpLink());
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	const QVector<double> programStartQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);

	QString rollFp = robotBackendId;
	rollFp += QLatin1Char('|');
	rollFp += QStringLiteral("consistent");
	rollFp += QLatin1Char('|');
	rollFp += QString::number(nj);
	rollFp += QLatin1Char('|');
	for (double v : programStartQ)
	{
		rollFp += QString::number(v, 'g', 9);
		rollFp += QLatin1Char(',');
	}
	for (const RobotInstruction::Base* m : motions)
	{
		rollFp += m ? QString::fromStdString(m->id()) : QStringLiteral("-");
		rollFp += QLatin1Char(';');
	}
	if (rollFp != m_chainSeedRollFingerprint)
	{
		m_chainSeedRollFingerprint = rollFp;
		m_chainSeedEndJointsByIndex.clear();
	}

	if (targetMotionIndex == 0)
	{
		outChainSeed = programStartQ;
		return true;
	}

	if (m_chainSeedEndJointsByIndex.size() >= targetMotionIndex &&
		m_chainSeedEndJointsByIndex[targetMotionIndex - 1].size() == nj)
	{
		outChainSeed = m_chainSeedEndJointsByIndex[targetMotionIndex - 1];
		return true;
	}

	QVector<double> rollingQ = programStartQ;
	int startMi = 0;
	if (!m_chainSeedEndJointsByIndex.isEmpty() && m_chainSeedEndJointsByIndex.size() < targetMotionIndex)
	{
		const int lastCached = m_chainSeedEndJointsByIndex.size() - 1;
		if (lastCached >= 0 && m_chainSeedEndJointsByIndex[lastCached].size() == nj)
		{
			rollingQ = m_chainSeedEndJointsByIndex[lastCached];
			startMi = lastCached + 1;
		}
	}
	else if (m_chainSeedEndJointsByIndex.size() > targetMotionIndex)
	{
		m_chainSeedEndJointsByIndex.resize(targetMotionIndex);
		if (targetMotionIndex > 0 && m_chainSeedEndJointsByIndex[targetMotionIndex - 1].size() == nj)
		{
			outChainSeed = m_chainSeedEndJointsByIndex[targetMotionIndex - 1];
			return true;
		}
		startMi = 0;
		rollingQ = programStartQ;
		m_chainSeedEndJointsByIndex.clear();
	}

	auto storeEndAt = [&](int mi, const QVector<double>& q)
	{
		if (m_chainSeedEndJointsByIndex.size() == mi)
		{
			m_chainSeedEndJointsByIndex.push_back(q);
		}
		else if (m_chainSeedEndJointsByIndex.size() > mi)
		{
			m_chainSeedEndJointsByIndex[mi] = q;
		}
		else
		{
			while (m_chainSeedEndJointsByIndex.size() < mi)
			{
				m_chainSeedEndJointsByIndex.push_back(QVector<double>());
			}
			m_chainSeedEndJointsByIndex.push_back(q);
		}
	};

	m_chainSeedEndJointsByIndex.reserve(targetMotionIndex);
	for (int mi = startMi; mi < targetMotionIndex; ++mi)
	{
		RobotInstruction::Base* motionIns = const_cast<RobotInstruction::Base*>(motions[static_cast<size_t>(mi)]);
		if (!motionIns)
		{
			if (outChainReliable)
			{
				*outChainReliable = false;
			}
			outChainSeed = programStartQ;
			return true;
		}
		const RobotInstructionPlanning::MotionPoseBackup backup =
			RobotInstructionPlanning::backupInstructionPose(*motionIns);
		bool gotJoints = false;
		{
			// 与预览/Run 同一入口，避免 host 原始 IK 把另一支解写入 chainSeed
			std::string planErr;
			RobotInstruction::PlanResult plan{};
			motionIns->eraseExtensionProperty("context.playbackPlanLite");
			const bool okPlan = planMotionConsistentWithPreview(
				*motionIns, rollingQ, programStartQ, instIdx, urdfPath, defaultTcpLinkName, robotBackendId, frames,
				plan, &planErr, /*persistTaughtOnSuccess=*/false, /*gateTaughtResidual=*/true);
			if (okPlan && plan.ok && plan.jointTargetsRad.size() == static_cast<size_t>(nj))
			{
				if (rollingQ.size() != nj)
				{
					rollingQ.resize(nj);
				}
				for (int j = 0; j < nj; ++j)
				{
					rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
				}
				gotJoints = true;
			}
		}
		RobotInstructionPlanning::restoreInstructionPose(*motionIns, backup);
		if (!gotJoints)
		{
			if (outChainReliable)
			{
				*outChainReliable = false;
			}
			outChainSeed = programStartQ;
			m_chainSeedEndJointsByIndex.resize(mi);
			return true;
		}
		storeEndAt(mi, rollingQ);
	}
	outChainSeed = rollingQ;
	return outChainSeed.size() == nj;
}

void RobotSimulationController::applyToolFrameChangeToProgram(const RobotCoordinate::RobotCoordinateFrameSet& oldFrames,
															  const RobotCoordinate::RobotCoordinateFrameSet& newFrames,
															  const bool activeToolChanged,
															  const bool toolGeometryChanged)
{
	if (!activeToolChanged && !toolGeometryChanged)
	{
		return;
	}
	if (!m_host || !m_host->simulationCommandPage())
	{
		return;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	if (motions.empty())
	{
		return;
	}

	std::unordered_set<std::string> changedToolIds;
	if (toolGeometryChanged)
	{
		for (const RobotCoordinate::RobotToolFrame& nt : newFrames.toolFrames)
		{
			const RobotCoordinate::RobotToolFrame* ot = findToolFrameByIdInSet(oldFrames, nt.id);
			if (!ot || !toolFrameGeometryMatches(*ot, nt))
			{
				changedToolIds.insert(nt.id);
			}
		}
	}

	int firstInvalidateIndex = static_cast<int>(motions.size());
	for (size_t i = 0; i < motions.size(); ++i)
	{
		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motions[i]);
		if (!ins)
		{
			continue;
		}
		bool affected = false;
		if (activeToolChanged && RobotInstructionPlanning::motionFollowsActiveToolFrame(*ins))
		{
			RobotInstructionPlanning::syncInstructionToolContextFromFrames(*ins, newFrames);
			affected = true;
		}
		if (toolGeometryChanged)
		{
			const auto& ext = ins->extensionProperties();
			const auto itMotion = ext.find(RobotCoordinate::kExtMotionToolFrameId);
			const std::string motionToolId = (itMotion != ext.end()) ? itMotion->second : std::string();
			if (RobotInstructionPlanning::motionFollowsActiveToolFrame(*ins))
			{
				if (changedToolIds.count(newFrames.activeToolFrameId) > 0)
				{
					RobotInstructionPlanning::syncInstructionToolContextFromFrames(*ins, newFrames);
					affected = true;
				}
			}
			else if (!motionToolId.empty() && motionToolId != "active" && changedToolIds.count(motionToolId) > 0)
			{
				affected = true;
			}
		}
		if (affected && static_cast<int>(i) < firstInvalidateIndex)
		{
			firstInvalidateIndex = static_cast<int>(i);
		}
	}
	if (firstInvalidateIndex < static_cast<int>(motions.size()))
	{
		RobotInstructionPlanning::invalidateTaughtJointsFromMotionIndexForward(motions, firstInvalidateIndex);
		if (m_host->runInfoPage())
		{
			int pointIndex = RobotInstruction::motionPointIndex(*motions[static_cast<size_t>(firstInvalidateIndex)]);
			if (pointIndex <= 0)
			{
				pointIndex = firstInvalidateIndex + 1;
			}
			const QString pointTag = QString::fromStdString(RobotInstruction::formatMotionPointName(pointIndex));
			m_host->appendRunInfo(m_host->i18n(
										 QStringLiteral("Taught joints from %1 onward were invalidated by the tool frame change; please re-teach."),
										 QStringLiteral("%1 起示教角因工具系变更已失效，需重新示教。"))
									  .arg(pointTag));
		}
	}
}

void RobotSimulationController::syncRobotFrameSettingsFromDocument(const int instanceIndex)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->robotFrameSettingsPage() || instanceIndex < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instanceIndex);
	QStringList linkNames;
	QStringList childLinks;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
	QSet<QString> uniq;
	for (const QString& l : childLinks)
	{
		if (!l.isEmpty() && !uniq.contains(l))
		{
			uniq.insert(l);
			linkNames.push_back(l);
		}
	}
	m_host->robotFrameSettingsPage()->setLinkNameOptions(linkNames);
	m_host->robotFrameSettingsPage()->setCoordinateFrames(doc->robotCoordinateFramesForInstance(instanceIndex));
	if (m_host->robotFrameSettingsPage())
	{
		m_host->robotFrameSettingsPage()->setUseChinese(m_host->useChinese());
	}
}

void RobotSimulationController::syncRobotExternalAxisSettingsFromDocument(const int instanceIndex)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	RobotExternalAxisSettingsWidget* page = m_host ? m_host->robotExternalAxisSettingsPage() : nullptr;
	if (!doc || !page || instanceIndex < 0)
	{
		return;
	}
	QStringList backendIds;
	for (const std::shared_ptr<BackendDataBase>& data : doc->listObjects())
	{
		if (!data)
		{
			continue;
		}
		backendIds.append(QString::fromStdString(data->id()));
	}
	page->setBackendIdOptions(backendIds);
	page->setJointNameOptions(doc->robotRevoluteJointNamesForInstance(instanceIndex));
	page->setExternalAxes(doc->robotExternalAxesForInstance(instanceIndex));
	page->setUseChinese(m_host->useChinese());
	syncRobotAxisControlExternalAxes(instanceIndex);
}

void RobotSimulationController::onRobotExternalAxesChanged()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	RobotExternalAxisSettingsWidget* page = m_host ? m_host->robotExternalAxisSettingsPage() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !page)
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	doc->robotExternalAxesForInstance(instIdx) = page->externalAxes();
	{
		const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instIdx);
		std::vector<double> homes(set.axes.size(), 0.0);
		for (size_t i = 0; i < set.axes.size(); ++i)
		{
			homes[i] = set.axes[i].home;
		}
		doc->setRobotExternalAxisQ(instIdx, homes);
	}
	syncInstructionControllerExternalAxes(m_instructionController, doc, instIdx);
	m_axisControlExternalQApplied.clear();
	m_axisControlExternalInstIdx = -1;
	syncRobotAxisControlExternalAxes(instIdx);
	invalidateFeasibleAxisConfigurationCache();
}

void RobotSimulationController::onRobotCollisionSettingsChanged()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_simulationDock || !m_simulationDock->collisionPage())
		return;
	doc->robotCollisionSettings() = m_simulationDock->collisionPage()->settings();
	m_planResultCache.invalidateAll();
}

QVector<MotionPathWaypointItem> RobotSimulationController::collectMotionPathWaypoints() const
{
	QVector<MotionPathWaypointItem> out;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc)
		return out;
	const RobotInstruction::RobotProgramCatalog& catalog = doc->robotProgramStore().activeCatalog();
	const RobotInstruction::RobotProgram* prog = catalog.findProgram(catalog.activeProgramId());
	if (!prog)
		return out;
	std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
	// 与 insertRawTrajectoryBetween 一致：仅顶层 steps（避免嵌套路点无法插入）
	for (const auto& ins : prog->steps)
		flat.push_back(ins);
	int ordinal = 0;
	for (const std::shared_ptr<RobotInstruction::Base>& ins : flat)
	{
		if (!ins || !RobotInstruction::isMotionWaypointType(ins->type()))
			continue;
		++ordinal;
		MotionPathWaypointItem item;
		item.id = QString::fromStdString(ins->id());
		const QString name = QString::fromStdString(ins->name());
		const QString type = QString::fromStdString(RobotInstruction::typeToString(ins->type()));
		item.label = QStringLiteral("%1. %2 [%3]")
						 .arg(ordinal)
						 .arg(name.isEmpty() ? QStringLiteral("(unnamed)") : name)
						 .arg(type);
		out.push_back(item);
	}
	return out;
}

QVector<CollisionSceneObjectItem> RobotSimulationController::collectCollisionSceneObjects() const
{
	QVector<CollisionSceneObjectItem> out;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc)
		return out;
	BackendDataManager& backend = doc->backend();
	std::unordered_set<std::string> robotIds;
	for (int ri = 0; ri < doc->robotKinematicInstanceCount(); ++ri)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (!doc->robotPerLinkKinematicsForInstance(ri, pl))
			continue;
		for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
		{
			if (it.value().isEmpty())
				continue;
			robotIds.insert(it.value().toStdString());
			CollisionSceneObjectItem item;
			item.backendId = it.value();
			item.label = QStringLiteral("%1 (robot/%2)").arg(it.value(), it.key());
			out.push_back(item);
		}
	}
	for (const auto& data : backend.listData())
	{
		if (!data || !data->hasGeometry())
			continue;
		const std::string bid = data->id();
		if (robotIds.count(bid) > 0)
			continue;
		if (data->className() == "RobotAssembly" || data->className() == "Group")
			continue;
		const bool isMesh = static_cast<bool>(std::dynamic_pointer_cast<MeshBackendData>(data));
		const bool isBrep = static_cast<bool>(std::dynamic_pointer_cast<BrepBackendData>(data));
		if (!isMesh && !isBrep)
			continue;
		CollisionSceneObjectItem item;
		item.backendId = QString::fromStdString(bid);
		const QString kind = isMesh ? QStringLiteral("mesh") : QStringLiteral("brep");
		item.label = QStringLiteral("%1 (%2)").arg(item.backendId, kind);
		out.push_back(item);
	}
	return out;
}

void RobotSimulationController::refreshCollisionPageMotionWaypoints()
{
	if (!m_simulationDock)
		return;
	RobotCollisionSettingsWidget* col = m_simulationDock->collisionPage();
	if (!col)
		return;
	const QVector<MotionPathWaypointItem> wps = collectMotionPathWaypoints();
	col->setMotionWaypoints(wps);
	col->selectMotionWaypointIds(m_motionPathStartInstructionId, m_motionPathEndInstructionId);
}

void RobotSimulationController::refreshCollisionPageSceneObjects()
{
	if (!m_simulationDock)
		return;
	RobotCollisionSettingsWidget* col = m_simulationDock->collisionPage();
	if (!col)
		return;
	col->setCollisionSceneObjects(collectCollisionSceneObjects());
}

void RobotSimulationController::onMotionPathPlanRequested()
{
	if (m_programExecutor.isRunning())
		return;
	refreshCollisionPageMotionWaypoints();
	RobotCollisionSettingsWidget* col = m_simulationDock ? m_simulationDock->collisionPage() : nullptr;
	if (!col)
		return;
	const QString startId = col->selectedStartWaypointId();
	const QString endId = col->selectedEndWaypointId();
	if (startId.isEmpty() || endId.isEmpty() || startId == endId)
	{
		const QString msg = QStringLiteral("请选择不同的起点与终点路点");
		if (m_host && m_host->runInfoPage())
			m_host->appendRunWarning(msg);
		col->setPlanStatusText(msg);
		col->setConfirmEnabled(false);
		return;
	}
	runMotionPathPlanFromWaypoints(startId, endId);
}

void RobotSimulationController::runMotionPathPlanFromWaypoints(const QString& startId, const QString& endId)
{
	auto setUiStatus = [this](const QString& text, bool confirmEnabled) {
		if (m_simulationDock)
		{
			if (RobotCollisionSettingsWidget* col = m_simulationDock->collisionPage())
			{
				col->setPlanStatusText(text);
				col->setConfirmEnabled(confirmEnabled);
			}
		}
	};

	if (m_programExecutor.isRunning())
		return;
	if (startId.isEmpty() || endId.isEmpty() || startId == endId)
	{
		setUiStatus(QStringLiteral("请选择不同的起点与终点路点"), false);
		return;
	}

	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage())
		return;
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
		return;

	const std::shared_ptr<RobotInstruction::Base> startIns = findInstructionById(startId);
	const std::shared_ptr<RobotInstruction::Base> endIns = findInstructionById(endId);
	if (!startIns || !endIns || !RobotInstruction::isMotionWaypointType(startIns->type()) ||
		!RobotInstruction::isMotionWaypointType(endIns->type()))
	{
		setUiStatus(QStringLiteral("起终点路点无效"), false);
		return;
	}

	// 指令不落盘关节（persist 会 erase CSV）；起点关节与预览同源现算
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath, m_host->simulationCommandPage()->selectedTcpLink());
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0 || urdfPath.isEmpty())
	{
		setUiStatus(QStringLiteral("机器人运动学不可用"), false);
		return;
	}
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const QVector<double> programStartQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	QVector<double> chainSeedQ;
	int startMotionIndex = -1;
	bool chainReliable = true;
	if (!buildChainSeedJointRadForInstruction(startIns, chainSeedQ, &startMotionIndex, &chainReliable))
	{
		chainSeedQ = programStartQ;
		chainReliable = false;
	}
	const QVector<double> seedForStart = chainReliable ? chainSeedQ : programStartQ;
	RobotInstruction::PlanResult startPlan{};
	std::string startIkErr;
	const RobotCoordinate::RobotCoordinateFrameSet& framesForStart =
		doc->robotCoordinateFramesForInstance(instIdx);
	// 此处只要起点关节；勿用「到起点的路径」碰撞门控（leave-at-hit 后旧稠密轨迹会误拦，OMPL 永远进不去）
	const RobotCollision::Settings colBackupForStart = doc->robotCollisionSettings();
	{
		RobotCollision::Settings colSkip = colBackupForStart;
		colSkip.enabled = false;
		doc->robotCollisionSettings() = colSkip;
	}
	const bool startOk = planMotionConsistentWithPreview(
		*startIns, seedForStart, programStartQ, instIdx, urdfPath, defaultTcpLinkName, robotBackendId, framesForStart,
		startPlan, &startIkErr, /*persistTaughtOnSuccess=*/false, /*gateTaughtResidual=*/true);
	doc->robotCollisionSettings() = colBackupForStart;
	if (!startOk)
	{
		const QString detail =
			startIkErr.empty() ? QStringLiteral("起点 IK 失败") : QString::fromStdString(startIkErr);
		if (m_host->runInfoPage())
			m_host->appendRunWarning(detail);
		setUiStatus(QStringLiteral("起点 IK 失败（见日志）"), false);
		return;
	}
	QVector<double> startQ;
	startQ.reserve(static_cast<int>(startPlan.jointTargetsRad.size()));
	for (double v : startPlan.jointTargetsRad)
		startQ.push_back(v);
	if (startQ.size() != nj)
	{
		setUiStatus(QStringLiteral("起点 IK 关节维数不匹配"), false);
		return;
	}

	robot_path::TcpPose goal{};
	engine::RigidTransform Tgoal;
	if (!RobotInstruction::readTargetTransformFromInstruction(*endIns, Tgoal))
	{
		setUiStatus(QStringLiteral("终点路点无有效目标位姿"), false);
		return;
	}
	Tgoal.translationMm(goal.transMm[0], goal.transMm[1], goal.transMm[2]);
	{
		const Eigen::Quaterniond q = Tgoal.rotation();
		goal.quatXyzw[0] = q.x();
		goal.quatXyzw[1] = q.y();
		goal.quatXyzw[2] = q.z();
		goal.quatXyzw[3] = q.w();
	}

	m_motionPathStartInstructionId = startId;
	m_motionPathEndInstructionId = endId;

	RobotCoordinate::RobotCoordinateFrameSet frames = doc->robotCoordinateFramesForInstance(instIdx);
	const RobotCoordinate::RobotToolFrame* activeTool = RobotCoordinate::activeToolFrame(frames);
	QString flangeLink = activeTool
							 ? QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *activeTool))
							 : QString();
	if (flangeLink.isEmpty())
	{
		flangeLink = RobotSimulationMath::defaultTcpLinkNameForUrdf(
			urdfPath, m_host->simulationCommandPage()->selectedTcpLink());
	}
	BackendMat4 T_tool = BackendMat4::identity();
	if (activeTool)
	{
		T_tool = RobotCoordinate::frameToMat4(activeTool->T_flange_tool);
	}

	cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
	if (!doc->robotPerLinkKinematicsForInstance(instIdx, pl))
	{
		setUiStatus(QStringLiteral("per-link 运动学不可用"), false);
		return;
	}
	QHash<QString, collision::CollisionBodyId> linkBodies;
	for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
	{
		collision::CollisionBodyId id;
		id.kind = "robotLink";
		id.backendId = it.value().toStdString();
		id.linkName = it.key().toStdString();
		linkBodies.insert(it.key(), id);
	}

	if (!m_collisionWorld)
	{
		m_collisionWorld = std::make_unique<collision::CollisionWorld>();
	}
	RobotCollision::Settings col = doc->robotCollisionSettings();
	if (m_simulationDock && m_simulationDock->collisionPage())
	{
		col = m_simulationDock->collisionPage()->settings();
		doc->robotCollisionSettings() = col;
	}
	if (col.enabled)
	{
		BackendCollisionSync::rebuildWorld(*m_collisionWorld, doc, doc->backend(), col, osg);
		QVector<double> agg;
		(void)doc->applyJointAnglesRad(instIdx, startQ, agg);
		BackendCollisionSync::updatePoses(*m_collisionWorld, doc, doc->backend(), osg);
	}

	BackendMat4 T_world_urdfBase = BackendMat4::identity();
	{
		const QString refBid = doc->robotFrameWorldReferenceBackendId(instIdx);
		QString refLink;
		for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
		{
			if (it.value() == refBid)
			{
				refLink = it.key();
				break;
			}
		}
		if (!refLink.isEmpty())
		{
			QHash<QString, osg::Matrixd> meshFk;
			QString fkErr;
			if (UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, startQ, meshFk, &fkErr, pl.meshVerticesInLinkFrame))
			{
				const auto fkIt = meshFk.constFind(refLink);
				auto be = doc->backend().getData(refBid.toStdString());
				if (fkIt != meshFk.constEnd() && be)
				{
					const engine::RigidTransform T_be = engine::rigidTransformFromColMajor([&] {
						engine::ColMajorMat4 cm{};
						BackendMat4 W = be->worldMatrix();
						cloudsim::core::Mat4 osgPacked{};
						if (osg && osg->getBackendRootWorldMatrix(refBid.toStdString(), osgPacked))
						{
							W = RobotMatrixOsg::backendColMajorFromMatrix(
								RobotSimulationMath::osgMatrixFromCoreMat4(osgPacked));
						}
						for (int i = 0; i < 16; ++i)
							cm[static_cast<size_t>(i)] = W.v[i];
						return cm;
					}());
					const engine::RigidTransform T_fk = engine::rigidTransformFromOsg(*fkIt);
					const engine::RigidTransform T_wb = T_be.composeScene(T_fk.inverse());
					const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(T_wb);
					for (int i = 0; i < 16; ++i)
						T_world_urdfBase.v[i] = cm[static_cast<size_t>(i)];
				}
			}
		}
	}

	robot_path::PlanRequest req;
	req.urdfPath = urdfPath;
	req.flangeLinkName = flangeLink;
	req.startJointRad.assign(startQ.begin(), startQ.end());
	req.goalToolInBase = goal;
	req.T_flange_tool = T_tool;
	req.T_world_urdfBase = T_world_urdfBase;
	{
		auto backendFromOsgPacked = [](const cloudsim::core::Mat4& packed) {
			return RobotMatrixOsg::backendColMajorFromMatrix(RobotSimulationMath::osgMatrixFromCoreMat4(packed));
		};
		// 绑定位姿：直接 OSG 矩阵，与 applyPerLinkRobotBasePlacement 一致（禁止 Backend 往返）
		req.robotBasePlacementWorld = RobotSimulationMath::osgMatrixFromCoreMat4(pl.robotBasePlacementWorld);
		for (auto it = pl.fkMeshWorldT0.constBegin(); it != pl.fkMeshWorldT0.constEnd(); ++it)
			req.fkMeshWorldT0.insert(it.key(), RobotSimulationMath::osgMatrixFromCoreMat4(it.value()));
		for (auto it = pl.outerWorldAtBindByBackendId.constBegin(); it != pl.outerWorldAtBindByBackendId.constEnd(); ++it)
			req.outerWorldAtBindByBackendId.insert(it.key(), RobotSimulationMath::osgMatrixFromCoreMat4(it.value()));

		for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
		{
			const QString& linkName = it.key();
			const QString& bid = it.value();
			if (osg)
			{
				cloudsim::core::Mat4 wCm{};
				if (osg->getBackendRootWorldMatrix(bid.toStdString(), wCm))
					req.linkWorldAtStart.insert(linkName, backendFromOsgPacked(wCm));
			}
		}
	}
	req.world = col.enabled ? m_collisionWorld.get() : nullptr;
	req.linkBodies = linkBodies;
	req.meshVerticesInLinkFrame = pl.meshVerticesInLinkFrame;
	req.options.securityMarginMm = col.securityMarginMm;
	req.options.checkCollision = col.enabled;
	req.options.planningSpace = col.planningSpace.empty() ? "Auto" : col.planningSpace;
	req.options.plannerId = col.plannerId.empty() ? "Auto" : col.plannerId;
	req.options.planningTimeSec = col.planningTimeSec;
	// 场景有障碍时禁用关节直达，避免粗检漏过中段穿模；走 OMPL 绕障
	if (col.enabled && m_collisionWorld
		&& m_collisionWorld->bodyCount() > static_cast<std::size_t>(linkBodies.size()))
	{
		req.options.allowDirectJointLerp = false;
	}


	robot_path::PathResult plan;
	if (!robot_path::planToTcpPose(req, plan) || !plan.ok)
	{
		const QString err = QString::fromStdString(plan.errMsg.empty() ? "motion planning failed" : plan.errMsg);
		if (m_host->runInfoPage())
			m_host->appendRunWarning(err);
		// 状态栏只留短提示，详情走日志栏
		setUiStatus(QStringLiteral("规划失败（见日志）"), false);
		m_lastMotionPathRawValid = false;
		return;
	}

	// 画面复验（与 Run 同源）；Direct 漏检时禁用直达再采一次
	if (col.enabled && m_collisionWorld)
	{
		std::string osgColErr;
		auto sceneCheck = [&](const bool restoreOnHit) {
			return BackendCollisionSync::validateJointTrajectory(
				*m_collisionWorld, doc, doc->backend(), instIdx, startQ, plan.jointTrajectoryRad, col, &osgColErr, osg,
				/*rebuildWorldFirst=*/true, restoreOnHit);
		};
		// 命中不回滚，便于画面停在中段碰撞姿态（起点有缝≠路径安全）
		bool accepted = sceneCheck(/*restoreOnHit=*/false);
		if (!accepted && plan.plannerName == "Direct")
		{
			QVector<double> aggRestore;
			(void)doc->applyJointAnglesRad(instIdx, startQ, aggRestore);
			req.options.allowDirectJointLerp = false;
			robot_path::PathResult retry{};
			if (robot_path::planToTcpPose(req, retry) && retry.ok)
			{
				plan = std::move(retry);
				osgColErr.clear();
				accepted = sceneCheck(/*restoreOnHit=*/false);
			}
		}
		if (!accepted)
		{
			if (m_host->runInfoPage())
			{
				m_host->appendRunWarning(
					QStringLiteral("路径中段碰撞（直线/绕障均未通过画面复验）：%1")
						.arg(QString::fromStdString(osgColErr)));
			}
			setUiStatus(QStringLiteral("路径中段碰撞（见日志；画面已停在碰撞姿态）"), false);
			m_lastMotionPathRawValid = false;
			return;
		}
	}

	RobotInstruction::RawTrajectory raw;
	raw.points.reserve(plan.tcpPoses.size());
	const std::size_t nPose = plan.tcpPoses.size();
	const std::size_t nJoint = plan.jointTrajectoryRad.size();
	for (std::size_t i = 0; i < nPose; ++i)
	{
		const robot_path::TcpPose& tp = plan.tcpPoses[i];
		RobotInstruction::TrajectoryPoint pt;
		pt.poseMm.x = tp.transMm[0];
		pt.poseMm.y = tp.transMm[1];
		pt.poseMm.z = tp.transMm[2];
		pt.quatXyzw[0] = tp.quatXyzw[0];
		pt.quatXyzw[1] = tp.quatXyzw[1];
		pt.quatXyzw[2] = tp.quatXyzw[2];
		pt.quatXyzw[3] = tp.quatXyzw[3];
		pt.hasQuat = true;
		const osg::Quat q(tp.quatXyzw[3], tp.quatXyzw[0], tp.quatXyzw[1], tp.quatXyzw[2]);
		double ex = 0.0;
		double ey = 0.0;
		double ez = 0.0;
		engine::quatToEulerDeg(q, ex, ey, ez);
		pt.eulerDeg.x = ex;
		pt.eulerDeg.y = ey;
		pt.eulerDeg.z = ez;
		pt.reachable = true;
		pt.speedMmPerSec = 200.0;
		if (i < nJoint)
			pt.jointRad = plan.jointTrajectoryRad[i];
		raw.points.push_back(std::move(pt));
	}

	std::string previewErr;
	RobotOsgUi::RawTrajectoryPreviewOptions previewOpts;
	feature_pick_transform::applyWorldRawTrajectoryPreviewToOsg(osg, raw, previewOpts, &previewErr);
	m_lastMotionPathRaw = raw;
	m_lastMotionPathRawValid = !raw.points.empty();
	m_motionPathPreviewActive = true;
	setRawTrajectoryPreviewActive(true);

	const QString okMsg = QStringLiteral("OK：%1，%2 点，TCP 长 %3 mm（可确认插入中间点）")
							  .arg(QString::fromStdString(plan.plannerName))
							  .arg(static_cast<int>(plan.tcpPoses.size()))
							  .arg(plan.pathLengthTcpMm, 0, 'f', 1);
	if (m_host->runInfoPage())
		m_host->appendRunInfo(QStringLiteral("路径规划 ") + okMsg);
	setUiStatus(okMsg, m_lastMotionPathRawValid);

}

void RobotSimulationController::onMotionPathPreviewClearRequested()
{
	m_motionPathPreviewActive = false;
	m_lastMotionPathRawValid = false;
	m_lastMotionPathRaw = RobotInstruction::RawTrajectory{};
	setRawTrajectoryPreviewActive(false);
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->clearRawTrajectoryOverlay();
		osg->clearRawTrajectoryOverlayFrames();
	}
	refreshInstructionPoseAxes(false);
	if (m_simulationDock)
	{
		if (RobotCollisionSettingsWidget* col = m_simulationDock->collisionPage())
		{
			col->setPlanStatusText(QStringLiteral("已清除预览"));
			col->setConfirmEnabled(false);
		}
	}
}

void RobotSimulationController::onMotionPathConfirmTrajectoryRequested()
{
	if (!m_lastMotionPathRawValid || m_lastMotionPathRaw.points.empty())
	{
		if (m_host && m_host->runInfoPage())
			m_host->appendRunWarning(QStringLiteral("请先规划成功再确认生成"));
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc)
		return;
	RobotInstruction::RobotProgramCatalog& catalog = doc->robotProgramStore().activeCatalog();
	RobotInstruction::RobotProgram* prog = catalog.findProgram(catalog.activeProgramId());
	if (!prog)
	{
		if (m_host->runInfoPage())
			m_host->appendRunWarning(QStringLiteral("无活动程序，无法生成轨迹"));
		return;
	}

	std::string err;
	const bool insertBetween = !m_motionPathStartInstructionId.isEmpty() && !m_motionPathEndInstructionId.isEmpty();
	int writtenPoints = 0;
	if (insertBetween)
	{
		// 起终点已在程序中保留，只插入规划路径的中间点
		RobotInstruction::RawTrajectory mid = m_lastMotionPathRaw;
		if (mid.points.size() >= 2)
		{
			mid.points.erase(mid.points.begin());
			mid.points.pop_back();
		}
		else
		{
			mid.points.clear();
		}

		// 程序顺序与规划起终点相反时反转中间点，保证列表从前往后与路径连续
		{
			int iStart = -1;
			int iEnd = -1;
			const std::string sid = m_motionPathStartInstructionId.toStdString();
			const std::string eid = m_motionPathEndInstructionId.toStdString();
			for (int i = 0; i < static_cast<int>(prog->steps.size()); ++i)
			{
				const auto& ins = prog->steps[static_cast<std::size_t>(i)];
				if (!ins)
					continue;
				if (ins->id() == sid)
					iStart = i;
				if (ins->id() == eid)
					iEnd = i;
			}
			if (iStart >= 0 && iEnd >= 0 && iStart > iEnd)
				std::reverse(mid.points.begin(), mid.points.end());
		}

		writtenPoints = static_cast<int>(mid.points.size());
		if (!RobotInstruction::insertRawTrajectoryBetween(mid, *prog, m_motionPathStartInstructionId.toStdString(),
														  m_motionPathEndInstructionId.toStdString(), &err))
		{
			if (m_host->runInfoPage())
				m_host->appendRunWarning(QString::fromStdString(err.empty() ? "插入中间点失败" : err));
			if (RobotCollisionSettingsWidget* col = m_simulationDock ? m_simulationDock->collisionPage() : nullptr)
				col->setPlanStatusText(QString::fromStdString(err.empty() ? "插入失败" : err));
			return;
		}
		// 中间点只写 TCP；同步工具系，不落盘关节
		if (m_host->simulationCommandPage())
		{
			const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
			if (instIdx >= 0)
			{
				const RobotCoordinate::RobotCoordinateFrameSet frames =
					doc->robotCoordinateFramesForInstance(instIdx);
				int iStart = -1;
				int iEnd = -1;
				const std::string sid = m_motionPathStartInstructionId.toStdString();
				const std::string eid = m_motionPathEndInstructionId.toStdString();
				for (int i = 0; i < static_cast<int>(prog->steps.size()); ++i)
				{
					const auto& ins = prog->steps[static_cast<std::size_t>(i)];
					if (!ins)
						continue;
					if (ins->id() == sid)
						iStart = i;
					if (ins->id() == eid)
						iEnd = i;
				}
				if (iStart >= 0 && iEnd >= 0)
				{
					if (iStart > iEnd)
						std::swap(iStart, iEnd);
					for (int i = iStart + 1; i < iEnd; ++i)
					{
						auto& ins = prog->steps[static_cast<std::size_t>(i)];
						if (!ins)
							continue;
						RobotInstructionPlanning::persistTaughtJointsAndToolContext(*ins, {}, frames);
					}
				}
			}
		}
	}
	else
	{
		std::string emittedGroupId;
		if (!RobotInstruction::emitRawTrajectoryToProgram(m_lastMotionPathRaw, *prog, &err, &emittedGroupId, nullptr))
		{
			if (m_host->runInfoPage())
				m_host->appendRunWarning(QString::fromStdString(err.empty() ? "生成轨迹失败" : err));
			if (RobotCollisionSettingsWidget* col = m_simulationDock ? m_simulationDock->collisionPage() : nullptr)
				col->setPlanStatusText(QString::fromStdString(err.empty() ? "生成失败" : err));
			return;
		}
		writtenPoints = static_cast<int>(m_lastMotionPathRaw.points.size());
	}


	if (m_host->simulationCommandPage())
		m_host->simulationCommandPage()->refreshInstructionList();
	refreshCollisionPageMotionWaypoints();
	{
		std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
		RobotInstruction::flattenInstructionsRecursive(prog->steps, flat);
		for (const std::shared_ptr<RobotInstruction::Base>& ins : flat)
		{
			if (ins && RobotInstruction::isMotionWaypointType(ins->type()))
				syncInstructionRenderMatricesFromWorldPose(ins);
		}
	}
	m_motionPathPreviewActive = false;
	m_lastMotionPathRawValid = false;
	setRawTrajectoryPreviewActive(false);
	if (IRobotOsgViewHost* osg = m_host->osgView())
	{
		osg->clearRawTrajectoryOverlay();
		osg->clearRawTrajectoryOverlayFrames();
	}
	refreshInstructionPoseAxes(false);
	m_lastMotionPathRaw = RobotInstruction::RawTrajectory{};
	if (m_host->runInfoPage())
	{
		m_host->appendRunInfo(insertBetween
								  ? QStringLiteral("已在起终点之间插入中间点（%1 点）").arg(writtenPoints)
								  : QStringLiteral("已生成轨迹到程序（%1 点）").arg(writtenPoints));
	}
	if (RobotCollisionSettingsWidget* col = m_simulationDock ? m_simulationDock->collisionPage() : nullptr)
	{
		col->setPlanStatusText(insertBetween ? QStringLiteral("已插入中间点") : QStringLiteral("已写入程序"));
		col->setConfirmEnabled(false);
	}
}

void RobotSimulationController::onRobotCoordinateFramesChanged()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotFrameSettingsPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const RobotCoordinate::RobotCoordinateFrameSet oldFrames = doc->robotCoordinateFramesForInstance(instIdx);
	const RobotCoordinate::RobotCoordinateFrameSet newFrames = m_host->robotFrameSettingsPage()->coordinateFrames();
	if (coordinateFrameSetEquals(oldFrames, newFrames))
	{
		return;
	}
	const bool displayOnlyChange = coordinateFrameSetPlanningEquals(oldFrames, newFrames);
	const CoordinateFrameChangeKind changeKind = classifyCoordinateFrameChange(oldFrames, newFrames);
	doc->robotCoordinateFramesForInstance(instIdx) = newFrames;
	if (displayOnlyChange)
	{
		refreshRobotCoordinateFrameOverlays();
		return;
	}

	const bool activeToolChanged = changeKind == CoordinateFrameChangeKind::ActiveToolChanged;
	const bool toolGeometryChanged = changeKind == CoordinateFrameChangeKind::ToolGeometryChanged;
	if (activeToolChanged || toolGeometryChanged)
	{
		m_planResultCache.invalidateAll();
		invalidateChainSeedRollCache();
		m_motionReachabilityCache.clear();
		++m_reachabilityJobToken;
		m_host->invalidateInstructionPropertyCache();
		applyToolFrameChangeToProgram(oldFrames, newFrames, activeToolChanged, toolGeometryChanged);
		m_host->rebakeMountedCustomDevicesFollowLocalsForCurrentDocument();
	}
	else
	{
		m_host->invalidateInstructionPropertyCache();
	}

	refreshRobotCoordinateFrameOverlays();
	if (IRobotOsgViewHost* osg = m_host->osgView())
	{
		if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode() &&
			osg->isTcpDragTeachActive())
		{
			const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
			const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
			bool meshVerticesInLinkFrame = true;
			cloudsim::core::RobotPerLinkKinematicsSliceDto plSlice;
			if (doc->robotPerLinkKinematicsForInstance(instIdx, plSlice))
			{
				meshVerticesInLinkFrame = plSlice.meshVerticesInLinkFrame;
			}
			if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
			{
				const QString flangeQ = QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *tool));
				osg->updateTcpDragTeachToolLocalOnFlange(RobotSimulationMath::coreMat4FromOsgMatrix(
					RobotSimulationMath::linkFrameLocalOnMeshBackend(
						urdfPath, flangeQ.isEmpty() ? m_tcpDragTeachFlangeLink : flangeQ,
						RobotSimulationMath::osgMatrixFromRobotRigidFrame(tool->T_flange_tool),
						meshVerticesInLinkFrame)));
			}
			if (!urdfPath.isEmpty())
			{
				QStringList revoluteChildLinks;
				QString fallbackFlange;
				(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
				if (!revoluteChildLinks.isEmpty())
				{
					fallbackFlange = revoluteChildLinks.back();
				}
				const QVector<double> jointQ = localJointAnglesForInstance(instIdx);
				engine::RigidTransform fkTarget{};
				if (!jointQ.isEmpty() && RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
											 urdfPath, jointQ, frames, fallbackFlange, fkTarget, nullptr, nullptr))
				{
					osg->updateTcpDragTeachFromTarget(fkTarget, false);
				}
			}
		}
	}
	const bool computeReachability = activeToolChanged || toolGeometryChanged;
	refreshInstructionPoseAxes(computeReachability);
	if (const std::shared_ptr<RobotInstruction::Base> active = m_host->activeInstructionForProperty())
	{
		m_host->refreshInstructionPropertyPanel(active, false);
		const bool tcpDragActive =
			m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode();
		if (!tcpDragActive && RobotInstruction::isMotionWaypointType(active->type()) &&
			(activeToolChanged || toolGeometryChanged))
		{
			applyRobotPoseForInstructionPreview(active);
		}
	}
}

void RobotSimulationController::onCaptureToolFrameFromTcp()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotFrameSettingsPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 euler{};
	QString err;
	if (!tryCaptureCurrentRobotTcpPose(pose, euler, nullptr, nullptr, nullptr, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	const BackendMat4 T_base_tcp =
		RobotCoordinate::tcpInBaseFromPose(pose.x, pose.y, pose.z, euler.x, euler.y, euler.z);
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	RobotCoordinate::RobotCoordinateFrameSet frames = m_host->robotFrameSettingsPage()->coordinateFrames();
	const RobotCoordinate::RobotToolFrame* activeTool = RobotCoordinate::activeToolFrame(frames);
	QString flangeLink =
		activeTool ? QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *activeTool)) : QString();
	if (flangeLink.isEmpty())
	{
		flangeLink = RobotSimulationMath::defaultTcpLinkNameForUrdf(urdfPath,
																	m_host->simulationCommandPage()->selectedTcpLink());
	}
	QVector<double> q;
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() > 0)
	{
		q = m_host->robotAxisControlPage()->jointAnglesRad();
	}
	if (!doc->captureToolFrameFromTcp(instIdx, T_base_tcp, q, flangeLink, frames, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	m_host->robotFrameSettingsPage()->setCoordinateFrames(frames);
	doc->robotCoordinateFramesForInstance(instIdx) = frames;
	onRobotCoordinateFramesChanged();
}

void RobotSimulationController::onResetToolFrame()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotFrameSettingsPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	RobotCoordinate::RobotCoordinateFrameSet frames = m_host->robotFrameSettingsPage()->coordinateFrames();
	doc->resetToolFrame(instIdx, frames);
	m_host->robotFrameSettingsPage()->setCoordinateFrames(frames);
	doc->robotCoordinateFramesForInstance(instIdx) = frames;
	onRobotCoordinateFramesChanged();
}

void RobotSimulationController::onCaptureUserFrameFromTcp()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotFrameSettingsPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 euler{};
	QString err;
	if (!tryCaptureCurrentRobotTcpPose(pose, euler, nullptr, nullptr, nullptr, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	RobotCoordinate::RobotCoordinateFrameSet frames = m_host->robotFrameSettingsPage()->coordinateFrames();
	if (!doc->captureUserFrameFromTcp(instIdx, pose.x, pose.y, pose.z, euler.x, euler.y, euler.z, frames, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	m_host->robotFrameSettingsPage()->setCoordinateFrames(frames);
	doc->robotCoordinateFramesForInstance(instIdx) = frames;
	onRobotCoordinateFramesChanged();
}

void RobotSimulationController::refreshRobotCoordinateFrameOverlays(
	const std::shared_ptr<RobotInstruction::Base>& highlightInstruction, const QVector<double>* jointAnglesRadLocal)
{
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!osg || !doc || !m_host->simulationCommandPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString robotRootId = doc->robotSceneBackendIdForInstance(instIdx);
	if (robotRootId.isEmpty())
	{
		return;
	}
	osg->clearRobotFrameOverlays(robotRootId.toStdString());
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	if (!frames.showToolFrameInScene && !frames.showUserFramesInScene)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	QVector<double> jointQ;
	if (jointAnglesRadLocal && !jointAnglesRadLocal->isEmpty())
	{
		jointQ = *jointAnglesRadLocal;
	}
	else
	{
		const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
		const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
		if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
		{
			jointQ.resize(nj);
			for (int j = 0; j < nj; ++j)
			{
				jointQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
			}
		}
		else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
		{
			jointQ = m_host->robotAxisControlPage()->jointAnglesRad();
		}
	}
	const bool perLink = doc->robotUsesPerLinkBackendsForInstance(instIdx);
	bool meshVerticesInLinkFrame = true;
	if (perLink)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto plSlice;
		if (doc->robotPerLinkKinematicsForInstance(instIdx, plSlice))
		{
			meshVerticesInLinkFrame = plSlice.meshVerticesInLinkFrame;
		}
	}
	QString urdfRootLinkName;
	if (perLink)
	{
		if (m_overlayCachedUrdfPath == urdfPath && !m_overlayCachedUrdfRootLink.isEmpty())
		{
			urdfRootLinkName = m_overlayCachedUrdfRootLink;
		}
		else
		{
			QHash<QString, QString> linkMeshes;
			QString urdfListErr;
			(void)UrdfRobotLoader::enumerateLinkVisualMeshes(urdfPath, urdfRootLinkName, linkMeshes, &urdfListErr);
			m_overlayCachedUrdfPath = urdfPath;
			m_overlayCachedUrdfRootLink = urdfRootLinkName;
		}
	}
	const QString baseLinkBackendId = doc->robotFrameWorldReferenceBackendId(instIdx);
	std::string highlightToolId = frames.activeToolFrameId;
	if (highlightInstruction && highlightInstruction->hasPoseProperty())
	{
		if (const RobotCoordinate::RobotToolFrame* insTool =
				RobotCoordinate::resolveToolFrameForExtension(frames, highlightInstruction->extensionProperties()))
		{
			highlightToolId = insTool->id;
		}
	}
	RobotOsgUi::RobotFrameOverlayUpdate upd;
	upd.robotRootBackendId = robotRootId.toStdString();
	upd.showToolFrames = frames.showToolFrameInScene;
	upd.showUserFrames = frames.showUserFramesInScene;
	for (const RobotCoordinate::RobotToolFrame& tool : frames.toolFrames)
	{
		if (!tool.showInScene)
		{
			continue;
		}
		RobotOsgUi::RobotFrameOverlayUpdate::ToolEntry te;
		te.name = tool.name;
		te.active = (tool.id == highlightToolId);
		// per-link：工具轴挂在法兰 mesh 上。文件系顶点时 local=T_tool*inv(visual)
		if (perLink)
		{
			const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(frames, tool);
			te.mountBackendId =
				RobotSimulationMath::linkMeshBackendIdForInstance(doc, instIdx, flangeLink).toStdString();
			te.localMatrix = RobotSimulationMath::coreMat4FromOsgMatrix(RobotSimulationMath::linkFrameLocalOnMeshBackend(
				urdfPath, QString::fromStdString(flangeLink),
				RobotSimulationMath::osgMatrixFromRobotRigidFrame(tool.T_flange_tool), meshVerticesInLinkFrame));
			if (te.mountBackendId.empty())
			{
				continue;
			}
		}
		else
		{
			te.mountBackendId.clear();
			const osg::Matrixd tcpInBase = RobotSimulationMath::osgMatrixFromBackendMat4(
				RobotSimulationMath::toolTcpInBaseFromFk(urdfPath, jointQ, frames, tool));
			te.localMatrix = RobotSimulationMath::coreMat4FromOsgMatrix(tcpInBase);
		}
		upd.toolFrames.push_back(std::move(te));
	}
	for (const RobotCoordinate::RobotUserFrame& uf : frames.userFrames)
	{
		if (!uf.showInScene)
		{
			continue;
		}
		RobotOsgUi::RobotFrameOverlayUpdate::UserEntry ue;
		ue.name = uf.name;
		const QString userMountLink = urdfRootLinkName.isEmpty() ? QStringLiteral("base_link") : urdfRootLinkName;
		const osg::Matrixd userLinkLocal = RobotSimulationMath::osgMatrixFromRobotRigidFrame(uf.T_base_user);
		ue.localMatrix =
			perLink ? RobotSimulationMath::coreMat4FromOsgMatrix(RobotSimulationMath::linkFrameLocalOnMeshBackend(
						  urdfPath, userMountLink, userLinkLocal, meshVerticesInLinkFrame))
					: RobotSimulationMath::coreMat4FromOsgMatrix(userLinkLocal);
		if (perLink)
		{
			// per-link：用户系挂根 link mesh；FK 刷新 root worldMatrix
			if (!urdfRootLinkName.isEmpty())
			{
				const QString rootBackendId =
					RobotSimulationMath::linkMeshBackendIdForInstance(doc, instIdx, urdfRootLinkName.toStdString());
				ue.mountBackendId =
					rootBackendId.isEmpty() ? baseLinkBackendId.toStdString() : rootBackendId.toStdString();
			}
			else
			{
				ue.mountBackendId = baseLinkBackendId.toStdString();
			}
			if (ue.mountBackendId.empty())
			{
				continue;
			}
		}
		else
		{
			ue.mountBackendId.clear();
		}
		upd.userFrames.push_back(std::move(ue));
	}
	osg->setRobotFrameOverlays(upd);
}

void RobotSimulationController::refreshRobotCoordinateFrameOverlaysForPlayback()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage())
	{
		refreshRobotCoordinateFrameOverlays();
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		refreshRobotCoordinateFrameOverlays();
		return;
	}
	std::shared_ptr<RobotInstruction::Base> highlight;
	if (const RobotInstruction::Base* activeMotion = m_programExecutor.activeMotion())
	{
		if (m_playbackOverlayHighlight && m_playbackOverlayHighlight.get() == activeMotion)
		{
			highlight = m_playbackOverlayHighlight;
		}
		else
		{
			for (const std::shared_ptr<RobotInstruction::Base>& ins : m_host->simulationCommandPage()->instructionList())
			{
				if (ins && ins.get() == activeMotion)
				{
					highlight = ins;
					m_playbackOverlayHighlight = ins;
					break;
				}
			}
		}
	}
	else
	{
		m_playbackOverlayHighlight.reset();
	}
	if (!highlight)
	{
		const auto insList = m_host->simulationCommandPage()->instructionList();
		for (auto it = insList.rbegin(); it != insList.rend(); ++it)
		{
			if (*it && (*it)->hasPoseProperty())
			{
				highlight = *it;
				break;
			}
		}
	}
	QVector<double> jointQ;
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
	{
		jointQ.resize(nj);
		for (int j = 0; j < nj; ++j)
		{
			jointQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
	}
	refreshRobotCoordinateFrameOverlays(highlight, jointQ.isEmpty() ? nullptr : &jointQ);
}

void RobotSimulationController::clearPlaybackPathOverlays()
{
	m_waypointLabelOverlayFingerprint.clear();
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!osg)
	{
		return;
	}
	osg->clearPlaybackCursorOverlay();
	osg->clearWaypointIndexLabels();
}

void RobotSimulationController::onSimulationRobotSelectionChanged(int instanceIndex, const QString& sceneBackendId)
{
	(void)sceneBackendId;
	cancelArcTeach();
	m_planResultCache.invalidateAll();
	invalidateChainSeedRollCache();
	m_overlayCachedUrdfPath.clear();
	m_overlayCachedUrdfRootLink.clear();
	m_playbackOverlayHighlight.reset();
	clearPlaybackPathOverlays();
	clearReachableWorkspaceOverlayUi();
	if (RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr)
	{
		axis->setReachableWorkspaceChecked(false);
		axis->setReachableWorkspaceBusy(false);
	}
	if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode())
	{
		onSimulationTcpDragTeachModeChanged(false);
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
	}
	if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->instructionWaypointPickMode())
	{
		onInstructionWaypointPickModeChanged(false);
		m_host->simulationCommandPage()->setInstructionWaypointPickMode(false);
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage() || instanceIndex < 0)
	{
		return;
	}
	m_host->robotAxisControlPage()->selectControlTarget(
		AxisControlTargetKind::RobotInstance, doc->robotSceneBackendIdForInstance(instanceIndex));
	syncRobotFrameSettingsFromDocument(instanceIndex);
	syncRobotExternalAxisSettingsFromDocument(instanceIndex);
	if (m_simulationDock && m_simulationDock->collisionPage() && doc)
	{
		m_simulationDock->collisionPage()->setSettings(doc->robotCollisionSettings());
		refreshCollisionPageSceneObjects();
		refreshCollisionPageMotionWaypoints();
	}
	refreshRobotCoordinateFrameOverlays();
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instanceIndex);
	m_host->simulationCommandPage()->setRevoluteJointNames(doc->robotRevoluteJointNamesForInstance(instanceIndex));

	QStringList tcpLinks;
	QString preferredTcp;
	(void)UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferredTcp, nullptr);
	QStringList childLinks;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr);
	QSet<QString> uniq;
	if (!preferredTcp.isEmpty())
	{
		uniq.insert(preferredTcp);
		tcpLinks.push_back(preferredTcp);
	}
	for (const QString& l : childLinks)
	{
		if (l.isEmpty() || uniq.contains(l))
		{
			continue;
		}
		uniq.insert(l);
		tcpLinks.push_back(l);
	}
	m_host->simulationCommandPage()->setTcpLinkOptions(tcpLinks, preferredTcp);

	QVector<double> lower;
	QVector<double> upper;
	doc->robotJointLimitsForInstance(instanceIndex, lower, upper);
	const QStringList jn = doc->robotRevoluteJointNamesForInstance(instanceIndex);
	if (!jn.isEmpty() && lower.size() == jn.size() && upper.size() == jn.size())
	{
		m_host->robotAxisControlPage()->setJoints(jn, lower, upper);
	}
	syncRobotAxisControlExternalAxes(instanceIndex);
	{
		const int jointOffset = doc->robotJointOffsetInAggregatedVector(instanceIndex);
		const int nj = doc->robotRevoluteJointCountForInstance(instanceIndex);
		if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
		{
			const QVector<double> local = m_aggregatedJointAnglesRad.mid(jointOffset, nj);
			QSignalBlocker blocker(m_host->robotAxisControlPage());
			m_host->robotAxisControlPage()->setJointAnglesRad(local);
		}
	}
	captureMotionPreviewProgramStartJoints();
	m_host->invalidateInstructionPropertyCache();
	refreshRobotCoordinateFrameOverlays();
	refreshAxisControlTargets();
}

void RobotSimulationController::onRobotAxisJointAnglesChanged(const QVector<double>& jointAnglesRad)
{
	if (m_programExecutor.isRunning() || m_tcpDragApplyingIk)
	{
		return;
	}
	if (RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr)
	{
		if (axis->currentControlTarget().kind == AxisControlTargetKind::CustomDevice)
		{
			return;
		}
	}
	const bool tcpDragActive = m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode();
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	if (!doc || !poseSink)
	{
		return;
	}
	const int instIdx = axisControlActiveRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	if (m_aggregatedJointAnglesRad.size() != doc->robotRevoluteJointNames().size())
	{
		resyncAggregatedJointAnglesFromDocument(doc);
	}
	// 须走 doc->applyJointAnglesRad：KinematicsBatchScope + notify + Follow 批末求解
	if (!doc->applyJointAnglesRad(instIdx, jointAnglesRad, m_aggregatedJointAnglesRad))
	{
		return;
	}
	if (m_host->osgView())
	{
		m_host->osgView()->requestRedraw();
	}
	if (tcpDragActive)
	{
		syncTcpDragTeachAnchorFromCurrentJoints();
		refreshRobotCoordinateFrameOverlays();
		return;
	}
	refreshRobotCoordinateFrameOverlays();
	if (!m_suppressMotionPreviewStartCapture)
	{
		// 轴控拖动/「重置所有关节」不改写程序起点；否则零位会污染预览种子并牵动路点轴
		m_host->invalidateInstructionPropertyCache();
	}
}

void RobotSimulationController::syncRobotAxisControlExternalAxes(const int instanceIndex)
{
	RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!axis || !doc || instanceIndex < 0)
	{
		return;
	}
	if (axis->currentControlTarget().kind == AxisControlTargetKind::CustomDevice)
	{
		return;
	}
	const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instanceIndex);
	if (!RobotExternal::hasEnabledExternalAxes(set))
	{
		if (std::abs(doc->robotExternalAxisQMm(instanceIndex)) > 1e-9)
		{
			doc->setRobotExternalAxisQMm(instanceIndex, 0.0);
		}
		axis->clearExternalAxes();
		m_axisControlExternalQApplied.clear();
		m_axisControlExternalInstIdx = -1;
		return;
	}
	axis->setExternalAxes(set);
	const QVector<double> values = enabledValuesFromFullQ(set, doc->robotExternalAxisQ(instanceIndex));
	axis->setExternalAxisValuesSilent(values);
	m_axisControlExternalInstIdx = instanceIndex;
	m_axisControlExternalQApplied = values;
	if (const RobotExternal::RobotExternalAxisConfig* rail = RobotExternal::firstEnabledExternalAxis(set))
	{
		m_axisControlExternalAxis[0] = rail->axis[0];
		m_axisControlExternalAxis[1] = rail->axis[1];
		m_axisControlExternalAxis[2] = rail->axis[2];
	}
}

void RobotSimulationController::applyAxisControlExternalPose(const int instanceIndex, const QVector<double>& values)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || instanceIndex < 0)
	{
		return;
	}
	const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instanceIndex);
	const std::vector<double> fullQ = fullQFromEnabledValues(set, values);

	IRobotBackendPoseSink* sink = doc->poseSink();
	(void)RobotExternalAxisSceneApply::applyExternalAxisQ(doc, sink, instanceIndex, fullQ);

	QVector<double> joints = localJointAnglesForInstance(instanceIndex);
	if (joints.isEmpty() && m_host->robotAxisControlPage() &&
		m_host->robotAxisControlPage()->jointCount() == doc->robotRevoluteJointCountForInstance(instanceIndex))
	{
		joints = m_host->robotAxisControlPage()->jointAnglesRad();
	}
	if (!joints.isEmpty())
	{
		if (m_aggregatedJointAnglesRad.size() != doc->robotRevoluteJointNames().size())
		{
			resyncAggregatedJointAnglesFromDocument(doc);
		}
		(void)doc->applyJointAnglesRad(instanceIndex, joints, m_aggregatedJointAnglesRad);
	}

	m_axisControlExternalQApplied = values;
	m_axisControlExternalInstIdx = instanceIndex;
	if (const RobotExternal::RobotExternalAxisConfig* rail = RobotExternal::firstEnabledExternalAxis(set))
	{
		m_axisControlExternalAxis[0] = rail->axis[0];
		m_axisControlExternalAxis[1] = rail->axis[1];
		m_axisControlExternalAxis[2] = rail->axis[2];
	}
	if (m_host->osgView())
	{
		m_host->osgView()->requestRedraw();
	}
	refreshRobotCoordinateFrameOverlays();
}

void RobotSimulationController::applyExternalAxisFromPlan(const int instanceIndex,
														  const RobotInstruction::PlanResult& plan,
														  const RobotInstruction::Base* instruction,
														  const double progress01, const QVector<double>& segmentStartQs)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || instanceIndex < 0)
	{
		return;
	}
	const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instanceIndex);
	if (!RobotExternal::hasEnabledExternalAxes(set))
	{
		return;
	}

	std::vector<double> qeTarget;
	if (!plan.externalAxisQs.empty())
	{
		qeTarget = padExternalAxisQToConfig(set, plan.externalAxisQs);
	}
	else if (instruction)
	{
		const auto& ext = instruction->extensionProperties();
		const auto itCsv = ext.find(RobotExternal::kExtContextExternalAxisQCsv);
		if (itCsv != ext.end() && !itCsv->second.empty())
		{
			qeTarget = padExternalAxisQToConfig(set, RobotExternal::parseExternalAxisQCsv(itCsv->second));
		}
		else
		{
			const auto it = ext.find(RobotExternal::kExtContextExternalAxisQMm);
			bool haveQ = false;
			const double qe = (it != ext.end() && !it->second.empty())
								  ? QString::fromStdString(it->second).toDouble(&haveQ)
								  : 0.0;
			if (haveQ)
			{
				qeTarget = expandScalarExternalAxisQ(set, qe);
			}
		}
	}
	if (qeTarget.empty() && plan.hasExternalAxisQ)
	{
		qeTarget = expandScalarExternalAxisQ(set, plan.externalAxisQ);
	}
	if (qeTarget.empty())
	{
		return;
	}

	const double u = std::clamp(progress01, 0.0, 1.0);
	std::vector<double> qeFull = qeTarget;
	if (u < 1.0 - 1e-12)
	{
		std::vector<double> start =
			segmentStartQs.isEmpty() ? doc->robotExternalAxisQ(instanceIndex) : toStdVector(segmentStartQs);
		start = padExternalAxisQToConfig(set, start);
		qeFull.resize(set.axes.size());
		for (size_t i = 0; i < set.axes.size(); ++i)
		{
			const double s = i < start.size() ? start[i] : set.axes[i].home;
			const double t = i < qeTarget.size() ? qeTarget[i] : set.axes[i].home;
			qeFull[i] = s + (t - s) * u;
		}
	}

	const std::vector<double> qCur = doc->robotExternalAxisQ(instanceIndex);
	bool changed = qCur.size() != qeFull.size();
	if (!changed)
	{
		for (size_t i = 0; i < qeFull.size(); ++i)
		{
			if (std::abs(qCur[i] - qeFull[i]) >= 1e-6)
			{
				changed = true;
				break;
			}
		}
	}
	if (!changed)
	{
		return;
	}

	const QVector<double> values = enabledValuesFromFullQ(set, qeFull);
	if (RobotAxisControlWidget* axis = m_host->robotAxisControlPage())
	{
		if (axis->externalAxisCount() == values.size())
		{
			axis->setExternalAxisValuesSilent(values);
		}
	}
	applyAxisControlExternalPose(instanceIndex, values);
}

void RobotSimulationController::onRobotAxisExternalValuesChanged(const QVector<double>& values)
{
	if (m_programExecutor.isRunning() || m_tcpDragApplyingIk)
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc)
	{
		return;
	}
	RobotAxisControlWidget* axisPage = m_host->robotAxisControlPage();
	if (axisPage && axisPage->currentControlTarget().kind == AxisControlTargetKind::CustomDevice)
	{
		const QString deviceId = axisPage->currentControlTarget().id;
		const auto device =
			std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(deviceId.toStdString()));
		if (!device)
		{
			return;
		}
		const RobotExternal::RobotExternalAxisConfigSet ext =
			CustomDeviceKinematics::toExternalAxisConfigSet(device->axes());
		const std::vector<double> fullQ = fullQFromEnabledValues(ext, values);
		if (!KinematicModelApply::applyCustomDevice(KinematicModelRegistry::keyCustomDevice(deviceId.toStdString()),
													*device, &doc->backend(), doc->poseSink(), fullQ))
		{
			return;
		}
		if (m_host)
		{
			m_host->flushCustomDeviceLinkGeometryVisual(deviceId);
		}
		if (m_host->osgView())
		{
			m_host->osgView()->requestRedraw();
		}
		return;
	}
	const int instIdx = axisControlActiveRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	applyAxisControlExternalPose(instIdx, values);
}

// refreshAxisControlTargets / dock tabs / axis target → services

void RobotSimulationController::refreshAxisControlTargets()
{
	if (m_axisControlTargets)
	{
		m_axisControlTargets->refreshTargets();
	}
	// 组装提交等只刷轴目标时，信号 Owner 下拉也需跟上文档里的自定义设备
	syncIoOwnersFromDocument();
}

void RobotSimulationController::syncAxisControlTargetForDockMode(const SimulationDockMode mode)
{
	if (!m_host || !m_simulationDock)
	{
		return;
	}
	RobotAxisControlWidget* axis = m_host->robotAxisControlPage();
	IRobotDocumentHost* doc = m_host->document();
	if (!axis || !doc)
	{
		return;
	}
	if (mode == SimulationDockMode::CustomDevice)
	{
		for (const QString& id :
			 doc->documentData().findByClassName(QString::fromUtf8(backend_type::kClassCustomDevice)))
		{
			if (id.isEmpty())
			{
				continue;
			}
			axis->selectControlTarget(AxisControlTargetKind::CustomDevice, id);
			return;
		}
		return;
	}
	const int instIdx = m_host->currentSimulationRobotInstanceIndex();
	if (instIdx < 0 || instIdx >= doc->robotKinematicInstanceCount())
	{
		return;
	}
	axis->selectControlTarget(AxisControlTargetKind::RobotInstance, doc->robotSceneBackendIdForInstance(instIdx));
}

void RobotSimulationController::showRobotDockTab(const int tabIndex)
{
	if (m_dockNavigation)
	{
		m_dockNavigation->showRobotDockTab(tabIndex);
	}
}

void RobotSimulationController::showDeviceDockTab(const int tabIndex)
{
	if (m_dockNavigation)
	{
		m_dockNavigation->showDeviceDockTab(tabIndex);
	}
	if (m_simulationDock && m_simulationDock->dockMode() == SimulationDockMode::CustomDevice &&
		tabIndex == RobotSimulationDockWidget::kTabIndexDeviceAxisControl)
	{
		syncAxisControlTargetForDockMode(SimulationDockMode::CustomDevice);
	}
}

void RobotSimulationController::onAxisControlTargetChanged(const AxisControlTargetKind kind, const QString& id)
{
	if (m_axisControlTargets)
	{
		m_axisControlTargets->onTargetChanged(kind, id);
	}
}

void RobotSimulationController::clearReachableWorkspaceOverlayUi()
{
	++m_reachableWorkspaceJobToken;
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->clearReachableWorkspaceOverlay();
	}
}

namespace
{
struct ReachableWorkspaceJobInput
{
	QString urdfPath;
	RobotCoordinate::RobotCoordinateFrameSet frames;
	RobotCoordinate::RobotToolFrame tool;
	QVector<double> jointLower;
	QVector<double> jointUpper;
	RobotExternal::RobotExternalAxisConfigSet extSet;
	cloudsim::core::Mat4 p0World{};
	double cellSizeMm = 25.0;
	int sampleCount = 28000;
};

struct ReachableWorkspaceJobOutput
{
	RobotOsgUi::ReachableWorkspaceOverlay overlay;
};

double radicalInverseHalton(int base, int index)
{
	double f = 1.0;
	double r = 0.0;
	int i = index;
	while (i > 0)
	{
		f /= static_cast<double>(base);
		r += f * static_cast<double>(i % base);
		i /= base;
	}
	return r;
}

struct VoxelIjk
{
	int x = 0;
	int y = 0;
	int z = 0;
	bool operator==(const VoxelIjk& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelIjkHash
{
	size_t operator()(const VoxelIjk& v) const
	{
		size_t h = static_cast<size_t>(v.x) * 73856093u;
		h ^= static_cast<size_t>(v.y) * 19349663u;
		h ^= static_cast<size_t>(v.z) * 83492791u;
		return h;
	}
};

ReachableWorkspaceJobOutput computeReachableWorkspaceVoxels(const ReachableWorkspaceJobInput& in)
{
	ReachableWorkspaceJobOutput out;
	out.overlay.cellSizeMm = in.cellSizeMm;
	const int nj = in.jointLower.size();
	if (in.urdfPath.isEmpty() || nj <= 0 || in.jointUpper.size() != nj)
	{
		return out;
	}

	struct DofSpec
	{
		double lower = 0.0;
		double upper = 0.0;
		int armIndex = -1;
		int extConfigIndex = -1;
	};
	std::vector<DofSpec> dofs;
	dofs.reserve(static_cast<size_t>(nj) + in.extSet.axes.size());
	for (int j = 0; j < nj; ++j)
	{
		DofSpec d;
		d.lower = std::min(in.jointLower[j], in.jointUpper[j]);
		d.upper = std::max(in.jointLower[j], in.jointUpper[j]);
		d.armIndex = j;
		dofs.push_back(d);
	}
	for (size_t i = 0; i < in.extSet.axes.size(); ++i)
	{
		const RobotExternal::RobotExternalAxisConfig& a = in.extSet.axes[i];
		if (!a.enabled || a.attachment != RobotExternal::RobotExternalAttachment::RobotBase)
		{
			continue;
		}
		DofSpec d;
		d.lower = std::min(a.lower, a.upper);
		d.upper = std::max(a.lower, a.upper);
		d.extConfigIndex = static_cast<int>(i);
		dofs.push_back(d);
	}
	if (dofs.empty())
	{
		return out;
	}

	static const int kHaltonBases[] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37};
	const int dofN = static_cast<int>(dofs.size());
	const int samples = std::max(1000, in.sampleCount);
	const double cell = std::max(1.0, in.cellSizeMm);
	const int extN = static_cast<int>(in.extSet.axes.size());
	if (RobotCoordinate::effectiveFlangeLinkName(in.frames, in.tool).empty() && in.frames.flangeLinkName.empty())
	{
		return out;
	}

	std::unordered_set<VoxelIjk, VoxelIjkHash> voxels;
	voxels.reserve(static_cast<size_t>(samples / 4));

	for (int s = 0; s < samples; ++s)
	{
		QVector<double> qArm(nj, 0.0);
		std::vector<double> qExt(static_cast<size_t>(std::max(0, extN)), 0.0);
		for (size_t i = 0; i < in.extSet.axes.size(); ++i)
		{
			qExt[i] = std::clamp(in.extSet.axes[i].home, in.extSet.axes[i].lower, in.extSet.axes[i].upper);
		}

		// Halton 填满关节空间，避免正则网格在笛卡尔系成环
		for (int d = 0; d < dofN; ++d)
		{
			const int base = kHaltonBases[d % static_cast<int>(sizeof(kHaltonBases) / sizeof(kHaltonBases[0]))];
			const double u = radicalInverseHalton(base, s + 1);
			const double q = dofs[static_cast<size_t>(d)].lower +
							 u * (dofs[static_cast<size_t>(d)].upper - dofs[static_cast<size_t>(d)].lower);
			if (dofs[static_cast<size_t>(d)].armIndex >= 0)
			{
				qArm[dofs[static_cast<size_t>(d)].armIndex] = q;
			}
			else if (dofs[static_cast<size_t>(d)].extConfigIndex >= 0 &&
					 dofs[static_cast<size_t>(d)].extConfigIndex < extN)
			{
				qExt[static_cast<size_t>(dofs[static_cast<size_t>(d)].extConfigIndex)] = q;
			}
		}

		QHash<QString, osg::Matrixd> linkWorld;
		if (!UrdfRobotLoader::computeLinkWorldMatrices(in.urdfPath, qArm, linkWorld, nullptr))
		{
			continue;
		}
		const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(in.frames, in.tool);
		const QString flangeQ =
			flangeLink.empty() ? QString::fromStdString(in.frames.flangeLinkName) : QString::fromStdString(flangeLink);
		if (flangeQ.isEmpty() || !linkWorld.contains(flangeQ))
		{
			continue;
		}
		const BackendMat4 T_tool = RobotCoordinate::frameToMat4(in.tool.T_flange_tool);
		const engine::RigidTransform T_base_flange = engine::rigidTransformFromOsg(linkWorld.value(flangeQ));
		const engine::RigidTransform T_flange_tool = RobotCoordinate::rigidTransformFromBackendMat4(T_tool);
		const engine::RigidTransform T_base_tcp = engine::toolOriginFromFlange(T_base_flange, T_flange_tool);

		BackendMat4 peff = BackendMat4::identity();
		RobotExternal::composeBasePlacementWithExternalAxis(in.p0World.data(), in.extSet, qExt, peff.v);
		const engine::RigidTransform T_world_tcp =
			RobotCoordinate::rigidTransformFromBackendMat4(peff).composeColumn(T_base_tcp);
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		T_world_tcp.translationMm(x, y, z);
		VoxelIjk key;
		key.x = static_cast<int>(std::floor(x / cell));
		key.y = static_cast<int>(std::floor(y / cell));
		key.z = static_cast<int>(std::floor(z / cell));
		voxels.insert(key);
	}

	out.overlay.voxelCentersMm.reserve(voxels.size());
	for (const VoxelIjk& v : voxels)
	{
		cloudsim::core::Vec3 c;
		c.x = (static_cast<double>(v.x) + 0.5) * cell;
		c.y = (static_cast<double>(v.y) + 0.5) * cell;
		c.z = (static_cast<double>(v.z) + 0.5) * cell;
		out.overlay.voxelCentersMm.push_back(c);
	}
	return out;
}
} // namespace

void RobotSimulationController::startReachableWorkspaceCompute(const int instanceIndex)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr;
	if (!doc || !osg || !axis || instanceIndex < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instanceIndex);
	if (urdfPath.isEmpty())
	{
		axis->setReachableWorkspaceChecked(false);
		axis->setReachableWorkspaceBusy(false);
		return;
	}

	ReachableWorkspaceJobInput input;
	input.urdfPath = urdfPath;
	input.frames = doc->robotCoordinateFramesForInstance(instanceIndex);
	if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(input.frames))
	{
		input.tool = *tool;
	}
	else
	{
		input.tool.T_flange_tool = RobotCoordinate::identityRigidFrame();
	}
	doc->robotJointLimitsForInstance(instanceIndex, input.jointLower, input.jointUpper);
	input.extSet = doc->robotExternalAxesForInstance(instanceIndex);
	input.p0World = doc->robotBasePlacementWorldForInstance(instanceIndex);
	const double density01 =
		std::clamp(axis->reachableWorkspaceDensityPercent(), 1, 100) / 100.0;
	// 50% → ~28k 采样 / 25mm 体素；滑条同时调采样数与体素边长
	input.sampleCount = static_cast<int>(std::lround(6000.0 + density01 * 44000.0));
	input.cellSizeMm = 42.0 - density01 * 28.0;

	const quint64 token = ++m_reachableWorkspaceJobToken;
	axis->setReachableWorkspaceBusy(true);
	const auto jobOut = std::make_shared<ReachableWorkspaceJobOutput>();
	QPointer<RobotSimulationController> guard(this);
	m_host->enqueueBackgroundJob(
		QStringLiteral("Reachable workspace"),
		[input, jobOut]() { *jobOut = computeReachableWorkspaceVoxels(input); },
		[this, guard, token, jobOut](const bool threw, const QString&)
		{
			if (!guard || threw || token != m_reachableWorkspaceJobToken || !m_host)
			{
				return;
			}
			if (RobotAxisControlWidget* ax = m_host->robotAxisControlPage())
			{
				ax->setReachableWorkspaceBusy(false);
				if (!ax->isReachableWorkspaceChecked())
				{
					clearReachableWorkspaceOverlayUi();
					return;
				}
			}
			if (IRobotOsgViewHost* view = m_host->osgView())
			{
				view->setReachableWorkspaceOverlay(jobOut->overlay);
			}
		});
}

void RobotSimulationController::onReachableWorkspaceToggled(const bool enabled)
{
	if (!enabled)
	{
		clearReachableWorkspaceOverlayUi();
		if (RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr)
		{
			axis->setReachableWorkspaceBusy(false);
		}
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage())
	{
		if (RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr)
		{
			axis->setReachableWorkspaceChecked(false);
		}
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		if (RobotAxisControlWidget* axis = m_host->robotAxisControlPage())
		{
			axis->setReachableWorkspaceChecked(false);
		}
		return;
	}
	startReachableWorkspaceCompute(instIdx);
}

void RobotSimulationController::onReachableWorkspaceDensityChanged(int /*percent*/)
{
	RobotAxisControlWidget* axis = m_host ? m_host->robotAxisControlPage() : nullptr;
	if (!axis || !axis->isReachableWorkspaceChecked())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	startReachableWorkspaceCompute(instIdx);
}

namespace
{
/// 拖动 gizmo 相对 P_eff；示教 IK 相对 P0：T_p0 = E(q) * T_peff
engine::RigidTransform robotBaseExternalE(IRobotDocumentHost* doc, const int instIdx)
{
	BackendMat4 eMat = BackendMat4::identity();
	if (!doc)
	{
		return engine::RigidTransform::identity();
	}
	const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instIdx);
	const std::vector<double> qs = doc->robotExternalAxisQ(instIdx);
	RobotExternal::composeBasePlacementWithExternalAxis(BackendMat4::identity().v, set, qs, eMat.v);
	return RobotCoordinate::rigidTransformFromBackendMat4(eMat);
}

engine::RigidTransform tcpDragRigidPeffToP0(IRobotDocumentHost* doc, const int instIdx,
											const engine::RigidTransform& peff)
{
	if (!doc || !RobotExternal::hasEnabledExternalAxes(doc->robotExternalAxesForInstance(instIdx)))
	{
		return peff;
	}
	return robotBaseExternalE(doc, instIdx).composeColumn(peff);
}

/// 回写 gizmo：T_peff = inv(E) * T_p0
engine::RigidTransform tcpDragRigidP0ToPeff(IRobotDocumentHost* doc, const int instIdx,
											const engine::RigidTransform& p0)
{
	if (!doc || !RobotExternal::hasEnabledExternalAxes(doc->robotExternalAxesForInstance(instIdx)))
	{
		return p0;
	}
	return robotBaseExternalE(doc, instIdx).inverse().composeColumn(p0);
}

/// 沿各 RobotBase 平移轴投影位移，更新联立种子
void tcpDragProjectExternalSeed(IRobotDocumentHost* doc, const int instIdx, const double dxMm, const double dyMm,
								const double dzMm, std::vector<double>& qeInOut)
{
	if (!doc)
	{
		return;
	}
	const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instIdx);
	qeInOut.resize(set.axes.size());
	for (size_t i = 0; i < set.axes.size(); ++i)
	{
		const RobotExternal::RobotExternalAxisConfig& a = set.axes[i];
		if (!a.enabled || a.attachment != RobotExternal::RobotExternalAttachment::RobotBase ||
			a.motionType != RobotExternal::RobotExternalMotionType::Translate)
		{
			continue;
		}
		const double dAlong = dxMm * a.axis[0] + dyMm * a.axis[1] + dzMm * a.axis[2];
		qeInOut[i] = std::clamp(qeInOut[i] + dAlong, a.lower, a.upper);
	}
}
} // namespace

void RobotSimulationController::bindInstructionWaypointPickCallbacks()
{
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!osg)
	{
		return;
	}
	osg->setInstructionWaypointPickCallbacks(
		[this](const std::string& instructionId, bool isArcVia)
		{ onInstructionWaypointPicked(instructionId, isArcVia); },
		[this]()
		{
			if (SimulationCommandWidget* page = m_host ? m_host->simulationCommandPage() : nullptr)
			{
				page->setInstructionWaypointPickMode(false);
			}
		});
}

void RobotSimulationController::onInstructionWaypointPickModeChanged(const bool enabled)
{
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	SimulationCommandWidget* cmd = m_host ? m_host->simulationCommandPage() : nullptr;
	if (!osg || !cmd)
	{
		return;
	}
	if (!enabled)
	{
		osg->setInstructionWaypointPickMode(false);
		return;
	}
	if (m_programExecutor.isRunning())
	{
		cmd->setInstructionWaypointPickMode(false);
		return;
	}
	if (cmd->tcpDragTeachMode())
	{
		onSimulationTcpDragTeachModeChanged(false);
		cmd->setTcpDragTeachMode(false);
	}
	// 先开拾取再清对象选：clearTree 会级联 clearInstructionPoseAxes，须已被 mode 护住
	bindInstructionWaypointPickCallbacks();
	osg->setInstructionWaypointPickMode(true);
	m_host->clearBackendObjectSelection(false);
	if (osg->objectSelectionMode())
	{
		osg->setObjectSelectionMode(false);
	}
	refreshInstructionPoseAxes(false);
	scheduleInstructionPoseAxesRefresh(false);
}

void RobotSimulationController::onInstructionWaypointPicked(const std::string& instructionId, bool isArcVia)
{
	if (instructionId.empty() || !m_host)
	{
		return;
	}
	SimulationCommandWidget* cmd = m_host->simulationCommandPage();
	if (!cmd)
	{
		return;
	}
	for (const std::shared_ptr<RobotInstruction::Base>& ins : cmd->instructionList())
	{
		if (!ins || ins->id() != instructionId)
		{
			continue;
		}
		m_waypointPickHighlightInstructionId = instructionId;
		m_waypointPickHighlightPreferVia = isArcVia && ins->type() == RobotInstruction::Type::ARC;
		m_waypointLabelOverlayFingerprint.clear();
		if (InstructionProgramTreeWidget* tree = cmd->instructionTree())
		{
			tree->selectInstructionByRaw(ins.get());
			if (QTreeWidgetItem* cur = tree->currentItem())
			{
				tree->scrollToItem(cur);
			}
		}
		// 同指令 via/终点切换时树选中不变，须主动刷新游标
		refreshPlaybackPathOverlays(ins);
		return;
	}
}

void RobotSimulationController::onSimulationTcpDragTeachModeChanged(const bool enabled)
{
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!m_host->simulationCommandPage())
	{
		return;
	}
	if (!enabled)
	{
		if (osg && osg->isTcpDragTeachActive())
		{
			osg->endTcpDragTeach();
		}
		syncTcpDragExitJointState();
		m_tcpDragTeachFlangeLink.clear();
		m_tcpDragLastAppliedJointRad.clear();
		m_lastTcpDragTargetValid = false;
		m_tcpDragIkPosePending = false;
		m_tcpDragIkFlushScheduled = false;
		if (IRobotDocumentHost* docOff = m_host ? m_host->document() : nullptr)
		{
			docOff->setSuppressRobotFollowDirtyNotify(false);
			docOff->requestFollowSolveForced();
		}
		if (m_host)
		{
			m_host->runFollowSolveAndSyncForCurrentDocument();
			syncTcpDragExitJointState();
		}
		return;
	}
	if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->instructionWaypointPickMode())
	{
		if (osg)
		{
			osg->setInstructionWaypointPickMode(false);
		}
		m_host->simulationCommandPage()->setInstructionWaypointPickMode(false);
	}
	if (m_programExecutor.isRunning())
	{
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("Stop simulation before TCP drag teach."), QStringLiteral("请先停止仿真，再使用末端拖动示教。")));
		}
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !osg || !doc->hasRobotSimulationContext() || !m_host->robotAxisControlPage())
	{
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		return;
	}
	m_host->clearBackendObjectSelection(true);
	if (osg->objectSelectionMode())
	{
		osg->setObjectSelectionMode(false);
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	const QString robotRootId = doc->robotSceneBackendIdForInstance(instIdx);
	if (urdfPath.isEmpty() || robotRootId.isEmpty())
	{
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		return;
	}
	QStringList revoluteChildLinks;
	QString fallbackFlange;
	(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
	if (!revoluteChildLinks.isEmpty())
	{
		fallbackFlange = revoluteChildLinks.back();
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	const bool perLink = doc->robotUsesPerLinkBackendsForInstance(instIdx);
	bool meshVerticesInLinkFrame = true;
	if (perLink)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto plSlice;
		if (doc->robotPerLinkKinematicsForInstance(instIdx, plSlice))
		{
			meshVerticesInLinkFrame = plSlice.meshVerticesInLinkFrame;
		}
	}
	engine::RigidTransform targetInBase{};
	QString flangeLinkQ;
	if (const RobotCoordinate::RobotToolFrame* activeTool = RobotCoordinate::activeToolFrame(frames))
	{
		flangeLinkQ = QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *activeTool));
	}
	const QVector<double> axisJointQ = m_host->robotAxisControlPage()->jointAnglesRad();
	// 目标只用连杆系 FK。装配系 mesh 的场景矩阵≠连杆系，当法兰捕获会把罗盘算到世界原点附近
	if (!RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
			urdfPath, axisJointQ, frames, flangeLinkQ.isEmpty() ? fallbackFlange : flangeLinkQ, targetInBase,
			&flangeLinkQ, nullptr))
	{
		RobotInstruction::Vec3 pose{};
		RobotInstruction::Vec3 euler{};
		QString err;
		if (!tryCaptureCurrentRobotTcpPose(pose, euler, nullptr, nullptr, nullptr, &err))
		{
			if (m_host->runInfoPage())
			{
				m_host->appendRunWarning(err);
			}
			m_host->simulationCommandPage()->setTcpDragTeachMode(false);
			return;
		}
		targetInBase =
			engine::RigidTransform::fromTranslationEulerDeg(pose.x, pose.y, pose.z, euler.x, euler.y, euler.z);
		flangeLinkQ = RobotSimulationMath::defaultTcpLinkNameForUrdf(
			urdfPath, m_host->simulationCommandPage()->selectedTcpLink());
	}
	m_tcpDragTeachFlangeLink = flangeLinkQ;
	float modelDiag = 1000.0f;
	if (const QString rootBid = robotRootId; osg->hasBackendObjectBranch(rootBid.toStdString()))
	{
		double cx = 0.0;
		double cy = 0.0;
		double cz = 0.0;
		if (osg->tryGetBackendModelCenterMm(rootBid.toStdString(), cx, cy, cz))
		{
			(void)cx;
			(void)cy;
			(void)cz;
		}
	}
	(void)modelDiag;
	std::string mountBackendId = robotRootId.toStdString();
	bool mountOnFlange = false;
	// 与工具轴叠加一致：per-link 一律挂法兰 mesh，文件系顶点用 inv(visual) 对齐连杆原点
	if (perLink && !m_tcpDragTeachFlangeLink.isEmpty())
	{
		const std::string flangeId =
			RobotSimulationMath::linkMeshBackendIdForInstance(doc, instIdx, m_tcpDragTeachFlangeLink.toStdString())
				.toStdString();
		if (!flangeId.empty() && osg->hasBackendObjectBranch(flangeId))
		{
			mountBackendId = flangeId;
			mountOnFlange = true;
		}
	}
	if (!mountOnFlange && !osg->hasBackendObjectBranch(mountBackendId))
	{
		if (!m_tcpDragTeachFlangeLink.isEmpty())
		{
			const std::string flangeId =
				RobotSimulationMath::linkMeshBackendIdForInstance(doc, instIdx, m_tcpDragTeachFlangeLink.toStdString())
					.toStdString();
			if (!flangeId.empty() && osg->hasBackendObjectBranch(flangeId))
			{
				mountBackendId = flangeId;
				mountOnFlange = true;
			}
		}
		if (!osg->hasBackendObjectBranch(mountBackendId))
		{
			const QString refBackendId = doc->robotFrameWorldReferenceBackendId(instIdx);
			if (!refBackendId.isEmpty() && osg->hasBackendObjectBranch(refBackendId.toStdString()))
			{
				mountBackendId = refBackendId.toStdString();
			}
		}
	}
	std::function<bool(cloudsim::core::Mat4&)> resolveRobotBaseWorld;
	cloudsim::core::Mat4 toolLocalOnFlange;
	const cloudsim::core::Mat4* toolLocalPtr = nullptr;
	const auto makeResolveRobotBaseWorld = [this, instIdx](cloudsim::core::Mat4& outWorld) -> bool
	{
		// 退出/析构期 document()/osg 可能已空，勿再进 FK
		IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
		IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
		if (!doc || !osg)
		{
			return false;
		}
		QVector<double> jointQ = localJointAnglesForInstance(instIdx);
		osg::Matrixd world;
		if (!RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, osg, instIdx, world,
																  jointQ.isEmpty() ? nullptr : &jointQ))
		{
			return false;
		}
		outWorld = RobotSimulationMath::coreMat4FromOsgMatrix(world);
		return true;
	};
	if (mountOnFlange)
	{
		resolveRobotBaseWorld = makeResolveRobotBaseWorld;
		osg::Matrixd toolLinkLocal;
		toolLinkLocal.makeIdentity();
		if (const RobotCoordinate::RobotToolFrame* activeTool = RobotCoordinate::activeToolFrame(frames))
		{
			toolLinkLocal = RobotSimulationMath::osgMatrixFromRobotRigidFrame(activeTool->T_flange_tool);
		}
		toolLocalOnFlange = RobotSimulationMath::coreMat4FromOsgMatrix(RobotSimulationMath::linkFrameLocalOnMeshBackend(
			urdfPath, m_tcpDragTeachFlangeLink, toolLinkLocal, meshVerticesInLinkFrame));
		toolLocalPtr = &toolLocalOnFlange;
	}
	else if (mountBackendId != robotRootId.toStdString())
	{
		resolveRobotBaseWorld = makeResolveRobotBaseWorld;
	}
	m_tcpDragLastAppliedJointRad.clear();
	m_lastTcpDragTargetValid = false;
	if (doc->robotUsesPerLinkBackendsForInstance(instIdx))
	{
		const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
		const int njInst = doc->robotRevoluteJointCountForInstance(instIdx);
		// reconcile 必须与场景一致：用轴控角（与罗盘目标同源），勿混用删机残留的聚合角
		QVector<double> jointQ;
		if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == njInst)
		{
			jointQ = m_host->robotAxisControlPage()->jointAnglesRad();
		}
		else if (njInst > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + njInst)
		{
			jointQ.resize(njInst);
			for (int j = 0; j < njInst; ++j)
			{
				jointQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
			}
		}
		if (jointQ.size() == njInst)
		{
			if (m_aggregatedJointAnglesRad.size() != doc->robotRevoluteJointNames().size())
			{
				resyncAggregatedJointAnglesFromDocument(doc);
			}
			for (int j = 0; j < njInst && jointOffset + j < m_aggregatedJointAnglesRad.size(); ++j)
			{
				m_aggregatedJointAnglesRad[jointOffset + j] = jointQ[j];
			}
			doc->reconcilePerLinkOuterBindFromScene(instIdx, jointQ);
		}
	}
	doc->setSuppressRobotFollowDirtyNotify(false);
	doc->clearFollowDirtyBackendIds();
	osg->beginTcpDragTeach(mountBackendId, targetInBase, modelDiag, resolveRobotBaseWorld, toolLocalPtr);
	if (!osg->isTcpDragTeachActive())
	{
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("Failed to attach TCP drag gizmo."), QStringLiteral("无法挂载 TCP 拖动示教罗盘。")));
		}
	}
	else if (m_host->runInfoPage())
	{
		m_host->appendRunInfo(m_host->i18n(
			QStringLiteral("TCP drag teach active | flange=%1 | mount=%2")
				.arg(m_tcpDragTeachFlangeLink, QString::fromStdString(mountBackendId)),
			QStringLiteral("末端拖动示教已开启 | 法兰=%1 | 挂载=%2")
				.arg(m_tcpDragTeachFlangeLink, QString::fromStdString(mountBackendId))));
	}
	// 法兰路径会按场景挂载点校准 T_base；缓存须读回，勿用进入前的纯 FK
	m_lastTcpDragTargetInBase = tcpDragRigidPeffToP0(doc, instIdx, osg->tcpDragTeachTargetInBase());
	m_lastTcpDragTargetValid = true;
	m_tcpDragTeachIkTimer.start();
}

void RobotSimulationController::syncTcpDragTeachAnchorFromCurrentJoints()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage() ||
		!osg->isTcpDragTeachActive() || m_tcpDragTeachFlangeLink.isEmpty())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	engine::RigidTransform fkTarget{};
	if (!RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
			urdfPath, m_host->robotAxisControlPage()->jointAnglesRad(), frames, m_tcpDragTeachFlangeLink, fkTarget,
			nullptr, nullptr))
	{
		return;
	}
	osg->updateTcpDragTeachFromTarget(fkTarget, true);
	m_lastTcpDragTargetInBase = tcpDragRigidPeffToP0(doc, instIdx, osg->tcpDragTeachTargetInBase());
	m_lastTcpDragTargetValid = true;
	m_tcpDragLastAppliedJointRad = localJointAnglesForInstance(instIdx);
}

bool RobotSimulationController::applyTcpDragTeachIkFromPose(const double pxMm, const double pyMm, const double pzMm,
															const double exDeg, const double eyDeg, const double ezDeg)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage() ||
		m_programExecutor.isRunning())
	{
		return false;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty() || m_tcpDragTeachFlangeLink.isEmpty())
	{
		return false;
	}
	m_tcpDragTeachIkTimer.restart();
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const int njInst = doc->robotRevoluteJointCountForInstance(instIdx);
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);

	// IK 种子取当前实例在聚合角中的切片
	QVector<double> seedQ;
	if (njInst > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + njInst)
	{
		seedQ.resize(njInst);
		for (int j = 0; j < njInst; ++j)
		{
			seedQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
	}
	else
	{
		seedQ = m_host->robotAxisControlPage()->jointAnglesRad();
	}

	// 示教目标用四元数真值（gizmo 侧），避免信号里欧拉往返改姿态；IK 接口仍吃欧拉
	engine::RigidTransform targetPeff =
		engine::RigidTransform::fromTranslationEulerDeg(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
	if (osg->isTcpDragTeachActive())
	{
		targetPeff = osg->tcpDragTeachTargetInBase();
	}
	const engine::RigidTransform targetFromEmit = tcpDragRigidPeffToP0(doc, instIdx, targetPeff);
	const bool hadPrevTarget = m_lastTcpDragTargetValid;
	const engine::RigidTransform prevTarget = m_lastTcpDragTargetInBase;
	Eigen::Vector3d tStep = targetFromEmit.translationMm();
	const Eigen::Quaterniond rStep = targetFromEmit.rotation().normalized();
	// 跟手：单步追赶加大，避免罗盘已走远而臂还在小台阶上爬
	static constexpr double kTcpDragMaxChaseMmPerIk = 220.0;
	bool chaseClipped = false;
	if (hadPrevTarget)
	{
		const Eigen::Vector3d tPrev = prevTarget.translationMm();
		const Eigen::Vector3d d = tStep - tPrev;
		const double emitLen = d.norm();
		if (emitLen > kTcpDragMaxChaseMmPerIk)
		{
			chaseClipped = true;
			tStep = tPrev + d * (kTcpDragMaxChaseMmPerIk / emitLen);
		}
	}
	const engine::RigidTransform targetForIkStep = engine::RigidTransform::fromTranslationQuat(tStep, rStep);
	double ikPx = 0.0;
	double ikPy = 0.0;
	double ikPz = 0.0;
	double ikEx = 0.0;
	double ikEy = 0.0;
	double ikEz = 0.0;
	targetForIkStep.translationMm(ikPx, ikPy, ikPz);
	targetForIkStep.eulerDegForDisplay(ikEx, ikEy, ikEz);

	// 多轴联立种子：沿各 RobotBase 平移轴投影本步位移
	std::vector<double> qeSeed = doc->robotExternalAxisQ(instIdx);
	bool hasQeSeed = RobotExternal::hasEnabledExternalAxes(doc->robotExternalAxesForInstance(instIdx));
	if (hasQeSeed && hadPrevTarget)
	{
		const Eigen::Vector3d tPrev = prevTarget.translationMm();
		tcpDragProjectExternalSeed(doc, instIdx, ikPx - tPrev.x(), ikPy - tPrev.y(), ikPz - tPrev.z(), qeSeed);
	}

	const auto ikResult = doc->solveTcpDragTeachIk(instIdx, ikPx, ikPy, ikPz, ikEx, ikEy, ikEz, seedQ,
												   m_tcpDragTeachFlangeLink, qeSeed, hasQeSeed);
	if (!ikResult.ok)
	{
		if (m_host && m_host->runInfoPage())
		{
			static QElapsedTimer ikFailWarnTimer;
			if (!ikFailWarnTimer.isValid() || ikFailWarnTimer.elapsed() >= 800)
			{
				ikFailWarnTimer.start();
				const QString detail =
					ikResult.error.isEmpty() ? QStringLiteral("IK solve failed") : ikResult.error;
				m_host->appendRunWarning(
					m_host->i18n(QStringLiteral("TCP drag IK failed (flange=%1): %2")
									 .arg(m_tcpDragTeachFlangeLink, detail),
								 QStringLiteral("末端拖动 IK 失败（法兰=%1）：%2")
									 .arg(m_tcpDragTeachFlangeLink, detail)));
			}
		}
		return false;
	}
	QVector<double> qRad = ikResult.jointRad;
	wrapJointAnglesTowardSeed(qRad, seedQ);
	QVector<double> qClamped = clampJointAnglesToInstanceLimits(doc, instIdx, qRad);
	const bool anyClamped =
		(qClamped.size() == qRad.size()) && !std::equal(qClamped.begin(), qClamped.end(), qRad.begin());
	const bool hasPrevDragQ = (m_tcpDragLastAppliedJointRad.size() == qClamped.size());
	const double maxDeltaIk = hasPrevDragQ ? maxJointDeltaRad(qClamped, m_tcpDragLastAppliedJointRad) : 1.0;
	static constexpr double kTcpDragMaxJointStepRad = 0.45;
	bool jointStepClipped = false;
	if (hasPrevDragQ)
	{
		jointStepClipped = (maxDeltaIk > kTcpDragMaxJointStepRad + 1e-9);
		qClamped = clampJointStepFromPrevious(qClamped, m_tcpDragLastAppliedJointRad, kTcpDragMaxJointStepRad);
	}
	const double maxJointDelta = hasPrevDragQ ? maxJointDeltaRad(qClamped, m_tcpDragLastAppliedJointRad) : 1.0;
	const std::vector<double> qeOldFull = doc->robotExternalAxisQ(instIdx);
	const RobotExternal::RobotExternalAxisConfigSet& extSet = doc->robotExternalAxesForInstance(instIdx);
	std::vector<double> qeNewFull = qeOldFull;
	bool haveExtUpdate = false;
	if (!ikResult.externalAxisQs.empty())
	{
		qeNewFull = padExternalAxisQToConfig(extSet, ikResult.externalAxisQs);
		haveExtUpdate = true;
	}
	else if (ikResult.hasExternalAxisQ)
	{
		qeNewFull = expandScalarExternalAxisQ(extSet, ikResult.externalAxisQ);
		haveExtUpdate = true;
	}
	static constexpr double kTcpDragMaxExtStepMm = 120.0;
	static constexpr double kTcpDragMaxExtStepRad = 0.45;
	if (haveExtUpdate)
	{
		qeNewFull.resize(extSet.axes.size());
		for (size_t i = 0; i < extSet.axes.size(); ++i)
		{
			const double oldV = i < qeOldFull.size() ? qeOldFull[i] : extSet.axes[i].home;
			double& newV = qeNewFull[i];
			const double dQ = newV - oldV;
			const double maxStep = (extSet.axes[i].motionType == RobotExternal::RobotExternalMotionType::Rotate)
									  ? kTcpDragMaxExtStepRad
									  : kTcpDragMaxExtStepMm;
			if (std::abs(dQ) > maxStep)
			{
				newV = oldV + (dQ > 0.0 ? maxStep : -maxStep);
			}
		}
	}
	static constexpr double kTcpDragMinJointApplyRad = 0.002;
	static constexpr double kTcpDragMinExtApplyMm = 0.2;
	static constexpr double kTcpDragMinExtApplyRad = 0.002;
	auto extDeltaSignificant = [&]() -> bool
	{
		if (!haveExtUpdate)
		{
			return false;
		}
		for (size_t i = 0; i < extSet.axes.size(); ++i)
		{
			if (!extSet.axes[i].enabled)
			{
				continue;
			}
			const double oldV = i < qeOldFull.size() ? qeOldFull[i] : extSet.axes[i].home;
			const double newV = i < qeNewFull.size() ? qeNewFull[i] : oldV;
			const double thr = (extSet.axes[i].motionType == RobotExternal::RobotExternalMotionType::Rotate)
								  ? kTcpDragMinExtApplyRad
								  : kTcpDragMinExtApplyMm;
			if (std::abs(newV - oldV) >= thr)
			{
				return true;
			}
		}
		return false;
	};
	const bool extMoved = extDeltaSignificant();
	if (maxJointDelta < kTcpDragMinJointApplyRad && !extMoved)
	{
		return true;
	}
	m_tcpDragLastAppliedJointRad = qClamped;
	if (m_aggregatedJointAnglesRad.size() != doc->robotRevoluteJointNames().size())
	{
		resyncAggregatedJointAnglesFromDocument(doc);
	}
	for (int j = 0; j < njInst && jointOffset + j < m_aggregatedJointAnglesRad.size(); ++j)
	{
		m_aggregatedJointAnglesRad[jointOffset + j] = qClamped[j];
	}
	m_tcpDragApplyingIk = true;
	m_suppressMotionPreviewStartCapture = true;
	if (haveExtUpdate)
	{
		const QVector<double> enabledVals = enabledValuesFromFullQ(extSet, qeNewFull);
		if (RobotAxisControlWidget* axisPage = m_host->robotAxisControlPage())
		{
			if (axisPage->externalAxisCount() == enabledVals.size())
			{
				axisPage->setExternalAxisValuesSilent(enabledVals);
			}
		}
		applyAxisControlExternalPose(instIdx, enabledVals);
	}
	QString applyFkErr;
	if (!doc->applyJointAnglesRad(instIdx, qClamped, m_aggregatedJointAnglesRad, &applyFkErr))
	{
		if (m_host && m_host->runInfoPage())
		{
			static QElapsedTimer fkFailWarnTimer;
			if (!fkFailWarnTimer.isValid() || fkFailWarnTimer.elapsed() >= 800)
			{
				fkFailWarnTimer.start();
				const QString detail =
					applyFkErr.isEmpty() ? QStringLiteral("applyJointAnglesRad failed") : applyFkErr;
				m_host->appendRunWarning(
					m_host->i18n(QStringLiteral("TCP drag FK apply failed: %1").arg(detail),
								 QStringLiteral("末端拖动 FK 写回失败：%1").arg(detail)));
			}
		}
		m_suppressMotionPreviewStartCapture = false;
		m_tcpDragApplyingIk = false;
		return false;
	}
	if (m_host)
	{
		doc->requestFollowSolveForced();
		m_host->runFollowSolveAndSyncForCurrentDocument();
	}
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == njInst)
	{
		m_host->robotAxisControlPage()->setJointAnglesRadSilent(qClamped);
	}
	if (anyClamped && m_host->runInfoPage())
	{
		static QElapsedTimer clampWarnTimer;
		if (!clampWarnTimer.isValid() || clampWarnTimer.elapsed() >= 800)
		{
			clampWarnTimer.start();
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("TCP drag IK exceeded joint limits; angles were clamped to URDF range."),
							 QStringLiteral("末端拖动 IK 超关节限位，已钳制到 URDF 范围。")));
		}
	}
	m_suppressMotionPreviewStartCapture = false;
	m_tcpDragApplyingIk = false;
	engine::RigidTransform fkPeff{};
	if (RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(
			urdfPath, qClamped, frames, m_tcpDragTeachFlangeLink, fkPeff, nullptr, nullptr))
	{
		m_lastTcpDragTargetInBase = tcpDragRigidPeffToP0(doc, instIdx, fkPeff);
	}
	else
	{
		m_lastTcpDragTargetInBase = targetFromEmit;
	}
	m_lastTcpDragTargetValid = true;
	// 罗盘仍跟鼠标目标；追逐锚点用 FK，避免 IK 残差累积导致跳动
	osg->updateTcpDragTeachFromTarget(tcpDragRigidP0ToPeff(doc, instIdx, targetFromEmit), false);
	static constexpr int kTcpDragOverlayMinIntervalMs = 50;
	if (!m_tcpDragOverlayRefreshTimer.isValid() ||
		m_tcpDragOverlayRefreshTimer.elapsed() >= kTcpDragOverlayMinIntervalMs)
	{
		refreshRobotCoordinateFrameOverlays();
		m_tcpDragOverlayRefreshTimer.restart();
	}
	osg->requestRedraw();
	// 鼠标停住时仍继续追赶，避免停在 chase/关节台阶上
	if (chaseClipped || jointStepClipped)
	{
		m_tcpDragIkPendingPx = pxMm;
		m_tcpDragIkPendingPy = pyMm;
		m_tcpDragIkPendingPz = pzMm;
		m_tcpDragIkPendingEx = exDeg;
		m_tcpDragIkPendingEy = eyDeg;
		m_tcpDragIkPendingEz = ezDeg;
		m_tcpDragIkPosePending = true;
		if (!m_tcpDragIkFlushScheduled)
		{
			m_tcpDragIkFlushScheduled = true;
			QTimer::singleShot(8, this,
							   [this]()
							   {
								   m_tcpDragIkFlushScheduled = false;
								   flushTcpDragTeachIkPending();
							   });
		}
	}
	return true;
}

void RobotSimulationController::flushTcpDragTeachIkPending()
{
	if (!m_tcpDragIkPosePending)
	{
		m_tcpDragIkFlushScheduled = false;
		return;
	}
	static constexpr int kTcpDragIkMinIntervalMs = 8;
	if (m_tcpDragTeachIkTimer.isValid() && m_tcpDragTeachIkTimer.elapsed() < kTcpDragIkMinIntervalMs)
	{
		if (!m_tcpDragIkFlushScheduled)
		{
			m_tcpDragIkFlushScheduled = true;
			const int waitMs =
				std::max(1, kTcpDragIkMinIntervalMs - static_cast<int>(m_tcpDragTeachIkTimer.elapsed()));
			QTimer::singleShot(waitMs, this,
							   [this]()
							   {
								   m_tcpDragIkFlushScheduled = false;
								   flushTcpDragTeachIkPending();
							   });
		}
		return;
	}
	m_tcpDragIkPosePending = false;
	m_tcpDragIkFlushScheduled = false;
	(void)applyTcpDragTeachIkFromPose(m_tcpDragIkPendingPx, m_tcpDragIkPendingPy, m_tcpDragIkPendingPz,
									  m_tcpDragIkPendingEx, m_tcpDragIkPendingEy, m_tcpDragIkPendingEz);
}

void RobotSimulationController::onTcpDragTeachPoseChanged(const double pxMm, const double pyMm, const double pzMm,
														  const double exDeg, const double eyDeg, const double ezDeg)
{
	m_tcpDragIkPendingPx = pxMm;
	m_tcpDragIkPendingPy = pyMm;
	m_tcpDragIkPendingPz = pzMm;
	m_tcpDragIkPendingEx = exDeg;
	m_tcpDragIkPendingEy = eyDeg;
	m_tcpDragIkPendingEz = ezDeg;
	m_tcpDragIkPosePending = true;
	flushTcpDragTeachIkPending();
}

void RobotSimulationController::syncTcpDragExitJointState()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !m_host->robotAxisControlPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	const QVector<double> local = localJointAnglesForInstance(instIdx);
	if (local.isEmpty())
	{
		return;
	}
	IRobotBackendPoseSink* poseSink = doc->poseSink();
	if (poseSink)
	{
		m_tcpDragApplyingIk = true;
		m_suppressMotionPreviewStartCapture = true;
		(void)doc->applyJointAnglesRad(instIdx, local, m_aggregatedJointAnglesRad);
		m_suppressMotionPreviewStartCapture = false;
		m_tcpDragApplyingIk = false;
		if (m_host)
		{
			doc->requestFollowSolveForced();
			m_host->runFollowSolveAndSyncForCurrentDocument();
		}
	}
	if (m_host->robotAxisControlPage()->jointCount() == local.size())
	{
		m_host->robotAxisControlPage()->setJointAnglesRadSilent(local);
	}
	refreshRobotCoordinateFrameOverlays();
	if (IRobotOsgViewHost* osg = m_host->osgView())
	{
		osg->requestRedraw();
	}
}

void RobotSimulationController::onTcpDragTeachEnded()
{
	// Esc / 删挂载节点等路径已 endTcpDragTeach，这里对齐 UI 与 pending IK
	if (m_host && m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode())
	{
		onSimulationTcpDragTeachModeChanged(false);
		m_host->simulationCommandPage()->setTcpDragTeachMode(false);
	}
	else
	{
		m_tcpDragIkPosePending = false;
		m_tcpDragIkFlushScheduled = false;
		m_tcpDragTeachFlangeLink.clear();
		m_tcpDragLastAppliedJointRad.clear();
		m_lastTcpDragTargetValid = false;
	}
	if (m_host && m_host->simulationCommandPage() && m_host->document())
	{
		const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
		const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
			m_host->simulationCommandPage()->instructions(robotBackendId);
		if (RobotInstruction::collectMotionInstructions(program).empty())
		{
			captureMotionPreviewProgramStartJoints();
		}
	}
}

void RobotSimulationController::onSimulationStopRequested()
{
	stopRobotSimulation();
	if (m_host->runInfoPage())
	{
		m_host->appendRunInfo(m_host->i18n(QStringLiteral("Simulation stopped."), QStringLiteral("仿真已停止。")));
	}
}

void RobotSimulationController::onSimulationRunRequested()
{
	onSimulationStartTriggered();
}

void RobotSimulationController::onSimulationExportRequested()
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("Import a robot (URDF) first, then export the program."), QStringLiteral("请先导入机器人(URDF)，再导出程序。")));
		}
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}

	const auto& catalog = doc->robotProgramStore().activeCatalog();
	const QString activeProgramId = QString::fromStdString(catalog.activeProgramId());
	QVector<RobotWidget::BrandExportProgramItem> programItems;
	programItems.reserve(static_cast<int>(catalog.programs().size()));
	for (const RobotInstruction::RobotProgram& prog : catalog.programs())
	{
		programItems.push_back({QString::fromStdString(prog.id), QString::fromStdString(prog.name)});
	}
	if (programItems.isEmpty())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("No program to export."), QStringLiteral("没有可导出的程序。")));
		}
		return;
	}

	// 品牌/脚本/程序选择对话框
	RobotWidget::BrandProgramExportDialog brandDlg(programItems, activeProgramId);
	if (brandDlg.exec() != QDialog::Accepted)
	{
		return;
	}
	const RobotWidget::BrandExportChoice brand = brandDlg.selectedBrand();
	if (brand.brandId.isEmpty() || brand.scriptStem.isEmpty())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("Invalid brand selection."), QStringLiteral("品牌选择无效。")));
		}
		return;
	}

	RobotInstruction::RobotProgram* exportProg =
		doc->robotProgramStore().activeCatalog().findProgram(brandDlg.selectedProgramId().toStdString());
	if (!exportProg)
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("Selected program not found."), QStringLiteral("未找到所选程序。")));
		}
		return;
	}
	if (RobotInstruction::collectMotionInstructions(exportProg->steps).empty())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("No motion instructions to export."), QStringLiteral("没有可导出的运动指令。")));
		}
		return;
	}

	QString defaultBase = brandDlg.selectedProgramName().trimmed();
	if (defaultBase.isEmpty())
	{
		defaultBase = QStringLiteral("program_export");
	}
	for (QChar& ch : defaultBase)
	{
		if (ch == QLatin1Char('/') || ch == QLatin1Char('\\') || ch == QLatin1Char(':') || ch == QLatin1Char('*') ||
			ch == QLatin1Char('?') || ch == QLatin1Char('"') || ch == QLatin1Char('<') || ch == QLatin1Char('>') ||
			ch == QLatin1Char('|'))
		{
			ch = QLatin1Char('_');
		}
	}
	const QString defaultName = defaultBase + brand.defaultExt;
	const QString filter =
		brand.filter + QStringLiteral(";;") +
		m_host->i18n(QStringLiteral("All files (*.*)"), QStringLiteral("所有文件 (*.*)"));
	const QString outPath = QFileDialog::getSaveFileName(
		nullptr, m_host->i18n(QStringLiteral("Save brand robot program"), QStringLiteral("保存品牌机器人程序")),
		defaultName, filter);
	if (outPath.isEmpty())
	{
		return;
	}

	const QString scriptPath = QDir(QCoreApplication::applicationDirPath())
								   .filePath(QStringLiteral("resource/Python/ExportPython/%1.py").arg(brand.scriptStem));
	if (!QFile::exists(scriptPath))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("Brand export script not found: %1").arg(scriptPath), QStringLiteral("未找到品牌导出脚本：%1").arg(scriptPath)));
		}
		return;
	}

	QApplication::setOverrideCursor(Qt::WaitCursor);
	struct CursorGuard
	{
		~CursorGuard() { QApplication::restoreOverrideCursor(); }
	} cursorGuard;

	RobotCanonicalExport::InstructionRuntimeResolveContext ctx;
	ctx.robotInstanceIndex = instIdx;
	ctx.robotSceneBackendId = robotBackendId.toStdString();
	ctx.urdfPath = urdfPath.toStdString();
	ctx.coordinateFrames = &doc->robotCoordinateFramesForInstance(instIdx);

	RobotCanonicalExport::CanonicalProgramExportV1 exportDoc;
	std::string buildErr;
	// 导出走 canonical 树；不做 IK/playback 求解
	if (!RobotCanonicalExport::buildCanonicalExportV1(*exportProg, ctx,
													  RobotCanonicalExport::CanonicalExportLayout::NestedTree, false,
													  nullptr, exportDoc, &buildErr))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(QString::fromStdString(buildErr));
		}
		return;
	}

	std::string fileBody;
	std::string writeErr;
	if (!RobotCanonicalExport::writeCanonicalExportV1ToJson(exportDoc, fileBody, &writeErr, false))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(QString::fromStdString(writeErr));
		}
		return;
	}

	QTemporaryFile canonicalTemp(QDir::temp().filePath(QStringLiteral("cloudsim_canonical_XXXXXX.json")));
	canonicalTemp.setAutoRemove(true);
	if (!canonicalTemp.open())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("Cannot create temporary Canonical file."), QStringLiteral("无法创建临时 Canonical 文件。")));
		}
		return;
	}
	canonicalTemp.write(fileBody.c_str(), static_cast<qint64>(fileBody.size()));
	canonicalTemp.flush();
	const QString canonicalPath = canonicalTemp.fileName();

	nlohmann::json params = nlohmann::json::object();
	params["OutPutPath"] = outPath.toStdString();
	params["CanonicalPath"] = canonicalPath.toStdString();
	params["comment"] = std::string("CloudSim brand export");

	std::string pyErr;
	const std::string result =
		RobotWidget::PythonScriptCaller::instance().callPython(scriptPath.toStdString(), "ExportScript",
															   params.dump(), &pyErr);
	const bool ok = (result == "true" || result == "True" || result == "1");
	if (!ok)
	{
		if (m_host->runInfoPage())
		{
			const QString detail = pyErr.empty() ? QString::fromStdString(result) : QString::fromStdString(pyErr);
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("Brand export failed: %1").arg(detail), QStringLiteral("品牌导出失败：%1").arg(detail)));
		}
		refreshInstructionPoseAxes();
		return;
	}

	if (m_host->runInfoPage())
	{
		m_host->appendRunInfo(
			m_host->i18n(QStringLiteral("Exported %1 program \"%2\" to %3 (flat motion refs: %4).")
							 .arg(brand.brandId, brandDlg.selectedProgramName(), outPath)
							 .arg(exportDoc.flatMotionSequence.size()),
						 QStringLiteral("已导出 %1 程序「%2」到 %3（展平运动引用：%4）。")
							 .arg(brand.brandId, brandDlg.selectedProgramName(), outPath)
							 .arg(exportDoc.flatMotionSequence.size())));
	}
	refreshInstructionPoseAxes();
}

bool RobotSimulationController::tryCaptureCurrentRobotTcpPose(RobotInstruction::Vec3& outPoseMm,
															  RobotInstruction::Vec3& outEulerDeg,
															  osg::Matrixd* outTcpLocalMat,
															  osg::Matrixd* outTcpRenderWorldMat,
															  QString* outTcpLinkName, QString* errMsg) const
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !doc->hasRobotSimulationContext())
	{
		if (errMsg)
		{
			*errMsg = m_host->i18n(QStringLiteral("Robot simulation context is not ready."), QStringLiteral("机器人仿真上下文尚未就绪。"));
		}
		return false;
	}
	const int instIdx =
		m_host->simulationCommandPage() && m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
			? m_host->simulationCommandPage()->currentRobotInstanceIndex()
			: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		if (errMsg)
		{
			*errMsg = m_host->i18n(QStringLiteral("URDF path is empty."), QStringLiteral("URDF 路径为空。"));
		}
		return false;
	}
	const QString jointPrefix = doc->robotJointKeyPrefixForInstance(instIdx);
	const QStringList jointsLocal = doc->robotRevoluteJointNamesForInstance(instIdx);
	QVector<double> q(jointsLocal.size(), 0.0);
	QString qSource = QStringLiteral("zero-fallback");
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	osg::Matrixd robotBaseWorld;
	robotBaseWorld.makeIdentity();
	const bool hasRobotBaseWorld =
		RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, osg, instIdx, robotBaseWorld);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	if (m_aggregatedJointAnglesRad.size() >= jointOffset + q.size())
	{
		for (int j = 0; j < q.size(); ++j)
		{
			q[j] = m_aggregatedJointAnglesRad[jointOffset + j];
		}
		qSource = QStringLiteral("aggregated");
	}
	else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() > 0)
	{
		const QVector<double> sliderQ = m_host->robotAxisControlPage()->jointAnglesRad();
		const int nCopy = std::min(q.size(), sliderQ.size());
		for (int j = 0; j < nCopy; ++j)
		{
			q[j] = sliderQ[j];
		}
		if (nCopy == q.size() && nCopy > 0)
		{
			qSource = QStringLiteral("slider");
		}
		else if (nCopy > 0)
		{
			qSource = QStringLiteral("slider-partial");
		}
	}
	const QString lastJointName = jointsLocal.isEmpty() ? QString() : (jointPrefix + jointsLocal.back());
	QHash<QString, osg::Matrixd> linkWorldByName;
	QString computeErr;
	const bool hasLinkFk = UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, q, linkWorldByName, &computeErr);

	QString fallbackFlangeLink;
	{
		QStringList revoluteChildLinks;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
		if (!revoluteChildLinks.isEmpty())
		{
			fallbackFlangeLink = revoluteChildLinks.back();
		}
	}
	if (fallbackFlangeLink.isEmpty() && m_host->simulationCommandPage())
	{
		fallbackFlangeLink = m_host->simulationCommandPage()->selectedTcpLink();
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	QString tcpLinkName;
	osg::Matrixd tcpLocal;
	bool hasTcpLocal = false;
	QString tcpSource = QStringLiteral("None");
	osg::Matrixd tcpRenderWorld;
	bool capturedFromScene = false;
	BackendMat4 capturedTargetInBase{};
	bool hasCapturedTargetInBase = false;
	// 优先 URDF 法兰 FK+工具；避免 per-link PAT/SceneFlange 双重偏置
	if (hasLinkFk)
	{
		if (RobotSimulationMath::targetInBaseFromUrdfFlangeFk(urdfPath, q, frames, fallbackFlangeLink,
															  capturedTargetInBase, nullptr, &tcpLinkName))
		{
			tcpLocal = RobotMatrixOsg::matrixFromBackendColMajor(capturedTargetInBase);
			hasTcpLocal = true;
			hasCapturedTargetInBase = true;
			tcpSource = QStringLiteral("UrdfFlangeFk+Tool");
		}
	}
	osg::Matrixd tcpLocalFromHierarchy;
	bool hasHierarchyLocal = false;
	if (!lastJointName.isEmpty())
	{
		cloudsim::core::Mat4 jointWorldMat;
		if (doc->robotJointWorldMatrix(lastJointName, jointWorldMat))
		{
			const osg::Matrixd jointWorld = RobotSimulationMath::osgMatrixFromCoreMat4(jointWorldMat);
			osg::Matrixd invBase;
			invBase.makeIdentity();
			if (hasRobotBaseWorld)
			{
				invBase = osg::Matrixd::inverse(robotBaseWorld);
			}
			tcpLocalFromHierarchy = invBase * jointWorld;
			hasHierarchyLocal = true;
		}
	}
	if (!hasTcpLocal && RobotSimulationMath::captureTcpFromSceneFlangeBackend(
							doc, osg, instIdx, frames, fallbackFlangeLink, robotBaseWorld, tcpLocal, tcpRenderWorld,
							tcpLinkName, tcpSource))
	{
		hasTcpLocal = true;
		capturedFromScene = true;
	}
	if (!hasTcpLocal && hasHierarchyLocal)
	{
		const BackendMat4 T_flange_tool = RobotSimulationMath::toolMat4ForFrames(frames, nullptr);
		tcpLocal = tcpLocalFromHierarchy * RobotSimulationMath::osgMatrixFromBackendMat4(T_flange_tool);
		if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
		{
			tcpLinkName = QString::fromStdString(RobotCoordinate::effectiveFlangeLinkName(frames, *tool));
		}
		else
		{
			tcpLinkName = fallbackFlangeLink;
		}
		hasTcpLocal = true;
		tcpSource = QStringLiteral("HierarchyJoint+Tool");
	}
	if (!hasTcpLocal)
	{
		if (errMsg)
		{
			if (!hasLinkFk)
			{
				const QString detail = computeErr.isEmpty()
										   ? m_host->i18n(QStringLiteral("URDF forward kinematics failed."), QStringLiteral("URDF 正解计算失败。"))
										   : computeErr;
				*errMsg = m_host->i18n(QStringLiteral("Cannot evaluate TCP: %1").arg(detail), QStringLiteral("无法求 TCP：%1").arg(detail));
			}
			else
			{
				std::string flangeLink;
				if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
				{
					flangeLink = RobotCoordinate::effectiveFlangeLinkName(frames, *tool);
				}
				else
				{
					flangeLink = fallbackFlangeLink.toStdString();
				}
				const QString flangeQ = QString::fromStdString(flangeLink);
				if (flangeQ.isEmpty())
				{
					*errMsg = m_host->i18n(QStringLiteral("Flange link name is not configured."), QStringLiteral("未配置法兰连杆名。"));
				}
				else if (!linkWorldByName.contains(flangeQ))
				{
					*errMsg = m_host->i18n(QStringLiteral("Link '%1' not in URDF FK result (check tool frame flange link).").arg(flangeQ), QStringLiteral("连杆「%1」不在 URDF 正解结果中（请检查工具系法兰连杆）。").arg(flangeQ));
				}
				else if (!lastJointName.isEmpty() && !doc->hasRobotJointLocalMatrix(lastJointName))
				{
					*errMsg = m_host->i18n(QStringLiteral("Per-link robot has no joint scene node '%1'; use URDF FK path.")
							.arg(lastJointName), QStringLiteral("每连杆机器人无关节场景节点「%1」；请使用 URDF 正解路径。").arg(lastJointName));
				}
				else
				{
					*errMsg = m_host->i18n(QStringLiteral("Cannot evaluate TCP world transform."), QStringLiteral("无法获取末端世界坐标。"));
				}
			}
		}
		return false;
	}

	// tcpLocal 已是基座系 T_base_target（非仅局部）
	const osg::Matrixd tcpWorld = tcpLocal;
	const osg::Matrixd renderWorld = capturedFromScene ? tcpRenderWorld : (tcpWorld * robotBaseWorld);
	// pose/euler 与 URDF FK 的 RigidTransform 对齐，供 IK/捕获
	if (hasCapturedTargetInBase && hasLinkFk)
	{
		engine::RigidTransform target{};
		QString flangeLinkQ;
		if (RobotSimulationMath::targetRigidTransformFromUrdfFlangeFk(urdfPath, q, frames, fallbackFlangeLink, target,
																	  &flangeLinkQ, nullptr))
		{
			target.translationMm(outPoseMm.x, outPoseMm.y, outPoseMm.z);
			target.eulerDegForDisplay(outEulerDeg.x, outEulerDeg.y, outEulerDeg.z);
			tcpLocal = engine::osgMatrixFromRigidTransform(target);
			capturedTargetInBase = RobotCoordinate::backendMat4FromRigidTransform(target);
		}
		else
		{
			const engine::RigidTransform target = RobotCoordinate::rigidTransformFromBackendMat4(capturedTargetInBase);
			target.translationMm(outPoseMm.x, outPoseMm.y, outPoseMm.z);
			target.eulerDegForDisplay(outEulerDeg.x, outEulerDeg.y, outEulerDeg.z);
			tcpLocal = engine::osgMatrixFromRigidTransform(target);
		}
	}
	else if (hasCapturedTargetInBase)
	{
		const engine::RigidTransform target = RobotCoordinate::rigidTransformFromBackendMat4(capturedTargetInBase);
		target.translationMm(outPoseMm.x, outPoseMm.y, outPoseMm.z);
		target.eulerDegForDisplay(outEulerDeg.x, outEulerDeg.y, outEulerDeg.z);
		tcpLocal = engine::osgMatrixFromRigidTransform(target);
	}
	else
	{
		const osg::Vec3d t = tcpWorld.getTrans();
		const osg::Vec3f euler = OsgScene::quatToEulerDeg(tcpWorld.getRotate());
		outPoseMm.x = t.x();
		outPoseMm.y = t.y();
		outPoseMm.z = t.z();
		outEulerDeg.x = euler.x();
		outEulerDeg.y = euler.y();
		outEulerDeg.z = euler.z();
	}
	if (outTcpLocalMat)
	{
		*outTcpLocalMat = tcpWorld;
	}
	if (outTcpRenderWorldMat)
	{
		*outTcpRenderWorldMat = renderWorld;
	}
	if (outTcpLinkName)
	{
		// Planning/IK reference should be a link name (URDF link frame key).
		*outTcpLinkName = tcpLinkName;
	}
	return true;
}

void RobotSimulationController::cancelArcTeach()
{
	m_arcTeachPending = false;
	m_arcTeachViaPose = {};
	m_arcTeachViaEuler = {};
	m_arcTeachViaTransform = {};
	m_arcTeachViaJointCsv.clear();
	m_arcTeachViaTcpLinkName.clear();
	if (m_host && m_host->simulationCommandPage())
	{
		m_host->simulationCommandPage()->setArcTeachPending(false);
	}
}

void RobotSimulationController::onSimulationAddInstructionRequested(RobotInstruction::Type type)
{
	if (!m_host->simulationCommandPage())
	{
		return;
	}
	if (type != RobotInstruction::Type::ARC && m_arcTeachPending)
	{
		cancelArcTeach();
	}
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 euler{};
	osg::Matrixd tcpLocalMat;
	osg::Matrixd tcpRenderWorldMat;
	QString tcpLinkName;
	QString err;
	const int capInstIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							   ? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							   : 0;
	IRobotDocumentHost* capDoc = m_host->document();
	IRobotOsgViewHost* capOsg = m_host->osgView();
	const QVector<double> qLocal = localJointAnglesForInstance(capInstIdx);
	bool usedGizmoTarget = false;
	if (m_lastTcpDragTargetValid)
	{
		m_lastTcpDragTargetInBase.translationMm(pose.x, pose.y, pose.z);
		m_lastTcpDragTargetInBase.eulerDegForDisplay(euler.x, euler.y, euler.z);
		tcpLocalMat = engine::osgMatrixFromRigidTransform(m_lastTcpDragTargetInBase);
		tcpRenderWorldMat = tcpLocalMat;
		if (capDoc && capOsg)
		{
			osg::Matrixd robotBaseWorld;
			robotBaseWorld.makeIdentity();
			if (RobotSimulationMath::robotBaseWorldMatrixForInstance(capDoc, capOsg, capInstIdx, robotBaseWorld,
																	 qLocal.isEmpty() ? nullptr : &qLocal))
			{
				// 位姿字段/local 为 P0；渲染基座为 P_eff，世界矩阵用 peff·P_eff
				const osg::Matrixd tcpInPeff = engine::osgMatrixFromRigidTransform(
					tcpDragRigidP0ToPeff(capDoc, capInstIdx, m_lastTcpDragTargetInBase));
				tcpRenderWorldMat = tcpInPeff * robotBaseWorld;
			}
		}
		usedGizmoTarget = true;
		if (capDoc && !capDoc->robotUrdfAbsolutePathForInstance(capInstIdx).isEmpty())
		{
			QStringList revoluteChildLinks;
			QString fallbackFlange;
			(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(
				capDoc->robotUrdfAbsolutePathForInstance(capInstIdx), revoluteChildLinks, nullptr);
			if (!revoluteChildLinks.isEmpty())
			{
				fallbackFlange = revoluteChildLinks.back();
			}
			if (m_host->simulationCommandPage())
			{
				fallbackFlange =
					RobotSimulationMath::defaultTcpLinkNameForUrdf(capDoc->robotUrdfAbsolutePathForInstance(capInstIdx),
																   m_host->simulationCommandPage()->selectedTcpLink());
			}
			tcpLinkName = fallbackFlange;
		}
	}
	else if (!tryCaptureCurrentRobotTcpPose(pose, euler, &tcpLocalMat, &tcpRenderWorldMat, &tcpLinkName, &err))
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(err);
		}
		return;
	}
	const engine::RigidTransform capturedRt =
		m_lastTcpDragTargetValid
			? m_lastTcpDragTargetInBase
			: engine::RigidTransform::fromTranslationEulerDeg(pose.x, pose.y, pose.z, euler.x, euler.y, euler.z);

	if (type == RobotInstruction::Type::ARC && !m_arcTeachPending)
	{
		m_arcTeachPending = true;
		m_arcTeachViaPose = pose;
		m_arcTeachViaEuler = euler;
		m_arcTeachViaTransform = capturedRt;
		m_arcTeachViaJointCsv =
			qLocal.isEmpty() ? std::string() : RobotInstructionPlanning::encodeJointAnglesRadCsv(qLocal);
		m_arcTeachViaTcpLinkName = tcpLinkName;
		m_host->simulationCommandPage()->setArcTeachPending(true);
		if (m_host->runInfoPage())
		{
			m_host->appendRunInfo(QStringLiteral("ARC teach: Via captured / 已捕获途经点，请移到终点后再点圆弧"));
		}
		m_lastTcpDragTargetValid = false;
		(void)usedGizmoTarget;
		return;
	}

	std::shared_ptr<RobotInstruction::Base> ins;
	if (type == RobotInstruction::Type::ARC)
	{
		ins = m_host->simulationCommandPage()->appendArcInstructionFromPoses(m_arcTeachViaPose, m_arcTeachViaEuler, pose,
																			 euler, true);
	}
	else
	{
		ins = m_host->simulationCommandPage()->appendInstructionFromCurrentPose(type, pose, euler, true);
	}
	m_host->invalidateInstructionPropertyCache();
	if (ins)
	{
		if (type == RobotInstruction::Type::ARC)
		{
			RobotInstruction::writeViaTransformToInstruction(*ins, m_arcTeachViaTransform);
			if (!m_arcTeachViaJointCsv.empty())
			{
				ins->setExtensionProperty("context.viaJointRadCsv", m_arcTeachViaJointCsv);
			}
			if (!m_arcTeachViaTcpLinkName.isEmpty())
			{
				ins->setExtensionProperty("context.viaTcpLinkName", m_arcTeachViaTcpLinkName.toStdString());
			}
			cancelArcTeach();
		}
		RobotInstruction::writeTargetTransformToInstruction(*ins, capturedRt);
		const std::string matCsv = RobotSimulationMath::encodeMatrix4Csv(tcpLocalMat);
		const std::string renderMatCsv = RobotSimulationMath::encodeMatrix4Csv(tcpRenderWorldMat);
		const osg::Matrixd renderWorldToFk = tcpLocalMat * osg::Matrixd::inverse(tcpRenderWorldMat);
		const std::string renderToFkCsv = RobotSimulationMath::encodeMatrix4Csv(renderWorldToFk);
		const osg::Vec3d deltaPosMm = tcpLocalMat.getTrans() - tcpRenderWorldMat.getTrans();
		std::ostringstream deltaOss;
		deltaOss.imbue(std::locale::classic());
		deltaOss << deltaPosMm.x() << "," << deltaPosMm.y() << "," << deltaPosMm.z();
		ins->setExtensionProperty("render.tcpWorldMat4", renderMatCsv);
		ins->setExtensionProperty("render.tcpLocalMat4", matCsv);
		ins->setExtensionProperty("render.tcpLinkName", tcpLinkName.toStdString());
		ins->setExtensionProperty("context.renderWorldToFkMat4", renderToFkCsv);
		ins->setExtensionProperty("context.renderToFkDeltaPosMmCsv", deltaOss.str());
		ins->setExtensionProperty("context.poseFrame", "base_tool_origin");
		ins->setExtensionProperty("context.capturedTcpLinkName", tcpLinkName.toStdString());
		if (IRobotDocumentHost* capDoc2 = m_host->document())
		{
			const int capInstIdx2 = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
										? m_host->simulationCommandPage()->currentRobotInstanceIndex()
										: 0;
			const RobotCoordinate::RobotCoordinateFrameSet& capFrames =
				capDoc2->robotCoordinateFramesForInstance(capInstIdx2);
			if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(capFrames))
			{
				const BackendMat4 toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
				ins->setExtensionProperty(RobotCoordinate::kExtContextToolFrameMat4,
										  RobotCoordinate::encodeMat4Csv(toolMat));
				const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(capFrames, *tool);
				if (!flangeLink.empty())
				{
					ins->setExtensionProperty("context.flangeLinkName", flangeLink);
				}
				ins->setExtensionProperty("context.activeToolFrameId", tool->id);
				ins->setExtensionProperty(RobotCoordinate::kExtMotionToolFrameId, tool->id);
			}
			if (const RobotCoordinate::RobotUserFrame* uf = RobotCoordinate::activeUserFrame(capFrames))
			{
				ins->setExtensionProperty(RobotCoordinate::kExtMotionUserFrameId, uf->id);
			}
			ins->setExtensionProperty(RobotCoordinate::kExtMotionTargetFrame, "base");
			const QString capUrdf = capDoc2->robotUrdfAbsolutePathForInstance(capInstIdx2);
			if (!capUrdf.isEmpty())
			{
				ins->setExtensionProperty("context.urdfPath", capUrdf.toStdString());
			}
			if (!tcpLinkName.isEmpty())
			{
				ins->setExtensionProperty("context.tcpLinkName", tcpLinkName.toStdString());
			}
			QStringList revoluteChildLinks;
			QString fallbackFlange;
			if (!capUrdf.isEmpty())
			{
				(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(capUrdf, revoluteChildLinks, nullptr);
				if (!revoluteChildLinks.isEmpty())
				{
					fallbackFlange = revoluteChildLinks.back();
				}
			}
			if (!qLocal.isEmpty())
			{
				// 示教只落 TCP；关节仅用于 FK 求目标，不写指令
				ins->eraseExtensionProperty("context.currentJointRadCsv");
				osg::Matrixd tcpWorldFromJoints;
				if (instructionTcpWorldMat4FromTaughtJoints(capDoc2, capInstIdx2, *ins, qLocal, tcpWorldFromJoints))
				{
					BackendMat4 targetInBase = BackendMat4::identity();
					(void)RobotSimulationMath::targetInBaseFromUrdfFlangeFk(capUrdf, qLocal, capFrames, fallbackFlange,
																			targetInBase, ins.get(), nullptr);
					const osg::Matrixd tcpLocalFromJoints = RobotMatrixOsg::matrixFromBackendColMajor(targetInBase);
					ins->setExtensionProperty("render.tcpWorldMat4",
											  RobotSimulationMath::encodeMatrix4Csv(tcpWorldFromJoints));
					ins->setExtensionProperty("render.tcpLocalMat4",
											  RobotSimulationMath::encodeMatrix4Csv(tcpLocalFromJoints));
				}
			}
			// 拖动示教落点写入外轴，规划可跳过密网格
			if (RobotExternal::hasEnabledExternalAxes(capDoc2->robotExternalAxesForInstance(capInstIdx2)))
			{
				const RobotExternal::RobotExternalAxisConfigSet& capSet =
					capDoc2->robotExternalAxesForInstance(capInstIdx2);
				const std::vector<double> qs = capDoc2->robotExternalAxisQ(capInstIdx2);
				const double qe = firstRobotBaseEnabledQ(capSet, qs, capDoc2->robotExternalAxisQMm(capInstIdx2));
				ins->setExtensionProperty(RobotExternal::kExtContextExternalAxisQMm,
										  QString::number(qe, 'g', 12).toStdString());
				ins->setExtensionProperty(RobotExternal::kExtContextExternalAxisQCsv,
										  RobotExternal::encodeExternalAxisQCsv(padExternalAxisQToConfig(capSet, qs)));
				if (const RobotExternal::RobotExternalAxisConfig* rail =
						RobotExternal::firstEnabledExternalAxis(capSet))
				{
					const QString dir = QStringLiteral("%1,%2,%3")
											.arg(rail->axis[0], 0, 'g', 12)
											.arg(rail->axis[1], 0, 'g', 12)
											.arg(rail->axis[2], 0, 'g', 12);
					ins->setExtensionProperty(RobotExternal::kExtContextExternalAxisDir, dir.toStdString());
				}
				// REP：相对工作架 TCP，供规划外层采样
				if (RobotExternal::hasEnabledWorkpieceExternalAxes(capSet) && m_lastTcpDragTargetValid)
				{
					const std::string boundId = RobotExternal::primaryWorkpieceBackendId(capSet);
					const cloudsim::core::Mat4 p0 = capDoc2->robotBasePlacementWorldForInstance(capInstIdx2);
					const cloudsim::core::Mat4 w0 =
						capDoc2->workpieceExternalBasePlacement(capInstIdx2, QString::fromStdString(boundId));
					const cloudsim::core::Mat4 offset =
						capDoc2->workpieceWorkingFrameOffset(capInstIdx2, QString::fromStdString(boundId));
					double tp0w[16];
					if (RobotExternal::composeWorkpieceWorkingFrameInRobotP0(
							p0.data(), w0.data(), capSet, boundId, padExternalAxisQToConfig(capSet, qs), offset.data(),
							tp0w))
					{
						BackendMat4 bm = BackendMat4::identity();
						for (int i = 0; i < 16; ++i)
						{
							bm.v[i] = tp0w[i];
						}
						const engine::RigidTransform T_p0_work = RobotCoordinate::rigidTransformFromBackendMat4(bm);
						const engine::RigidTransform T_work =
							T_p0_work.inverse().composeColumn(m_lastTcpDragTargetInBase);
						RobotInstruction::writeWorkingTcpToInstruction(*ins, T_work);
					}
				}
			}
			if (RunLogger::isDiagnosticsEnabled() && m_host->runInfoPage() && m_host->robotAxisControlPage() &&
				!capUrdf.isEmpty())
			{
				QString toolName = QStringLiteral("-");
				if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(capFrames))
				{
					toolName = QString::fromStdString(tool->name);
				}
				BackendMat4 T_target{};
				QString flangeLinkQ;
				if (RobotSimulationMath::targetInBaseFromUrdfFlangeFk(capUrdf, qLocal, capFrames, fallbackFlange,
																	  T_target, ins.get(), &flangeLinkQ))
				{
					const RobotCoordinate::RobotRigidFrame fTool = RobotCoordinate::mat4ToFrame(T_target);
					const RobotCoordinate::RobotRigidFrame fFlange = fTool;
					m_host->appendRunInfo(QStringLiteral("[Teach] tool=%1 path=UrdfFlange*Tool flange=(%2,%3,%4) "
														 "FK_tool=(%5,%6,%7) pose=(%8,%9,%10) link=%11")
											  .arg(toolName)
											  .arg(fFlange.positionMm[0], 0, 'f', 2)
											  .arg(fFlange.positionMm[1], 0, 'f', 2)
											  .arg(fFlange.positionMm[2], 0, 'f', 2)
											  .arg(fTool.positionMm[0], 0, 'f', 2)
											  .arg(fTool.positionMm[1], 0, 'f', 2)
											  .arg(fTool.positionMm[2], 0, 'f', 2)
											  .arg(ins->pose().x, 0, 'f', 2)
											  .arg(ins->pose().y, 0, 'f', 2)
											  .arg(ins->pose().z, 0, 'f', 2)
											  .arg(flangeLinkQ));
				}
			}
		}
		{
			const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
			const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
				m_host->simulationCommandPage()->instructions(robotBackendId);
			if (RobotInstruction::collectMotionInstructions(program).size() <= 1)
			{
				captureMotionPreviewProgramStartJoints();
			}
		}
		m_skipInstructionPreviewOnce = true;
		onSimulationInstructionSelectionChanged(ins);
		m_lastTcpDragTargetValid = false;
		if (ins->hasMotionAxisConfigurationProperty() && m_host->robotAxisControlPage() &&
			m_host->robotAxisControlPage()->jointCount() > 0)
		{
			QVector<double> seedQ = m_host->robotAxisControlPage()->jointAnglesRad();
			RobotInstruction::FeasibleMotionAxisConfigurationOptions feasible =
				feasibleMotionAxisConfigurationOptionsForInstruction(ins, &seedQ);
			m_host->applySuggestedAxisPresetFromSeedIfNeeded(ins, seedQ, feasible);
			ins->setExtensionProperty("context.axisConfigSeeded", "1");
		}
	}
	else
	{
		refreshInstructionPoseAxes();
	}
}

std::shared_ptr<RobotInstruction::Base>
RobotSimulationController::findInstructionById(const QString& instructionId) const
{
	if (instructionId.isEmpty() || !m_programEditService)
	{
		return nullptr;
	}
	const std::string idUtf8 = instructionId.toStdString();
	const RobotInstruction::InstructionProgramDocument doc = m_programEditService->currentDocument();
	std::unordered_map<std::string, std::shared_ptr<RobotInstruction::Base>> idMap;
	doc.collectIdMap(idMap);
	const auto it = idMap.find(idUtf8);
	if (it == idMap.end())
	{
		return nullptr;
	}
	return it->second;
}

void RobotSimulationController::invalidateFeasibleAxisConfigurationCache()
{
	++m_feasibleAxisJobToken;
	m_lastFeasibleAxisInstructionId.clear();
	m_lastFeasibleAxisFingerprint.clear();
	m_lastFeasibleAxisOptions = {};
	m_planResultCache.invalidateAll();
	invalidateChainSeedRollCache();
}

RobotInstruction::FeasibleMotionAxisConfigurationOptions
RobotSimulationController::feasibleMotionAxisConfigurationOptionsForInstruction(
	const std::shared_ptr<RobotInstruction::Base>& instruction, QVector<double>* outSeedJointRad,
	const PrecomputedChainSeed* precomputedChainSeed)
{
	RobotInstruction::FeasibleMotionAxisConfigurationOptions out;
	if (!instruction || !RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		return out;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return out;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return out;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return out;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return out;
	}
	ensureInstructionControllerKinematics(doc, instIdx, urdfPath);
	int targetMotionIndex = -1;
	QVector<double> rollingQ;
	if (precomputedChainSeed && !precomputedChainSeed->jointRad.isEmpty())
	{
		rollingQ = precomputedChainSeed->jointRad;
		targetMotionIndex = precomputedChainSeed->motionIndex;
	}
	else if (!buildChainSeedJointRadForInstruction(instruction, rollingQ, &targetMotionIndex))
	{
		return out;
	}
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath, m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	QString fingerprint = QString::fromStdString(instruction->id());
	if (instruction->hasPoseProperty())
	{
		const RobotInstruction::Vec3 p = instruction->pose();
		const RobotInstruction::Vec3 e = instruction->eulerDeg();
		fingerprint += QStringLiteral("|%1,%2,%3|%4,%5,%6")
						   .arg(p.x, 0, 'g', 8)
						   .arg(p.y, 0, 'g', 8)
						   .arg(p.z, 0, 'g', 8)
						   .arg(e.x, 0, 'g', 8)
						   .arg(e.y, 0, 'g', 8)
						   .arg(e.z, 0, 'g', 8);
	}
	fingerprint += QStringLiteral("|mi=%1").arg(targetMotionIndex);
	for (int j = 0; j < rollingQ.size(); ++j)
	{
		fingerprint += QLatin1Char(',') + QString::number(rollingQ[j], 'g', 8);
	}
	osg::Matrixd fpBaseWorld;
	fpBaseWorld.makeIdentity();
	if (RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, m_host->osgView(), instIdx, fpBaseWorld, &rollingQ))
	{
		fingerprint += QStringLiteral("|bw=%1,%2,%3")
						   .arg(fpBaseWorld(3, 0), 0, 'g', 8)
						   .arg(fpBaseWorld(3, 1), 0, 'g', 8)
						   .arg(fpBaseWorld(3, 2), 0, 'g', 8);
	}
	const QString instructionId = QString::fromStdString(instruction->id());
	if (const PlanResultCache::FeasibleAxisEntry* hit =
			m_planResultCache.fetchFeasibleAxis(instructionId, fingerprint))
	{
		m_lastFeasibleAxisInstructionId = instructionId;
		m_lastFeasibleAxisFingerprint = fingerprint;
		m_lastFeasibleAxisOptions = hit->options;
		if (outSeedJointRad)
		{
			*outSeedJointRad = hit->seedJointRad;
		}
		return hit->options;
	}

	const RobotInstructionPlanning::MotionPoseBackup targetBackup =
		RobotInstructionPlanning::backupInstructionPose(*instruction);
	RobotInstructionPlanning::prepareMotionInstructionForPlanning(*instruction, rollingQ, doc, m_host->osgView(),
																  instIdx, urdfPath, defaultTcpLinkName.toStdString(),
																  &doc->robotCoordinateFramesForInstance(instIdx));
	out = m_instructionController.queryFeasibleMotionAxisConfigurationOptions(*instruction);
	RobotInstructionPlanning::restoreInstructionPose(*instruction, targetBackup);

	m_planResultCache.storeFeasibleAxis(instructionId, fingerprint, out, rollingQ);
	m_lastFeasibleAxisInstructionId = instructionId;
	m_lastFeasibleAxisFingerprint = fingerprint;
	m_lastFeasibleAxisOptions = out;
	if (outSeedJointRad)
	{
		*outSeedJointRad = rollingQ;
	}
	return out;
}

void RobotSimulationController::onSimulationInstructionSelectionChanged(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (instruction && m_rawTrajectoryPreviewActive)
	{
		setRawTrajectoryPreviewActive(false);
	}
	if (instruction)
	{
		m_host->clearBackendObjectSelection(true);
	}
	if (m_trajectoryEditSession)
	{
		if (instruction && instruction->type() == RobotInstruction::Type::PathPlan)
		{
			m_trajectoryEditSession->bindPathPlan(instruction->id());
			if (m_simulationDock && m_simulationDock->trajectoryEditPage())
			{
				m_simulationDock->trajectoryEditPage()->syncBoundPathPlanFromSession();
			}
			if (const RobotInstruction::PathPlanInstruction* pp = RobotInstruction::asPathPlan(*instruction))
			{
				if (!pp->sourceFeatureJson().empty() && m_simulationDock)
				{
					showRobotDockTab(RobotSimulationDockWidget::kTabIndexTrajectoryGeneration);
				}
			}
		}
		else if (!instruction)
		{
			const std::string prev = m_trajectoryEditSession->boundPathPlanId();
			if (!prev.empty())
			{
				IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
				if (doc)
				{
					const RobotInstruction::RobotProgramCatalog& catalog = doc->robotProgramStore().activeCatalog();
					if (!catalog.findPathPlan(catalog.activeProgramId(), prev))
					{
						m_trajectoryEditSession->clearPathPlanBinding();
						if (m_simulationDock && m_simulationDock->trajectoryEditPage())
						{
							m_simulationDock->trajectoryEditPage()->refreshProgramAndGroupCombos();
						}
					}
				}
			}
		}
	}
	// 运行中只刷属性与选中路点游标/编号，不预览 / 不重建路点轴
	if (m_programExecutor.isRunning())
	{
		m_host->refreshInstructionPropertyPanel(instruction);
		refreshPlaybackPathOverlays(instruction);
		return;
	}
	std::optional<PrecomputedChainSeed> chainSeed;
	if (instruction && RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		PrecomputedChainSeed built;
		if (buildChainSeedJointRadForInstruction(instruction, built.jointRad, &built.motionIndex, &built.reliable))
		{
			chainSeed = built;
		}
	}
	const PrecomputedChainSeed* chainPtr = chainSeed.has_value() ? &*chainSeed : nullptr;
	const bool tcpDragActive = m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode();
	// 先摆姿再刷属性面板，降低「点了半天机器人才动」的体感延迟
	if (!tcpDragActive)
	{
		applyRobotPoseForInstructionPreview(instruction, chainPtr);
	}
	m_host->refreshInstructionPropertyPanel(instruction);
	if (instruction && instruction->hasMotionAxisConfigurationProperty())
	{
		const auto& ext = instruction->extensionProperties();
		if (instruction->motionAxisConfiguration().preset == "AUTO" &&
			ext.find("context.axisConfigSeeded") == ext.end())
		{
			scheduleDeferredFeasibleAxisProbe(instruction, FeasibleAxisProbePurpose::SelectionAutoSeed);
		}
	}
	scheduleInstructionPoseAxesRefresh(false);
	refreshPlaybackPathOverlays(instruction);
}

void RobotSimulationController::ensureInstructionControllerKinematics(IRobotDocumentHost* doc, const int instanceIndex,
																	  const QString& urdfPath)
{
	if (urdfPath.isEmpty())
	{
		m_instructionController.clearDhRows();
		m_cachedInstructionDhUrdfPath.clear();
		m_instructionController.clearExternalAxes();
		return;
	}
	// 有 URDF 时走数值 IK；自动 DH 对多数工业臂 origin 不可分解，跳过建表
	if (m_cachedInstructionDhUrdfPath != urdfPath)
	{
		m_instructionController.clearDhRows();
		m_cachedInstructionDhUrdfPath = urdfPath;
	}
	syncInstructionControllerExternalAxes(m_instructionController, doc, instanceIndex);
}

void RobotSimulationController::scheduleInstructionPoseAxesRefresh(const bool computeReachability)
{
	++m_poseAxesRefreshToken;
	const quint64 token = m_poseAxesRefreshToken;
	QTimer::singleShot(0, this,
					   [this, token, computeReachability]()
					   {
						   if (token != m_poseAxesRefreshToken || m_programExecutor.isRunning())
						   {
							   return;
						   }
						   refreshInstructionPoseAxes(computeReachability);
					   });
}

void RobotSimulationController::applyRobotPoseForInstructionPreview(
	const std::shared_ptr<RobotInstruction::Base>& instruction, const PrecomputedChainSeed* precomputedChainSeed,
	const bool forceCartesianIk)
{
	if (m_skipInstructionPreviewOnce)
	{
		m_skipInstructionPreviewOnce = false;
		return;
	}
	if (m_host->simulationCommandPage() && m_host->simulationCommandPage()->tcpDragTeachMode())
	{
		return;
	}
	if (m_programExecutor.isRunning() || !instruction || !RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !poseSink || !osg || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return;
	}
	{
		ensureInstructionControllerKinematics(doc, instIdx, urdfPath);
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath, m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	QVector<double> chainSeedQ;
	int targetMotionIndex = -1;
	bool chainReliable = true;
	if (precomputedChainSeed && !precomputedChainSeed->jointRad.isEmpty())
	{
		chainSeedQ = precomputedChainSeed->jointRad;
		targetMotionIndex = precomputedChainSeed->motionIndex;
		chainReliable = precomputedChainSeed->reliable;
	}
	else if (!buildChainSeedJointRadForInstruction(instruction, chainSeedQ, &targetMotionIndex, &chainReliable))
	{
		chainSeedQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
		chainReliable = false;
	}
	const QVector<double> programStartQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	const QVector<double> seedQ = chainReliable ? chainSeedQ : programStartQ;

	RobotInstruction::Base* targetIns = instruction.get();
	const RobotCoordinate::RobotCoordinateFrameSet& framesForPlan = doc->robotCoordinateFramesForInstance(instIdx);

	RobotInstruction::PlanResult plan{};
	std::string planErr;
	if (!planMotionConsistentWithPreview(*targetIns, seedQ, programStartQ, instIdx, urdfPath, defaultTcpLinkName,
										 robotBackendId, framesForPlan, plan, &planErr, true,
										 /*gateTaughtResidual=*/true, forceCartesianIk))
	{
		if (m_host->runInfoPage())
		{
			const QString pointTag = QString::fromStdString(
				RobotInstruction::formatMotionPointName(RobotInstruction::motionPointIndex(*targetIns)));
			const QString detail =
				planErr.empty() ? m_host->i18n(QStringLiteral("Preview IK failed."), QStringLiteral("预览 IK 失败。"))
								: QString::fromStdString(planErr);
			m_host->appendRunWarning(pointTag.isEmpty() ? detail : QStringLiteral("%1: %2").arg(pointTag, detail));
		}
		return;
	}
	QVector<double> resultQ;
	resultQ.reserve(static_cast<int>(plan.jointTargetsRad.size()));
	for (double v : plan.jointTargetsRad)
	{
		resultQ.push_back(v);
	}

	const QVector<double> rollingQClamped = clampJointAnglesToInstanceLimits(doc, instIdx, resultQ);
	(void)rollingQClamped;
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	if (m_aggregatedJointAnglesRad.size() != jnamesAll.size())
	{
		resyncAggregatedJointAnglesFromDocument(doc);
	}
	for (int j = 0; j < nj && jointOffset + j < m_aggregatedJointAnglesRad.size(); ++j)
	{
		m_aggregatedJointAnglesRad[jointOffset + j] = resultQ[j];
	}
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		m_suppressMotionPreviewStartCapture = true;
		const QSignalBlocker blocker(m_host->robotAxisControlPage());
		m_host->robotAxisControlPage()->setJointAnglesRad(resultQ);
		m_suppressMotionPreviewStartCapture = false;
	}
	applyExternalAxisFromPlan(instIdx, plan, targetIns);
	(void)doc->applyJointAnglesRad(instIdx, resultQ, m_aggregatedJointAnglesRad);

	refreshRobotCoordinateFrameOverlays(instruction, &resultQ);
	osg->requestRedraw();
}

namespace
{
/// 示教关节经 URDF FK+工具得到 TCP 世界矩阵
bool instructionTcpWorldMat4FromTaughtJoints(IRobotDocumentHost* doc, int instIdx, const RobotInstruction::Base& ins,
											 const QVector<double>& taughtQ, osg::Matrixd& outTcpWorld)
{
	if (!doc || instIdx < 0 || taughtQ.isEmpty())
	{
		return false;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return false;
	}
	QString fallbackFlangeLink;
	{
		QStringList revoluteChildLinks;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildLinks, nullptr);
		if (!revoluteChildLinks.isEmpty())
		{
			fallbackFlangeLink = revoluteChildLinks.back();
		}
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	BackendMat4 targetInBase = BackendMat4::identity();
	QString tcpLinkName;
	if (!RobotSimulationMath::targetInBaseFromUrdfFlangeFk(urdfPath, taughtQ, frames, fallbackFlangeLink, targetInBase,
														   &ins, &tcpLinkName))
	{
		return false;
	}
	const osg::Matrixd tcpLocal = RobotMatrixOsg::matrixFromBackendColMajor(targetInBase);
	osg::Matrixd robotBaseWorld;
	robotBaseWorld.makeIdentity();
	cloudsim::core::RobotPerLinkKinematicsSliceDto slice;
	if (doc->robotPerLinkKinematicsForInstance(instIdx, slice))
	{
		robotBaseWorld = RobotSimulationMath::osgMatrixFromCoreMat4(slice.robotBasePlacementWorld);
	}
	else if (!RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, nullptr, instIdx, robotBaseWorld))
	{
		return false;
	}
	outTcpWorld = tcpLocal * robotBaseWorld;
	return true;
}

osg::Matrixd tcpLocalFromPoseFields(const RobotInstruction::Base& ins)
{
	engine::RigidTransform T_pose{};
	if (RobotInstruction::readTargetTransformFromInstruction(ins, T_pose))
	{
		return engine::osgMatrixFromRigidTransform(T_pose);
	}
	return osg::Matrixd();
}

/// 指令 pose/euler → 局部 TCP；供 capture/IK
bool instructionTcpLocalMatrix(const RobotInstruction::Base& ins, osg::Matrixd& outTcpLocal)
{
	outTcpLocal = tcpLocalFromPoseFields(ins);
	return true;
}

void assignPoseAxisWorld(RobotOsgUi::InstructionPoseAxis& axis, const osg::Matrixd& T_world, bool lineMotion,
						 bool reachable)
{
	axis.lineMotion = lineMotion;
	axis.reachable = reachable;
	axis.hasLocalMatrix = false;
	axis.robotBackendId.clear();
	axis.mountTcpOnPatRoot = false;
	axis.urdfTcpAttachLinkName.clear();
	axis.positionMm = {T_world(3, 0), T_world(3, 1), T_world(3, 2)};
	const osg::Vec3f euler = OsgScene::quatToEulerDeg(T_world.getRotate());
	axis.eulerDeg = {static_cast<double>(euler.x()), static_cast<double>(euler.y()), static_cast<double>(euler.z())};
}

/// 基座系 TCP → 世界系路点轴（Via 等无示教关节 FK 的点）
bool fillInstructionPoseAxisFromBaseTcp(IRobotDocumentHost* doc, IRobotOsgViewHost* osg, int instIdx,
										const engine::RigidTransform& T_base_tcp, bool lineMotion, bool reachable,
										RobotOsgUi::InstructionPoseAxis& axis,
										const QVector<double>* jointForBase = nullptr)
{
	const osg::Matrixd tcpLocal = engine::osgMatrixFromRigidTransform(T_base_tcp);
	osg::Matrixd T_world = tcpLocal;
	if (doc && osg && instIdx >= 0)
	{
		osg::Matrixd baseWorld;
		baseWorld.makeIdentity();
		if (RobotSimulationMath::robotBaseWorldMatrixForInstance(
				doc, osg, instIdx, baseWorld, jointForBase && !jointForBase->isEmpty() ? jointForBase : nullptr))
		{
			T_world = tcpLocal * baseWorld;
		}
	}
	assignPoseAxisWorld(axis, T_world, lineMotion, reachable);
	return true;
}

bool fillInstructionPoseAxisMount(IRobotDocumentHost* doc, IRobotOsgViewHost* osg, int instIdx,
								  const RobotInstruction::Base& ins, bool lineMotion, bool reachable,
								  RobotOsgUi::InstructionPoseAxis& axis,
								  const QVector<double>* jointAnglesRadLocal = nullptr)
{
	engine::RigidTransform T_target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(ins, T_target))
	{
		return false;
	}
	const osg::Matrixd tcpLocal = engine::osgMatrixFromRigidTransform(T_target);

	osg::Matrixd T_world = tcpLocal;
	bool positioned = false;
	// 有示教关节则 FK 到 TCP；否则用缓存世界矩阵 / 笛卡尔·基座
	if (jointAnglesRadLocal && !jointAnglesRadLocal->isEmpty() && doc && instIdx >= 0)
	{
		osg::Matrixd fkWorld;
		if (instructionTcpWorldMat4FromTaughtJoints(doc, instIdx, ins, *jointAnglesRadLocal, fkWorld))
		{
			T_world = fkWorld;
			positioned = true;
		}
	}
	if (!positioned)
	{
		const auto itWorld = ins.extensionProperties().find("render.tcpWorldMat4");
		if (itWorld != ins.extensionProperties().end() && !itWorld->second.empty())
		{
			osg::Matrixd T_cached;
			if (RobotSimulationMath::decodeMatrix4Csv(itWorld->second, T_cached))
			{
				T_world = T_cached;
				positioned = true;
			}
		}
	}
	if (!positioned && doc && osg && instIdx >= 0)
	{
		osg::Matrixd baseWorld;
		baseWorld.makeIdentity();
		const QVector<double>* jointForBase = jointAnglesRadLocal;
		if (RobotSimulationMath::robotBaseWorldMatrixForInstance(
				doc, osg, instIdx, baseWorld, jointForBase && !jointForBase->isEmpty() ? jointForBase : nullptr))
		{
			T_world = tcpLocal * baseWorld;
		}
	}

	assignPoseAxisWorld(axis, T_world, lineMotion, reachable);
	return true;
}
} // namespace

void RobotSimulationController::refreshPlaybackPathOverlays(
	const std::shared_ptr<RobotInstruction::Base>& highlightInstruction)
{
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	SimulationCommandWidget* cmd = m_host ? m_host->simulationCommandPage() : nullptr;
	if (!osg || !cmd)
	{
		return;
	}

	// 仅指令树选中驱动游标+编号（含运行中树跟播）
	const std::string focusId =
		(highlightInstruction && highlightInstruction->hasPoseProperty()) ? highlightInstruction->id() : std::string();
	if (focusId.empty() || focusId != m_waypointPickHighlightInstructionId)
	{
		m_waypointPickHighlightPreferVia = false;
		m_waypointPickHighlightInstructionId.clear();
	}
	const bool preferVia = m_waypointPickHighlightPreferVia && highlightInstruction &&
						   highlightInstruction->type() == RobotInstruction::Type::ARC;
	const std::string overlayFp = preferVia ? (focusId + "|via") : focusId;
	if (overlayFp == m_waypointLabelOverlayFingerprint)
	{
		return;
	}
	m_waypointLabelOverlayFingerprint = overlayFp;

	if (focusId.empty())
	{
		osg->clearPlaybackCursorOverlay();
		osg->clearWaypointIndexLabels();
		return;
	}

	const int axisInstIdx = cmd->currentRobotInstanceIndex() >= 0 ? cmd->currentRobotInstanceIndex() : 0;
	const int njForTaught = (doc && axisInstIdx >= 0) ? doc->robotRevoluteJointCountForInstance(axisInstIdx) : 0;
	QVector<double> taughtScratch =
		RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*highlightInstruction);
	const QVector<double>* jointPtr = nullptr;
	if (njForTaught > 0 && taughtScratch.size() == njForTaught)
	{
		jointPtr = &taughtScratch;
	}
	const bool lineLike = highlightInstruction->type() == RobotInstruction::Type::LINE ||
						  highlightInstruction->type() == RobotInstruction::Type::ARC;
	RobotOsgUi::InstructionPoseAxis axis;
	bool axisOk = false;
	if (preferVia)
	{
		engine::RigidTransform T_via{};
		if (RobotInstruction::readViaTransformFromInstruction(*highlightInstruction, T_via))
		{
			axisOk = fillInstructionPoseAxisFromBaseTcp(doc, osg, axisInstIdx, T_via, true, true, axis, nullptr);
		}
	}
	if (!axisOk)
	{
		axisOk = fillInstructionPoseAxisMount(doc, osg, axisInstIdx, *highlightInstruction, lineLike, true, axis,
											  jointPtr);
	}
	if (!axisOk)
	{
		osg->clearPlaybackCursorOverlay();
		osg->clearWaypointIndexLabels();
		return;
	}

	RobotOsgUi::PlaybackCursorOverlay cursor;
	cursor.positionMm = axis.positionMm;
	cursor.eulerDeg = axis.eulerDeg;
	osg->setPlaybackCursorOverlay(cursor);

	int displayIndex = 0;
	for (const std::shared_ptr<RobotInstruction::Base>& ins : cmd->instructionList())
	{
		if (!ins || !ins->hasPoseProperty())
		{
			continue;
		}
		// 隐藏分组不计入序号，但选中项仍显示其序号
		if (!isInstructionVisibleIn3d(ins->id()) && ins->id() != focusId)
		{
			continue;
		}
		++displayIndex;
		if (ins->id() == focusId)
		{
			RobotOsgUi::WaypointIndexLabel lab;
			lab.displayIndex = displayIndex;
			lab.positionMm = axis.positionMm;
			osg->setWaypointIndexLabels({lab});
			return;
		}
	}
	osg->clearWaypointIndexLabels();
}

void RobotSimulationController::syncInstructionRenderMatricesFromWorldPose(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (!instruction || !instruction->hasPoseProperty())
	{
		return;
	}
	engine::RigidTransform target{};
	if (!RobotInstruction::readTargetTransformFromInstruction(*instruction, target))
	{
		return;
	}
	const osg::Matrixd world = engine::osgMatrixFromRigidTransform(target);
	instruction->setExtensionProperty("render.tcpWorldMat4", RobotSimulationMath::encodeMatrix4Csv(world));
	instruction->setExtensionProperty("render.tcpLocalMat4", RobotSimulationMath::encodeMatrix4Csv(world));
}

void RobotSimulationController::syncInstructionRenderMatricesFromPose(
	const std::shared_ptr<RobotInstruction::Base>& instruction)
{
	if (!instruction || !instruction->hasPoseProperty())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	osg::Matrixd tcpLocal = tcpLocalFromPoseFields(*instruction);
	instruction->setExtensionProperty("render.tcpLocalMat4", RobotSimulationMath::encodeMatrix4Csv(tcpLocal));
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	if (const RobotCoordinate::RobotToolFrame* tool =
			RobotCoordinate::resolveToolFrameForExtension(frames, instruction->extensionProperties()))
	{
		instruction->setExtensionProperty("context.activeToolFrameId", tool->id);
	}
	osg::Matrixd tcpWorld;
	tcpWorld.makeIdentity();
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	const auto& ext = instruction->extensionProperties();
	const auto itTargetQ = ext.find(RobotInstruction::kExtContextTargetTransformQuatCsv);
	const auto itTargetT = ext.find(RobotInstruction::kExtContextTargetTransformTransMmCsv);
	const bool hasCartesianTarget =
		itTargetQ != ext.end() && itTargetT != ext.end() && !itTargetQ->second.empty() && !itTargetT->second.empty();
	QVector<double> taughtQ = RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*instruction);
	// 无笛卡尔目标时用示教角 FK 刷 world 路点
	const bool usedTaughtFk = !hasCartesianTarget && taughtQ.size() == nj &&
							  instructionTcpWorldMat4FromTaughtJoints(doc, instIdx, *instruction, taughtQ, tcpWorld);
	if (usedTaughtFk)
	{
		instruction->setExtensionProperty("render.tcpWorldMat4", RobotSimulationMath::encodeMatrix4Csv(tcpWorld));
	}
	else
	{
		// 笛卡尔目标但关节维不齐：已有 world 缓存则直接返回
		const auto itWorld = ext.find("render.tcpWorldMat4");
		if (hasCartesianTarget && taughtQ.size() != nj && itWorld != ext.end() && !itWorld->second.empty())
		{
			osg::Matrixd cachedWorld;
			if (RobotSimulationMath::decodeMatrix4Csv(itWorld->second, cachedWorld))
			{
				return;
			}
		}
		if (hasCartesianTarget && taughtQ.size() != nj)
		{
			// 维不齐时退回把 tcpLocal 写入 world 缓存，避免坏 FK
			instruction->setExtensionProperty("render.tcpWorldMat4", RobotSimulationMath::encodeMatrix4Csv(tcpLocal));
			return;
		}
		osg::Matrixd robotBaseWorld;
		robotBaseWorld.makeIdentity();
		const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
		QVector<double> syncJointQ;
		if (hasCartesianTarget && taughtQ.size() == nj)
		{
			syncJointQ = taughtQ;
		}
		else if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj)
		{
			syncJointQ.resize(nj);
			for (int j = 0; j < nj; ++j)
			{
				syncJointQ[j] = m_aggregatedJointAnglesRad[jointOffset + j];
			}
		}
		if (RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, osg, instIdx, robotBaseWorld,
																 syncJointQ.isEmpty() ? nullptr : &syncJointQ))
		{
			instruction->setExtensionProperty("render.tcpWorldMat4",
											  RobotSimulationMath::encodeMatrix4Csv(tcpLocal * robotBaseWorld));
		}
	}
}

QHash<QString, bool> RobotSimulationController::computeMotionReachabilityForCurrentProgram()
{
	QHash<QString, bool> reachability;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return reachability;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return reachability;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return reachability;
	}
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath, m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	const std::vector<robot_kinematics::DhRow> dhRows; // 有 URDF 不建 DH，worker 走数值 IK
	QVector<double> rollingQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	const QVector<double> programStartQ = rollingQ;
	for (size_t mi = 0; mi < motions.size(); ++mi)
	{
		const RobotInstruction::Base* motionPtr = motions[mi];
		if (!motionPtr)
		{
			continue;
		}
		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motionPtr);
		const QString insIdQ = QString::fromStdString(ins->id());
		const RobotInstructionPlanning::MotionPoseBackup backup = RobotInstructionPlanning::backupInstructionPose(*ins);
		std::string planErr;
		RobotInstruction::PlanResult plan{};
		ins->eraseExtensionProperty("context.playbackPlanLite");
		const bool ok = planMotionConsistentWithPreview(*ins, rollingQ, programStartQ, instIdx, urdfPath,
														defaultTcpLinkName, robotBackendId, frames, plan, &planErr,
														false);
		reachability.insert(insIdQ, ok);
		RobotInstructionPlanning::restoreInstructionPose(*ins, backup);
		if (ok && !plan.jointTargetsRad.empty() && plan.jointTargetsRad.size() == static_cast<size_t>(rollingQ.size()))
		{
			for (int j = 0; j < rollingQ.size(); ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
	}
	return reachability;
}

void RobotSimulationController::refreshInstructionPoseAxesWithReachability(const QHash<QString, bool>& reachability)
{
	if (m_rawTrajectoryPreviewActive)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!osg || !m_host->simulationCommandPage())
	{
		if (osg)
		{
			osg->clearInstructionPoseAxes();
		}
		return;
	}
	const std::vector<std::shared_ptr<RobotInstruction::Base>> insList =
		m_host->simulationCommandPage()->instructionList();
	std::vector<RobotOsgUi::InstructionPoseAxis> axes;
	axes.reserve(insList.size());
	const int axisInstIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
								? m_host->simulationCommandPage()->currentRobotInstanceIndex()
								: 0;
	const int njForTaught =
		(doc && axisInstIdx >= 0) ? doc->robotRevoluteJointCountForInstance(axisInstIdx) : 0;
	QVector<double> taughtScratch;
	for (const auto& ins : insList)
	{
		if (!ins || !ins->hasPoseProperty())
		{
			continue;
		}
		if (!isInstructionVisibleIn3d(ins->id()))
		{
			continue;
		}
		const auto itReach = reachability.constFind(QString::fromStdString(ins->id()));
		const bool reachable = (itReach == reachability.constEnd()) ? true : itReach.value();
		const bool lineLike =
			ins->type() == RobotInstruction::Type::LINE || ins->type() == RobotInstruction::Type::ARC;
		// 路点轴跟示教/笛卡尔目标，勿用轴控当前角（重置零位会把选中点画到法兰零位）
		const QVector<double>* jointPtr = nullptr;
		taughtScratch = RobotInstructionPlanning::jointAnglesRadFromInstructionContext(*ins);
		if (njForTaught > 0 && taughtScratch.size() == njForTaught)
		{
			jointPtr = &taughtScratch;
		}
		if (ins->type() == RobotInstruction::Type::ARC)
		{
			engine::RigidTransform T_via{};
			if (RobotInstruction::readViaTransformFromInstruction(*ins, T_via))
			{
				RobotOsgUi::InstructionPoseAxis viaAxis;
				if (fillInstructionPoseAxisFromBaseTcp(doc, osg, axisInstIdx, T_via, true, reachable, viaAxis,
													   nullptr))
				{
					viaAxis.instructionId = ins->id();
					viaAxis.isArcVia = true;
					axes.push_back(viaAxis);
				}
			}
		}
		RobotOsgUi::InstructionPoseAxis a;
		if (!fillInstructionPoseAxisMount(doc, osg, axisInstIdx, *ins, lineLike, reachable, a, jointPtr))
		{
			continue;
		}
		a.instructionId = ins->id();
		axes.push_back(a);
	}
	if (axes.empty())
	{
		osg->clearInstructionPoseAxes();
		std::shared_ptr<RobotInstruction::Base> sel;
		if (InstructionProgramTreeWidget* tree = m_host->simulationCommandPage()->instructionTree())
		{
			sel = tree->selectedInstruction();
		}
		m_waypointLabelOverlayFingerprint.clear();
		refreshPlaybackPathOverlays(sel);
		return;
	}
	osg->setInstructionPoseAxes(axes);
	{
		std::shared_ptr<RobotInstruction::Base> sel;
		if (InstructionProgramTreeWidget* tree = m_host->simulationCommandPage()->instructionTree())
		{
			sel = tree->selectedInstruction();
		}
		m_waypointLabelOverlayFingerprint.clear();
		refreshPlaybackPathOverlays(sel);
	}
}

bool RobotSimulationController::isInstructionGroupVisible(const std::string& groupId) const
{
	return !m_hiddenInstructionGroupIds.contains(QString::fromStdString(groupId));
}

void RobotSimulationController::setInstructionGroupVisible(const std::string& groupId, const bool visible)
{
	const QString qid = QString::fromStdString(groupId);
	if (visible)
	{
		m_hiddenInstructionGroupIds.remove(qid);
	}
	else
	{
		m_hiddenInstructionGroupIds.insert(qid);
	}
	refreshPathPlanRawOverlays();
	refreshInstructionPoseAxes(false);
	if (m_simulationDock && m_simulationDock->commandPage())
	{
		m_simulationDock->commandPage()->refreshInstructionList();
	}
}

void RobotSimulationController::onInstructionGroupVisibilityChangeRequested(const std::string& groupId,
																			const bool visible)
{
	if (groupId.empty())
	{
		return;
	}
	setInstructionGroupVisible(groupId, visible);
}

bool RobotSimulationController::isInstructionVisibleIn3d(const std::string& instructionId) const
{
	if (!m_host || !m_host->document())
	{
		return true;
	}
	const RobotInstruction::RobotProgramCatalog& catalog = m_host->document()->robotProgramStore().activeCatalog();
	const RobotInstruction::RobotProgram* prog = catalog.findProgram(catalog.activeProgramId());
	if (!prog)
	{
		return true;
	}
	for (const RobotInstruction::InstructionGroup& group : prog->groups)
	{
		if (isInstructionGroupVisible(group.id))
		{
			continue;
		}
		for (const std::string& memberId : group.memberInstructionIds)
		{
			if (memberId == instructionId)
			{
				return false;
			}
		}
	}
	return true;
}

bool RobotSimulationController::isPathPlanRawVisible(const std::string& pathPlanId) const
{
	if (!m_host || !m_host->document() || pathPlanId.empty())
	{
		return true;
	}
	const RobotInstruction::RobotProgramCatalog& catalog = m_host->document()->robotProgramStore().activeCatalog();
	const RobotInstruction::RobotProgram* prog = catalog.findProgram(catalog.activeProgramId());
	if (!prog)
	{
		return true;
	}
	for (const RobotInstruction::InstructionGroup& group : prog->groups)
	{
		if (group.role != RobotInstruction::InstructionGroupRole::PathPlanOutput)
		{
			continue;
		}
		if (group.pathPlanInstructionId == pathPlanId)
		{
			return isInstructionGroupVisible(group.id);
		}
	}
	return true;
}

bool RobotSimulationController::isTrajectoryGenerationTabActive() const
{
	if (!m_simulationDock || !m_simulationDock->robotTabWidget())
	{
		return false;
	}
	if (m_simulationDock->dockMode() != SimulationDockMode::Robot)
	{
		return false;
	}
	return m_simulationDock->robotTabWidget()->currentIndex() ==
		   RobotSimulationDockWidget::kTabIndexTrajectoryGeneration;
}

bool RobotSimulationController::shouldShowTrajectoryGenerationPreview() const
{
	if (!m_simulationDock)
	{
		return false;
	}
	const FeatureTrajectoryPageWidget* feat = m_simulationDock->featureTrajectoryPage();
	return feat && feat->isFeatureEditActive();
}

void RobotSimulationController::onSimulationDockTabChanged(int index)
{
	if (m_simulationDock && m_simulationDock->dockMode() == SimulationDockMode::CustomDevice &&
		index == RobotSimulationDockWidget::kTabIndexDeviceAxisControl)
	{
		syncAxisControlTargetForDockMode(SimulationDockMode::CustomDevice);
	}
	if (isTrajectoryGenerationTabActive())
	{
		if (TrajectoryGenerationPageWidget* gen =
				m_simulationDock ? m_simulationDock->trajectoryGenerationPage() : nullptr)
			gen->refreshWorkpieces();
	}
	refreshCollisionPageMotionWaypoints();
	refreshCollisionPageSceneObjects();
	refreshPathPlanPreviewForActiveTab();
}

void RobotSimulationController::refreshPathPlanPreviewForActiveTab(const RobotInstruction::RawTrajectory* preferRaw)
{
	if (isTrajectoryGenerationTabActive())
	{
		if (shouldShowTrajectoryGenerationPreview())
		{
			refreshBoundPathPlanPreview(preferRaw);
		}
		else
		{
			clearBoundPathPlanPreview();
			refreshInstructionPoseAxes(false);
		}
	}
	else
	{
		refreshPathPlanRawOverlays();
		// Applied 后若未开 raw 预览则刷指令路点轴
		if (!m_rawTrajectoryPreviewActive)
		{
			refreshInstructionPoseAxes(false);
		}
	}
}

void RobotSimulationController::clearBoundPathPlanPreview()
{
	if (!m_host)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	// raw 预览会清掉指令路点轴；退出时必须恢复，否则轴会一直“消失”
	const bool wasRaw = m_rawTrajectoryPreviewActive;
	m_rawTrajectoryPreviewActive = false;
	osg->clearRawTrajectoryOverlay();
	osg->clearRawTrajectoryOverlayFrames();
	osg->requestRedraw();
	if (wasRaw)
	{
		refreshInstructionPoseAxes(false);
	}
}

void RobotSimulationController::refreshBoundPathPlanPreview(const RobotInstruction::RawTrajectory* preferRaw)
{
	if (!m_host)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}

	const RobotInstruction::RawTrajectory* src = preferRaw;
	RobotInstruction::RawTrajectory stored;
	if ((!src || src->points.empty()) && m_trajectoryEditSession)
	{
		if (const RobotInstruction::RawTrajectory* sessionRaw = m_trajectoryEditSession->rawTrajectory())
		{
			if (!sessionRaw->points.empty())
			{
				src = sessionRaw;
			}
		}
	}
	if ((!src || src->points.empty()) && m_trajectoryEditSession && m_host->document())
	{
		const std::string boundId = m_trajectoryEditSession->boundPathPlanId();
		if (!boundId.empty() &&
			m_host->document()->robotProgramStore().activeCatalog().pathPlanRaws().load(boundId, stored) &&
			!stored.points.empty())
		{
			src = &stored;
		}
	}
	if (!src || src->points.empty())
	{
		clearBoundPathPlanPreview();
		return;
	}

	const std::string backendId = RobotInstruction::rawTrajectoryWorkpieceBackendId(*src);
	if (backendId.empty())
	{
		clearBoundPathPlanPreview();
		return;
	}

	RobotOsgUi::RawTrajectoryPreviewOptions options;
	if (FeatureTrajectoryPageWidget* feat = m_simulationDock ? m_simulationDock->featureTrajectoryPage() : nullptr)
	{
		options = feat->previewOptions();
	}

	std::string err;
	feature_pick_transform::applyRawTrajectoryPreviewToOsg(osg, backendId, *src, options, &err);
	if (!err.empty())
	{
		m_host->appendRunWarning(QString::fromStdString(err));
		// apply 已清路点轴，失败时按“曾开启 raw”恢复
		m_rawTrajectoryPreviewActive = true;
		clearBoundPathPlanPreview();
		return;
	}
	m_rawTrajectoryPreviewActive = true;
}

void RobotSimulationController::refreshPathPlanRawOverlays()
{
	if (isTrajectoryGenerationTabActive())
	{
		if (shouldShowTrajectoryGenerationPreview())
		{
			refreshBoundPathPlanPreview();
		}
		else
		{
			clearBoundPathPlanPreview();
		}
		return;
	}
	if (!m_host)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	IRobotDocumentHost* doc = m_host->document();
	if (!osg || !doc)
	{
		return;
	}
	RobotInstruction::RobotProgramCatalog& catalog = doc->robotProgramStore().activeCatalog();
	const std::string programId = catalog.activeProgramId();
	const std::vector<RobotInstruction::PathPlanInstruction*> pathPlans = catalog.listPathPlans(programId);

	RobotOsgUi::RawTrajectoryPreviewOptions options;
	if (FeatureTrajectoryPageWidget* feat = m_simulationDock ? m_simulationDock->featureTrajectoryPage() : nullptr)
	{
		options = feat->previewOptions();
	}

	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex> mergedOverlay;
	std::vector<std::size_t> mergedSegmentEnds;
	std::vector<RobotInstruction::RawTrajectory> axisRawStore;
	std::vector<std::pair<std::string, const RobotInstruction::RawTrajectory*>> axesSources;
	axisRawStore.reserve(pathPlans.size());
	axesSources.reserve(pathPlans.size());

	for (const RobotInstruction::PathPlanInstruction* pp : pathPlans)
	{
		if (!pp)
		{
			continue;
		}
		// Applied 阶段不再叠 pathPlanRaws（CAD 预览已提交）
		if (pp->phase() == RobotInstruction::PathPlanPhase::Applied)
		{
			continue;
		}
		if (!isPathPlanRawVisible(pp->id()))
		{
			continue;
		}
		RobotInstruction::RawTrajectory raw;
		if (!catalog.pathPlanRaws().load(pp->id(), raw) || raw.points.empty())
		{
			continue;
		}
		const std::string backendId = RobotInstruction::rawTrajectoryWorkpieceBackendId(raw);
		if (backendId.empty())
		{
			continue;
		}
		std::string err;
		if (!feature_pick_transform::appendRawTrajectoryOverlayWorld(osg, backendId, raw, mergedOverlay,
																	 mergedSegmentEnds, &err))
		{
			if (!err.empty())
			{
				m_host->appendRunWarning(QString::fromStdString(err));
			}
			continue;
		}
		axisRawStore.push_back(std::move(raw));
		axesSources.emplace_back(backendId, &axisRawStore.back());
	}

	if (mergedOverlay.empty())
	{
		const bool wasRaw = m_rawTrajectoryPreviewActive;
		m_rawTrajectoryPreviewActive = false;
		osg->clearRawTrajectoryOverlay();
		osg->clearRawTrajectoryOverlayFrames();
		osg->requestRedraw();
		if (wasRaw)
		{
			refreshInstructionPoseAxes(false);
		}
		return;
	}

	feature_pick_transform::finalizeOverlaySegmentEnds(mergedOverlay.size(), mergedSegmentEnds);

	std::string err;
	feature_pick_transform::applyMergedRawTrajectoryPreviewToOsg(osg, mergedOverlay, mergedSegmentEnds, axesSources,
																 options, &err);
	if (!err.empty())
	{
		m_host->appendRunWarning(QString::fromStdString(err));
		// applyMerged 已清路点轴，失败后恢复
		m_rawTrajectoryPreviewActive = false;
		osg->clearRawTrajectoryOverlay();
		osg->clearRawTrajectoryOverlayFrames();
		refreshInstructionPoseAxes(false);
		return;
	}
	m_rawTrajectoryPreviewActive = true;
}

void RobotSimulationController::setRawTrajectoryPreviewActive(const bool active)
{
	const bool wasActive = m_rawTrajectoryPreviewActive;
	m_rawTrajectoryPreviewActive = active;
	if (!active && m_host && m_host->osgView())
	{
		IRobotOsgViewHost* osg = m_host->osgView();
		osg->clearRawTrajectoryOverlay();
		osg->clearRawTrajectoryOverlayFrames();
		if (wasActive)
		{
			refreshInstructionPoseAxes(false);
		}
	}
}

void RobotSimulationController::refreshInstructionPoseAxes(const bool computeReachability)
{
	if (m_rawTrajectoryPreviewActive)
	{
		return;
	}
	static bool s_matrixConventionSelfTestDone = false;
	if (!s_matrixConventionSelfTestDone)
	{
		s_matrixConventionSelfTestDone = true;
		std::vector<std::string> matrixTestFailures;
		const bool matrixOk = RobotMatrixOsg::runConventionSelfTest(matrixTestFailures);
		if (m_host->runInfoPage())
		{
			if (matrixOk)
			{
				if (RunLogger::isDiagnosticsEnabled())
				{
					m_host->appendRunInfo(QStringLiteral("[Matrix self-test] BackendMat4/OSG convention OK"));
				}
			}
			else
			{
				m_host->appendRunWarning(QStringLiteral("[Matrix self-test] failed %1 checks; pose axes may be wrong.")
											 .arg(static_cast<int>(matrixTestFailures.size())));
				for (const std::string& msg : matrixTestFailures)
				{
					m_host->appendRunWarning(QString::fromStdString(msg));
				}
			}
		}
	}

	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!osg || !m_host->simulationCommandPage())
	{
		if (osg)
		{
			osg->clearInstructionPoseAxes();
		}
		return;
	}

	if (computeReachability)
	{
		// 先空可达性刷新 UI，再丢 Job 异步算
		m_motionReachabilityCache.clear();
		refreshInstructionPoseAxesWithReachability(QHash<QString, bool>{});
		scheduleAsyncMotionReachabilityRefresh();
		return;
	}
	refreshInstructionPoseAxesWithReachability(QHash<QString, bool>{});
}

void RobotSimulationController::onSimulationStartTriggered()
{
	if (m_programExecutor.isRunning())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage())
	{
		return;
	}
	if (!doc->hasRobotSimulationContext())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(
				m_host->i18n(QStringLiteral("Import a robot (URDF) first, then add simulation commands."), QStringLiteral("请先导入机器人(URDF)，再添加仿真指令。")));
		}
		return;
	}
	// 碰撞页绕障结果仅预览；未确认插入时 Run 仍按原指令 Direct，易全员穿模失败
	if (m_motionPathPreviewActive && m_lastMotionPathRawValid && m_host->runInfoPage())
	{
		m_host->appendRunWarning(QStringLiteral(
			"绕障路径仍为预览：请先在「碰撞与规划」点「确认插入中间点」，再运行；否则按原指令直线规划，易穿模失败。"));
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	ensureInstructionControllerKinematics(doc, instIdx, urdfPath);
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(
				QStringLiteral(
					"No revolute joints in URDF (joints need type=\"revolute\" or \"continuous\" and an axis)."),
				QStringLiteral("URDF 中无可旋转关节（需 type=“revolute/continuous” 及 axis）。")));
		}
		return;
	}
	const std::vector<std::shared_ptr<RobotInstruction::Base>> instructions =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	if (instructions.empty())
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("Add at least one instruction row."), QStringLiteral("请至少添加一条指令。")));
		}
		return;
	}
	const std::vector<const RobotInstruction::Base*> motions =
		RobotInstruction::collectMotionInstructions(instructions);
	const QStringList jnamesAll = doc->robotRevoluteJointNames();
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath, m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	QVector<double> initialAngles(jnamesAll.size(), 0.0);
	bool usedFrozenStart = false;
	if (m_motionPreviewProgramStartJointRad.size() == jnamesAll.size())
	{
		initialAngles = m_motionPreviewProgramStartJointRad;
		usedFrozenStart = true;
	}
	else if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		const QVector<double> local = m_host->robotAxisControlPage()->jointAnglesRad();
		for (int j = 0; j < nj && jointOffset + j < initialAngles.size(); ++j)
		{
			initialAngles[jointOffset + j] = local[j];
		}
	}
	// 冻结起点全零但轴控已有位姿：用轴控纠正（避免未预览直接 Run 全失败）
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointCount() == nj)
	{
		const QVector<double> axisQ = m_host->robotAxisControlPage()->jointAnglesRad();
		bool frozenNearZero = true;
		for (int j = 0; j < nj && jointOffset + j < initialAngles.size(); ++j)
		{
			if (std::abs(initialAngles[jointOffset + j]) > 1e-6)
			{
				frozenNearZero = false;
				break;
			}
		}
		bool axisMeaningful = false;
		for (int j = 0; j < axisQ.size(); ++j)
		{
			if (std::abs(axisQ[j]) > 1e-3)
			{
				axisMeaningful = true;
				break;
			}
		}
		if ((!usedFrozenStart || (frozenNearZero && axisMeaningful)) && axisQ.size() == nj)
		{
			for (int j = 0; j < nj && jointOffset + j < initialAngles.size(); ++j)
			{
				initialAngles[jointOffset + j] = axisQ[j];
			}
			if (frozenNearZero && axisMeaningful)
			{
				m_motionPreviewProgramStartJointRad = initialAngles;
			}
		}
	}
	m_aggregatedJointAnglesRad = initialAngles;
	const QVector<double> playbackStartAngles = initialAngles;

	std::vector<RobotInstruction::PlanResult> planResults;
	planResults.reserve(motions.size());
	QVector<double> rollingQ(nj, 0.0);
	for (int j = 0; j < nj; ++j)
	{
		rollingQ[j] = initialAngles[jointOffset + j];
	}
	const QVector<double> programStartQ = rollingQ;
	std::vector<double> rollingExternalAxisQ = doc->robotExternalAxisQ(instIdx);
	const RobotExternal::RobotExternalAxisConfigSet& extAxesForRun = doc->robotExternalAxesForInstance(instIdx);
	int successMotionCount = 0;
	bool planningStoppedAfterFailure = false;
	size_t firstFailedMotionIndex = motions.size();
	QString firstFailedMotionLabel;
	QString firstFailedReason;
	// 前 16 段急切规划；其余 lazyPending，回放时再 IK
	constexpr size_t kEagerPlanCount = 16;
	for (size_t mi = 0; mi < motions.size(); ++mi)
	{
		const RobotInstruction::Base* motionPtr = motions[mi];
		if (!motionPtr)
		{
			if (m_host->runInfoPage())
			{
				m_host->appendRunWarning(
					m_host->i18n(QStringLiteral("Instruction row is invalid."), QStringLiteral("指令行无效。")));
			}
			return;
		}

		// 先前失败后：后续段 ok=false 占位，仍对齐 motions
		if (planningStoppedAfterFailure)
		{
			RobotInstruction::PlanResult skipped{};
			skipped.ok = false;
			skipped.plannerName = "skippedAfterFailure";
			skipped.summary = "Skipped after earlier planning failure";
			planResults.push_back(std::move(skipped));
			continue;
		}

		// 超出急切窗口：写入 lazyPending，延后规划
		if (mi >= kEagerPlanCount)
		{
			RobotInstruction::PlanResult pending{};
			pending.ok = false;
			pending.plannerName = "lazyPending";
			pending.summary = "Deferred until playback";
			planResults.push_back(std::move(pending));
			continue;
		}

		RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motionPtr);
		const QString insIdQ = QString::fromStdString(ins->id());
		const QString fp = computePlanFingerprint(*ins, rollingQ, urdfPath, defaultTcpLinkName);
		const RobotCoordinate::RobotCoordinateFrameSet& framesForRun = doc->robotCoordinateFramesForInstance(instIdx);
		std::string planErr;
		RobotInstruction::PlanResult plan{};
		const bool planOk = planMotionConsistentWithPreview(*ins, rollingQ, programStartQ, instIdx, urdfPath,
															defaultTcpLinkName, robotBackendId, framesForRun, plan,
															&planErr, true);
		if (!planOk)
		{
			RobotInstruction::PlanResult failed{};
			failed.ok = false;
			failed.plannerName = "failed";
			failed.summary = planErr.empty() ? "Instruction planning failed" : planErr;
			const std::string summaryCopy = failed.summary;
			planResults.push_back(std::move(failed));
			planningStoppedAfterFailure = true;
			firstFailedMotionIndex = mi;
			firstFailedMotionLabel = QString::fromStdString(ins->name());
			if (firstFailedMotionLabel.isEmpty())
			{
				firstFailedMotionLabel = insIdQ;
			}
			firstFailedReason = QString::fromStdString(summaryCopy);
			if (m_host->runInfoPage())
			{
				m_host->appendRunWarning(QString::fromStdString(summaryCopy));
			}
			continue;
		}
		if (!plan.jointTargetsRad.empty() && plan.jointTargetsRad.size() == static_cast<size_t>(rollingQ.size()))
		{
			for (int j = 0; j < rollingQ.size(); ++j)
			{
				rollingQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
		}
		if (plan.durationSec > 1e-6)
		{
			ins->setExtensionProperty("motion.durationSec", QString::number(plan.durationSec, 'f', 3).toStdString());
		}
		writeExternalAxisPlanToInstruction(*ins, plan, doc, instIdx);
		if (plan.hasExternalAxisQ)
		{
			std::vector<double> targetQs =
				!plan.externalAxisQs.empty() ? padExternalAxisQToConfig(extAxesForRun, plan.externalAxisQs)
											 : expandScalarExternalAxisQ(extAxesForRun, plan.externalAxisQ);
			plan.durationSec =
				std::max(plan.durationSec, externalAxisTravelDurationSec(extAxesForRun, rollingExternalAxisQ, targetQs));
			rollingExternalAxisQ = std::move(targetQs);
			if (plan.durationSec > 1e-6)
			{
				ins->setExtensionProperty("motion.durationSec", QString::number(plan.durationSec, 'f', 3).toStdString());
			}
		}
		// 保留 jointTrajectoryRad：LINE/点云路径插帧依赖多样本轨迹
		m_planResultCache.store(insIdQ, fp, plan, mi);
		planResults.push_back(std::move(plan));
		++successMotionCount;
	}
	if (successMotionCount <= 0)
	{
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("All motion instructions failed to plan; simulation not started."), QStringLiteral("所有运动指令规划失败，未启动仿真。")));
		}
		return;
	}
	if (planningStoppedAfterFailure && m_host->runInfoPage())
	{
		const int failOneBased = static_cast<int>(firstFailedMotionIndex) + 1;
		m_host->appendRunWarning(m_host->i18n(QStringLiteral("Partial plan failure: will play until motion %1 (%2), then stop. Reason: %3")
				.arg(failOneBased)
				.arg(firstFailedMotionLabel)
				.arg(firstFailedReason), QStringLiteral("部分规划失败：将播放至第 %1 条（%2）后停止。原因：%3")
				.arg(failOneBased)
				.arg(firstFailedMotionLabel)
				.arg(firstFailedReason)));
	}
	else if (motions.size() > kEagerPlanCount && m_host->runInfoPage())
	{
		m_host->appendRunInfo(m_host->i18n(QStringLiteral("Lazy planning: %1/%2 motions planned at start; rest on demand.")
				.arg(successMotionCount)
				.arg(static_cast<int>(motions.size())), QStringLiteral("懒加载规划：启动时已规划 %1/%2 条，其余按需规划。")
				.arg(successMotionCount)
				.arg(static_cast<int>(motions.size()))));
	}
	// 关掉 CAD raw 双轨；若此前开着 raw，setRawTrajectoryPreviewActive 内已恢复指令轴，勿再全量重建
	setRawTrajectoryPreviewActive(false);
	if (m_host->simulationCommandPage())
	{
		m_host->simulationCommandPage()->refreshInstructionList();
	}
	QString err;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	m_currentRunMotions.clear();
	m_currentRunMotions.reserve(motions.size());
	for (const RobotInstruction::Base* motion : motions)
	{
		m_currentRunMotions.push_back(motion);
	}
	m_lastHighlightedInstructionId.clear();
	m_playbackMotionIndex = 0;
	m_playbackProgramStartQ = programStartQ;
	m_playbackRollingSeedQ = programStartQ;
	m_playbackSegmentExternalAxisStart =
		doc ? toQVector(doc->robotExternalAxisQ(instIdx)) : QVector<double>();
	m_playbackExtInterpMotion = nullptr;
	IRobotIoSink* ioSink = nullptr;
	if (m_ioNetwork && doc)
	{
		const QString robotId = doc->robotSceneBackendIdForInstance(instIdx);
		ioSink = m_ioNetwork->sink(robotId);
	}
	if (!poseSink || !ioSink ||
		!m_programExecutor.tryStart(doc, poseSink, ioSink, instIdx, instructions, planResults, playbackStartAngles,
									&err))
	{
		if (m_host->runInfoPage())
		{
			if (err.contains(QLatin1String("Invalid joint index")))
			{
				m_host->appendRunWarning(m_host->i18n(QStringLiteral("Invalid joint index in simulation command."), QStringLiteral("仿真指令关节索引无效。")));
			}
			else if (!err.isEmpty())
			{
				m_host->appendRunWarning(err);
			}
		}
		return;
	}
	// tryStart 后同步控件倍率（运行中改倍率走 playbackRateChanged）
	if (m_host->simulationCommandPage())
	{
		m_programExecutor.setPlaybackRate(m_host->simulationCommandPage()->playbackRate());
	}
	if (m_host->robotAxisControlPage())
	{
		m_host->robotAxisControlPage()->setInteractionEnabled(false);
	}
	m_host->simulationCommandPage()->setSimulationRunning(true);
	if (m_simulationDock && m_simulationDock->trajectoryEditPage())
	{
		m_simulationDock->trajectoryEditPage()->setReadOnly(true);
	}
	if (m_playbackTimer)
	{
		m_playbackTimer->start();
	}
	if (m_host->runInfoPage())
	{
		m_host->appendRunInfo(m_host->i18n(QStringLiteral("Simulation started."), QStringLiteral("仿真已开始。")));
	}
}

void RobotSimulationController::logPlaybackFrameComparison(const QVector<double>& finalJointAnglesRad)
{
	if (!m_host->runInfoPage())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !osg || !m_host->simulationCommandPage() || finalJointAnglesRad.isEmpty())
	{
		return;
	}
	const auto insList = m_host->simulationCommandPage()->instructionList();
	std::shared_ptr<RobotInstruction::Base> targetIns;
	for (auto it = insList.rbegin(); it != insList.rend(); ++it)
	{
		if (*it && (*it)->hasPoseProperty())
		{
			targetIns = *it;
			break;
		}
	}
	if (!targetIns)
	{
		return;
	}

	const RobotInstruction::Vec3 targetPose = targetIns->pose();
	QString tcpLinkName;
	{
		const auto& ext = targetIns->extensionProperties();
		const auto itCaptured = ext.find("context.capturedTcpLinkName");
		if (itCaptured != ext.end() && !itCaptured->second.empty())
		{
			tcpLinkName = QString::fromStdString(itCaptured->second);
		}
		if (tcpLinkName.isEmpty())
		{
			const auto itTcp = ext.find("context.tcpLinkName");
			if (itTcp != ext.end() && !itTcp->second.empty())
			{
				tcpLinkName = QString::fromStdString(itTcp->second);
			}
		}
	}

	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	QHash<QString, osg::Matrixd> linkWorldByName;
	QString fkErr;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, finalJointAnglesRad, linkWorldByName, &fkErr))
	{
		m_host->appendRunWarning(m_host->i18n(QStringLiteral("Forward kinematics failed: %1").arg(fkErr), QStringLiteral("正解失败：%1").arg(fkErr)));
		return;
	}
}

void RobotSimulationController::onRobotSimulationTick()
{
	if (!m_programExecutor.isRunning())
	{
		if (m_playbackTimer)
		{
			m_playbackTimer->stop();
		}
		return;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	const bool uiBusy = isPlaybackUiInteractionBusy();
	// 先把前瞻结果灌进执行器，再段前补算，避免每段都在 UI 线程现算
	tickLookaheadPlanning();
	ensurePlaybackPlansReady();
	const RobotInstructionPlaybackTickResult r = m_programExecutor.tick(doc, poseSink);
	m_aggregatedJointAnglesRad = m_programExecutor.jointAnglesRad();
	if (doc && osg)
	{
		if (const RobotInstruction::Base* activeMotion = m_programExecutor.activeMotion())
		{
			const int playInst = m_host->simulationCommandPage()
									 ? m_host->simulationCommandPage()->currentRobotInstanceIndex()
									 : -1;
			const int instForPlay = playInst >= 0 ? playInst : 0;
			if (const RobotInstruction::PlanResult* playPlan = m_programExecutor.motionPlanResult(activeMotion))
			{
				// 段切换时记下起点 qe，避免每 tick 直接落到目标造成滑轨瞬移
				if (activeMotion != m_playbackExtInterpMotion)
				{
					m_playbackExtInterpMotion = activeMotion;
					m_playbackSegmentExternalAxisStart = toQVector(doc->robotExternalAxisQ(instForPlay));
				}
				if (playPlan->ok)
				{
					applyExternalAxisFromPlan(instForPlay, *playPlan, activeMotion,
											  m_programExecutor.motionSegmentProgress01(),
											  m_playbackSegmentExternalAxisStart);
				}
			}
		}
		if (!uiBusy)
		{
			refreshRobotCoordinateFrameOverlaysForPlayback();
		}
		osg->requestRedraw();

		if (SimulationCommandWidget* cmd = m_host->simulationCommandPage())
		{
			if (InstructionProgramTreeWidget* tree = cmd->instructionTree())
			{
				const RobotInstruction::Base* curIns = m_programExecutor.currentInstruction();
				if (curIns && curIns->id() != m_lastHighlightedInstructionId)
				{
					const QSignalBlocker blocker(tree);
					tree->selectInstructionByRaw(const_cast<RobotInstruction::Base*>(curIns));
					m_lastHighlightedInstructionId = curIns->id();
					// SignalBlocker 挡掉选中信号，此处同步同一套游标/编号
					for (const std::shared_ptr<RobotInstruction::Base>& ins : cmd->instructionList())
					{
						if (ins && ins.get() == curIns)
						{
							refreshPlaybackPathOverlays(ins);
							break;
						}
					}
				}
			}
		}
		tickLookaheadPlanning();
	}
	switch (r)
	{
	case RobotInstructionPlaybackTickResult::Continue:
		break;
	case RobotInstructionPlaybackTickResult::Finished:
		logPlaybackFrameComparison(m_programExecutor.jointAnglesRad());
		refreshRobotCoordinateFrameOverlaysForPlayback();
		stopRobotSimulation();
		if (m_host->runInfoPage())
		{
			m_host->appendRunInfo(m_host->i18n(QStringLiteral("Simulation finished."), QStringLiteral("仿真已结束。")));
		}
		break;
	case RobotInstructionPlaybackTickResult::Aborted:
	{
		const bool dueToPlanFail = m_programExecutor.abortedDueToFailedPlan();
		const QString abortSummary = QString::fromStdString(m_programExecutor.lastAbortSummary());
		const RobotInstruction::Base* failedIns = m_programExecutor.activeMotion();
		QString failedLabel;
		if (failedIns)
		{
			failedLabel = QString::fromStdString(failedIns->name());
			if (failedLabel.isEmpty())
			{
				failedLabel = QString::fromStdString(failedIns->id());
			}
		}
		stopRobotSimulation();
		if (dueToPlanFail && m_host->runInfoPage())
		{
			m_host->appendRunWarning(m_host->i18n(QStringLiteral("Simulation stopped before failed motion%1. %2")
					.arg(failedLabel.isEmpty() ? QString() : QStringLiteral(" (%1)").arg(failedLabel))
					.arg(abortSummary), QStringLiteral("仿真已在失败运动前停止%1。%2")
					.arg(failedLabel.isEmpty() ? QString() : QStringLiteral("（%1）").arg(failedLabel))
					.arg(abortSummary)));
		}
		break;
	}
	}
}

bool RobotSimulationController::planMotionOnHost(RobotInstruction::Base& instruction,
												 const QVector<double>& seedJointRad, const int instanceIndex,
												 const QString& urdfPath, const QString& defaultTcpLinkName,
												 const QString& sceneRootBackendId, RobotInstruction::PlanResult& plan,
												 std::string* planErr, const bool skipValidate,
												 const bool skipCollision) const
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || !m_host)
	{
		if (planErr)
		{
			*planErr = "missing host/document";
		}
		return false;
	}
	(void)defaultTcpLinkName;
	(void)sceneRootBackendId;
	// Run/Preview 直调本地 Controller，避免 Host DTO 丢掉 duration/外轴/轨迹
	RobotSimulationController* self = const_cast<RobotSimulationController*>(this);
	self->ensureInstructionControllerKinematics(doc, instanceIndex, urdfPath);
	std::string err;
	plan = {};
	if (!skipValidate && !self->m_instructionController.validate(instruction, &err))
	{
		if (planErr)
		{
			*planErr = err.empty() ? "Instruction validation failed" : err;
		}
		return false;
	}
	if (!self->m_instructionController.plan(instruction, plan, &err) || !plan.ok)
	{
		if (planErr)
		{
			*planErr = plan.summary.empty() ? (err.empty() ? "Instruction planning failed" : err) : plan.summary;
		}
		return false;
	}

	if (!skipCollision)
	{
		const RobotCollision::Settings& col = doc->robotCollisionSettings();
		if (col.enabled)
		{
			std::string colErr;
			const QVector<double> seedBefore = seedJointRad;
			if (!BackendCollisionSync::validateJointTrajectory(*self->m_collisionWorld, doc, doc->backend(),
															   instanceIndex, seedBefore, plan.jointTrajectoryRad, col,
															   &colErr, m_host->osgView()))
			{
				plan.ok = false;
				plan.summary = colErr.empty() ? "Collision detected" : colErr;
				if (planErr)
					*planErr = plan.summary;
				return false;
			}
		}
	}
	return true;
}

bool RobotSimulationController::planMotionConsistentWithPreview(
	RobotInstruction::Base& instruction, const QVector<double>& chainSeedQ, const QVector<double>& programStartQ,
	const int instanceIndex, const QString& urdfPath, const QString& defaultTcpLinkName,
	const QString& sceneRootBackendId, const RobotCoordinate::RobotCoordinateFrameSet& frames,
	RobotInstruction::PlanResult& outPlan, std::string* planErr, const bool persistTaughtOnSuccess,
	const bool gateTaughtResidual, const bool skipTaughtShortCircuit)
{
	outPlan = {};
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!doc || !m_host)
	{
		if (planErr)
		{
			*planErr = "missing host/document";
		}
		return false;
	}
	const int nj = chainSeedQ.size() > 0 ? chainSeedQ.size() : programStartQ.size();
	if (nj <= 0)
	{
		if (planErr)
		{
			*planErr = "empty seed";
		}
		return false;
	}

	(void)skipTaughtShortCircuit;
	(void)gateTaughtResidual;

	const QString insIdQ = QString::fromStdString(instruction.id());
	const QString planFp = computePlanFingerprint(instruction, chainSeedQ, urdfPath, defaultTcpLinkName);
	if (const RobotInstruction::PlanResult* cached = m_planResultCache.fetch(insIdQ, planFp))
	{
		if (cached->ok && cached->jointTargetsRad.size() == static_cast<size_t>(nj))
		{
			QVector<double> cachedQ(nj);
			for (int j = 0; j < nj; ++j)
			{
				cachedQ[j] = cached->jointTargetsRad[static_cast<size_t>(j)];
			}
			if (isTaughtOrCacheReuseAcceptable(
					targetResidualMmForInstruction(urdfPath, cachedQ, frames, defaultTcpLinkName, instruction),
					targetOrientationResidualDegForInstruction(urdfPath, cachedQ, frames, defaultTcpLinkName,
															   instruction)))
			{
				RobotInstruction::PlanResult plan = *cached;
				std::vector<double> refQ(chainSeedQ.begin(), chainSeedQ.end());
				QVector<double> limLo, limHi;
				doc->robotJointLimitsForInstance(instanceIndex, limLo, limHi);
				const std::vector<double> targetsBeforeNorm = plan.jointTargetsRad;
				bool cacheOk = normalizeJointRevolutionsToReference(plan.jointTargetsRad, refQ, limLo, limHi);
				if (cacheOk)
				{
					alignTrajectoryAfterTargetNormalize(plan, targetsBeforeNorm);
					const RobotCollision::Settings& colGate = doc->robotCollisionSettings();
					if (colGate.enabled && m_collisionWorld && !m_programExecutor.isRunning())
					{
						std::vector<std::vector<double>> seg = plan.jointTrajectoryRad;
						if (seg.empty())
						{
							seg.push_back(plan.jointTargetsRad);
						}
						std::string colErr;
						if (!BackendCollisionSync::validateJointTrajectory(*m_collisionWorld, doc, doc->backend(),
																		   instanceIndex, chainSeedQ, seg, colGate,
																		   &colErr, m_host->osgView(),
																		   /*rebuildWorldFirst=*/false))
						{
							cacheOk = false;
						}
					}
				}
				if (cacheOk)
				{
					outPlan = std::move(plan);
					if (persistTaughtOnSuccess)
					{
						RobotInstructionPlanning::persistTaughtJointsAndToolContext(instruction, cachedQ, frames);
					}
					return true;
				}
				m_planResultCache.invalidateByInstruction(insIdQ);
			}
		}
	}

	auto seedsDiffer = [&](const QVector<double>& a, const QVector<double>& b) -> bool
	{
		if (a.size() != b.size())
		{
			return true;
		}
		for (int j = 0; j < a.size(); ++j)
		{
			if (std::abs(a[j] - b[j]) > 1e-9)
			{
				return true;
			}
		}
		return false;
	};

	QVector<double> currentPoseQ = programStartQ;
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instanceIndex);
	if (m_host->robotAxisControlPage() && m_host->robotAxisControlPage()->jointAnglesRad().size() == nj)
	{
		currentPoseQ = m_host->robotAxisControlPage()->jointAnglesRad();
	}
	else if (m_aggregatedJointAnglesRad.size() >= jointOffset + nj && nj > 0)
	{
		currentPoseQ = m_aggregatedJointAnglesRad.mid(jointOffset, nj);
	}

	QVector<double> refInstructionQ = chainSeedQ;
	const bool explicitSeedIns =
		m_ikSeedPolicy == RobotInstruction::IkSeedPolicy::FromInstruction && !m_ikSeedInstructionId.empty();
	if (explicitSeedIns)
	{
		refInstructionQ.clear();
		if (const RobotInstruction::PlanResult* hit =
				m_planResultCache.findLatestOkByInstructionId(QString::fromStdString(m_ikSeedInstructionId)))
		{
			if (hit->jointTargetsRad.size() == static_cast<size_t>(nj))
			{
				refInstructionQ.resize(nj);
				for (int j = 0; j < nj; ++j)
				{
					refInstructionQ[j] = hit->jointTargetsRad[static_cast<size_t>(j)];
				}
			}
		}
		if (refInstructionQ.size() != nj)
		{
			if (planErr)
			{
				*planErr = "seed instruction has no planned joints yet; plan the referenced waypoint first";
			}
			return false;
		}
	}

	RobotInstruction::IkRequest ikReq;
	ikReq.seedPolicy = m_ikSeedPolicy;
	ikReq.seedInstructionId = m_ikSeedInstructionId;
	std::vector<double> currentStd(currentPoseQ.begin(), currentPoseQ.end());
	std::vector<double> refStd(refInstructionQ.begin(), refInstructionQ.end());
	RobotInstruction::resolveIkRequestSeed(ikReq, currentStd, refStd);
	QVector<double> primarySeed;
	primarySeed.reserve(static_cast<int>(ikReq.qSeedResolved.size()));
	for (double v : ikReq.qSeedResolved)
	{
		primarySeed.push_back(v);
	}
	if (m_ikSeedPolicy == RobotInstruction::IkSeedPolicy::FromCurrentPose && primarySeed.size() != nj)
	{
		if (planErr)
		{
			*planErr = "current pose seed unavailable";
		}
		return false;
	}

	QVector<QVector<double>> seedOrder;
	auto pushSeed = [&](const QVector<double>& s)
	{
		if (s.size() != nj)
		{
			return;
		}
		for (const QVector<double>& existing : seedOrder)
		{
			if (!seedsDiffer(existing, s))
			{
				return;
			}
		}
		seedOrder.push_back(s);
	};
	pushSeed(primarySeed);
	// 链式：确定性多种子；当前位姿：轴控优先，仍兜底链式/程序起点（否则未预览直接 Run 易全败）
	if (m_ikSeedPolicy == RobotInstruction::IkSeedPolicy::FromInstruction && !explicitSeedIns)
	{
		pushSeed(chainSeedQ);
		pushSeed(programStartQ);
	}
	else if (m_ikSeedPolicy == RobotInstruction::IkSeedPolicy::FromCurrentPose)
	{
		pushSeed(chainSeedQ);
		pushSeed(programStartQ);
	}
	if (seedOrder.isEmpty())
	{
		if (planErr)
		{
			*planErr = "empty IK seed list";
		}
		return false;
	}

	const RobotInstructionPlanning::MotionPoseBackup backup =
		RobotInstructionPlanning::backupInstructionPose(instruction);
	std::string lastErr;
	bool planned = false;
	QVector<double> resultQ;
	QVector<double> jointLowerRad, jointUpperRad;
	doc->robotJointLimitsForInstance(instanceIndex, jointLowerRad, jointUpperRad);

	// 同一次多种子终接受只 rebuild 一次碰撞世界
	bool collisionWorldReady = false;
	const RobotCollision::Settings& colSettings = doc->robotCollisionSettings();
	if (colSettings.enabled && m_collisionWorld && m_host && !m_programExecutor.isRunning())
	{
		BackendCollisionSync::rebuildWorld(*m_collisionWorld, doc, doc->backend(), colSettings, m_host->osgView());
		collisionWorldReady = true;
	}

	// 接受尾段：转数归一化（R1→chainSeed）→ 尾点一致性 → 非链种子重锚/降级 → 写出
	auto finalizeAccept = [&](RobotInstruction::PlanResult& plan, const QVector<double>& trySeed) -> bool
	{
		std::vector<double> refQ(chainSeedQ.begin(), chainSeedQ.end());
		const std::vector<double> targetsBeforeNorm = plan.jointTargetsRad;
		if (!normalizeJointRevolutionsToReference(plan.jointTargetsRad, refQ, jointLowerRad, jointUpperRad))
		{
			lastErr = describeJointNormalizeFailure(targetsBeforeNorm, refQ, jointLowerRad, jointUpperRad);
			return false;
		}
		alignTrajectoryAfterTargetNormalize(plan, targetsBeforeNorm);
		resultQ.resize(nj);
		for (int j = 0; j < nj; ++j)
		{
			resultQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
		}
		// 非链式种子：只锁肘/腕/臂重锚（不锁 turn=0，避免回到链式种子失败的那一支）；失败则剥轨迹
		if (seedsDiffer(trySeed, chainSeedQ) && chainSeedQ.size() == nj)
		{
			const RobotInstruction::MotionAxisConfiguration cfgBackup = instruction.motionAxisConfiguration();
			RobotInstruction::MotionAxisConfiguration lockCfg = cfgBackup;
			{
				const QStringList jnLock = doc->robotRevoluteJointNamesForInstance(instanceIndex);
				std::vector<std::string> jnLockStd;
				jnLockStd.reserve(static_cast<size_t>(jnLock.size()));
				for (const QString& jn : jnLock)
				{
					jnLockStd.push_back(jn.toStdString());
				}
				std::vector<double> qDiscovered(resultQ.begin(), resultQ.end());
				std::vector<double> qChain(chainSeedQ.begin(), chainSeedQ.end());
				const RobotInstruction::JointConfigurationClass observed =
					RobotInstruction::classifyJointConfiguration(qDiscovered, jnLockStd, &qChain);
				if (observed.elbow != RobotInstruction::ElbowPosture::Auto)
				{
					lockCfg.elbow = observed.elbow;
				}
				if (observed.wrist != RobotInstruction::WristPosture::Auto)
				{
					lockCfg.wrist = observed.wrist;
				}
				if (observed.arm != RobotInstruction::ArmPosture::Auto)
				{
					lockCfg.arm = observed.arm;
				}
			}
			instruction.setMotionAxisConfiguration(lockCfg);
			RobotInstruction::PlanResult anchored{};
			std::string anchorErr;
			RobotInstructionPlanning::prepareMotionInstructionForPlanning(
				instruction, chainSeedQ, doc, osg, instanceIndex, urdfPath, defaultTcpLinkName.toStdString(),
				&frames);
			instruction.eraseExtensionProperty("context.playbackPlanLite");
			bool anchorOk =
				planMotionOnHost(instruction, chainSeedQ, instanceIndex, urdfPath, defaultTcpLinkName,
								 sceneRootBackendId, anchored, &anchorErr, /*skipValidate=*/false,
								 /*skipCollision=*/true) &&
				anchored.ok && anchored.jointTargetsRad.size() == static_cast<size_t>(nj);
			if (anchorOk)
			{
				const std::vector<double> anchoredBefore = anchored.jointTargetsRad;
				if (!normalizeJointRevolutionsToReference(anchored.jointTargetsRad, refQ, jointLowerRad,
														  jointUpperRad))
				{
					anchorOk = false;
				}
				else
				{
					alignTrajectoryAfterTargetNormalize(anchored, anchoredBefore);
				}
			}
			if (anchorOk)
			{
				QVector<double> anchoredQ(nj);
				for (int j = 0; j < nj; ++j)
				{
					anchoredQ[j] = anchored.jointTargetsRad[static_cast<size_t>(j)];
				}
				anchorOk = isFreshIkSolutionAcceptable(
					targetResidualMmForInstruction(urdfPath, anchoredQ, frames, defaultTcpLinkName, instruction),
					targetOrientationResidualDegForInstruction(urdfPath, anchoredQ, frames, defaultTcpLinkName,
															   instruction));
				if (anchorOk)
				{
					resultQ = anchoredQ;
					plan = std::move(anchored);
				}
			}
			instruction.setMotionAxisConfiguration(cfgBackup);
			if (!anchorOk)
			{
				if (!plan.jointTrajectoryRad.empty())
				{
					plan.jointTrajectoryRad.clear();
					plan.summary += " | anchor re-plan failed, degraded to joint lerp";
				}
			}
		}
		// 种子尝试跳过碰撞：终接受统一验最终轨迹（稠密或端点 lerp）
		{
			const RobotCollision::Settings& colGate = doc->robotCollisionSettings();
			if (colGate.enabled && m_collisionWorld && !m_programExecutor.isRunning())
			{
				std::vector<std::vector<double>> seg = plan.jointTrajectoryRad;
				if (seg.empty())
				{
					seg.push_back(std::vector<double>(resultQ.begin(), resultQ.end()));
				}
				std::string colErr;
				if (!BackendCollisionSync::validateJointTrajectory(
						*m_collisionWorld, doc, doc->backend(), instanceIndex, chainSeedQ, seg, colGate, &colErr,
						m_host->osgView(), /*rebuildWorldFirst=*/!collisionWorldReady))
				{
					// Direct/指令轨迹穿模：有场景障碍时改走关节空间 OMPL（与碰撞页同源）
					bool omplOk = false;
					cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
					const int nLinks = static_cast<int>(
						doc->robotPerLinkKinematicsForInstance(instanceIndex, pl) ? pl.linkNameToBackendId.size() : 0);
					if (m_collisionWorld->bodyCount() > static_cast<std::size_t>(std::max(0, nLinks)) && nLinks > 0
						&& chainSeedQ.size() == nj && resultQ.size() == nj)
					{
						// 与碰撞页一致：先落到起点再采 OSG 绑定位姿，否则 nearStart 检碰因缺 linkWorldAtStart 直接失败
						QVector<double> aggOmpl;
						(void)doc->applyJointAnglesRad(instanceIndex, chainSeedQ, aggOmpl);
						BackendCollisionSync::updatePoses(*m_collisionWorld, doc, doc->backend(), m_host->osgView());

						robot_path::PlanRequest req;
						req.urdfPath = urdfPath;
						req.startJointRad.assign(chainSeedQ.begin(), chainSeedQ.end());
						req.goalJointRad.assign(resultQ.begin(), resultQ.end());
						req.world = m_collisionWorld.get();
						req.meshVerticesInLinkFrame = pl.meshVerticesInLinkFrame;
						req.options.checkCollision = true;
						req.options.allowDirectJointLerp = false;
						req.options.securityMarginMm = colGate.securityMarginMm;
						req.options.planningSpace = colGate.planningSpace.empty() ? "Auto" : colGate.planningSpace;
						req.options.plannerId = colGate.plannerId.empty() ? "Auto" : colGate.plannerId;
						req.options.planningTimeSec = colGate.planningTimeSec;
						req.robotBasePlacementWorld =
							RobotSimulationMath::osgMatrixFromCoreMat4(pl.robotBasePlacementWorld);
						for (auto it = pl.fkMeshWorldT0.constBegin(); it != pl.fkMeshWorldT0.constEnd(); ++it)
							req.fkMeshWorldT0.insert(it.key(), RobotSimulationMath::osgMatrixFromCoreMat4(it.value()));
						for (auto it = pl.outerWorldAtBindByBackendId.constBegin();
							 it != pl.outerWorldAtBindByBackendId.constEnd(); ++it)
							req.outerWorldAtBindByBackendId.insert(
								it.key(), RobotSimulationMath::osgMatrixFromCoreMat4(it.value()));
						auto backendFromOsgPacked = [](const cloudsim::core::Mat4& packed) {
							return RobotMatrixOsg::backendColMajorFromMatrix(
								RobotSimulationMath::osgMatrixFromCoreMat4(packed));
						};
						for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd();
							 ++it)
						{
							collision::CollisionBodyId id;
							id.kind = "robotLink";
							id.backendId = it.value().toStdString();
							id.linkName = it.key().toStdString();
							req.linkBodies.insert(it.key(), id);
							if (osg)
							{
								cloudsim::core::Mat4 wCm{};
								if (osg->getBackendRootWorldMatrix(it.value().toStdString(), wCm))
									req.linkWorldAtStart.insert(it.key(), backendFromOsgPacked(wCm));
							}
						}
						{
							const QString refBid = doc->robotFrameWorldReferenceBackendId(instanceIndex);
							QString refLink;
							for (auto it = pl.linkNameToBackendId.constBegin();
								 it != pl.linkNameToBackendId.constEnd(); ++it)
							{
								if (it.value() == refBid)
								{
									refLink = it.key();
									break;
								}
							}
							if (!refLink.isEmpty())
							{
								QHash<QString, osg::Matrixd> meshFk;
								QString fkErr;
								if (UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, chainSeedQ, meshFk, &fkErr,
																			  pl.meshVerticesInLinkFrame))
								{
									const auto fkIt = meshFk.constFind(refLink);
									auto be = doc->backend().getData(refBid.toStdString());
									if (fkIt != meshFk.constEnd() && be)
									{
										const engine::RigidTransform T_be = engine::rigidTransformFromColMajor([&] {
											engine::ColMajorMat4 cm{};
											BackendMat4 W = be->worldMatrix();
											cloudsim::core::Mat4 osgPacked{};
											if (osg && osg->getBackendRootWorldMatrix(refBid.toStdString(), osgPacked))
											{
												W = RobotMatrixOsg::backendColMajorFromMatrix(
													RobotSimulationMath::osgMatrixFromCoreMat4(osgPacked));
											}
											for (int i = 0; i < 16; ++i)
												cm[static_cast<size_t>(i)] = W.v[i];
											return cm;
										}());
										const engine::RigidTransform T_fk = engine::rigidTransformFromOsg(*fkIt);
										const engine::RigidTransform T_wb = T_be.composeScene(T_fk.inverse());
										const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(T_wb);
										for (int i = 0; i < 16; ++i)
											req.T_world_urdfBase.v[i] = cm[static_cast<size_t>(i)];
									}
								}
							}
						}
						const RobotCoordinate::RobotToolFrame* activeTool =
							RobotCoordinate::activeToolFrame(frames);
						QString flangeLink =
							activeTool ? QString::fromStdString(
											 RobotCoordinate::effectiveFlangeLinkName(frames, *activeTool))
									   : QString();
						if (flangeLink.isEmpty())
							flangeLink = defaultTcpLinkName;
						req.flangeLinkName = flangeLink;
						if (activeTool)
							req.T_flange_tool = RobotCoordinate::frameToMat4(activeTool->T_flange_tool);
						robot_path::PathResult ompl;
						if (robot_path::planToTcpPose(req, ompl) && ompl.ok && !ompl.jointTrajectoryRad.empty())
						{
							std::string omplColErr;
							if (BackendCollisionSync::validateJointTrajectory(
									*m_collisionWorld, doc, doc->backend(), instanceIndex, chainSeedQ,
									ompl.jointTrajectoryRad, colGate, &omplColErr, m_host->osgView(),
									/*rebuildWorldFirst=*/false))
							{
								plan.jointTrajectoryRad = std::move(ompl.jointTrajectoryRad);
								plan.plannerName = ompl.plannerName.empty() ? "OMPL" : ompl.plannerName;
								plan.summary += " | ompl avoidance after direct collision";
								omplOk = true;
							}
							else
							{
								colErr = omplColErr.empty() ? colErr : omplColErr;
							}
						}
						else if (!ompl.errMsg.empty())
						{
							colErr = colErr + "; ompl: " + ompl.errMsg;
						}
					}
					if (!omplOk)
					{
						lastErr = colErr.empty() ? "Collision detected" : colErr;
						return false;
					}
				}
			}
		}
		outPlan = std::move(plan);
		writeExternalAxisPlanToInstruction(instruction, outPlan, doc, instanceIndex);
		return true;
	};

	// LINE 可走 lite；与 Preview/Run 种子策略一致。
	// collectNearest=true（随机兜底）时不在首个过门种子处收口：盒内收集全部过门候选，
	// 经连续性硬门（R3）后按距链式种子 SSE 排序，终接受（含碰撞）逐个尝试直至成功
	auto trySeedListPass = [&](const QVector<QVector<double>>& seeds, const bool useLite,
							   const bool collectNearest, const qint64 budgetMs) -> bool
	{
		QElapsedTimer budget;
		budget.start();
		struct NearestCand
		{
			double sse = 0.0;
			double wristJump = 0.0;
			RobotInstruction::PlanResult plan{};
			QVector<double> seed;
		};
		std::vector<NearestCand> nearestCands;
		const QStringList jnQ = doc->robotRevoluteJointNamesForInstance(instanceIndex);
		std::vector<std::string> jointNames;
		jointNames.reserve(static_cast<size_t>(jnQ.size()));
		for (const QString& n : jnQ)
		{
			jointNames.push_back(n.toStdString());
		}
		std::vector<double> chainSeedStd(chainSeedQ.begin(), chainSeedQ.end());
		for (const QVector<double>& trySeed : seeds)
		{
			if (collectNearest && budgetMs > 0 && budget.elapsed() >= budgetMs)
			{
				break;
			}
			RobotInstructionPlanning::prepareMotionInstructionForPlanning(
				instruction, trySeed, doc, osg, instanceIndex, urdfPath, defaultTcpLinkName.toStdString(), &frames);
			if (useLite)
			{
				instruction.setExtensionProperty("context.playbackPlanLite", "1");
			}
			else
			{
				instruction.eraseExtensionProperty("context.playbackPlanLite");
			}
			std::string seedErr;
			if (!m_instructionController.validate(instruction, &seedErr))
			{
				lastErr = seedErr.empty() ? "Instruction validation failed" : seedErr;
				instruction.eraseExtensionProperty("context.playbackPlanLite");
				continue;
			}
			RobotInstruction::PlanResult plan{};
			if (!planMotionOnHost(instruction, trySeed, instanceIndex, urdfPath, defaultTcpLinkName, sceneRootBackendId,
								  plan, &seedErr, /*skipValidate=*/true, /*skipCollision=*/true) ||
				!plan.ok)
			{
				lastErr = plan.summary.empty() ? (seedErr.empty() ? "IK???" : seedErr) : plan.summary;
				instruction.eraseExtensionProperty("context.playbackPlanLite");
				continue;
			}
			instruction.eraseExtensionProperty("context.playbackPlanLite");
			if (plan.jointTargetsRad.empty() || plan.jointTargetsRad.size() != static_cast<size_t>(nj))
			{
				lastErr = seedErr.empty() ? "IK?????????" : seedErr;
				continue;
			}
			resultQ.resize(nj);
			for (int j = 0; j < nj; ++j)
			{
				resultQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
			const double residualMm =
				targetResidualMmForInstruction(urdfPath, resultQ, frames, defaultTcpLinkName, instruction);
			const double orientDeg =
				targetOrientationResidualDegForInstruction(urdfPath, resultQ, frames, defaultTcpLinkName, instruction);
			if (!isFreshIkSolutionAcceptable(residualMm, orientDeg))
			{
				char gateBuf[192];
				std::snprintf(gateBuf, sizeof(gateBuf),
							  "%s residual exceeds gate (pos=%.2fmm gate=%.1fmm, orient=%.2fdeg gate=%.1fdeg)",
							  (residualMm < 0.0 || residualMm > kFreshIkPositionRejectMm) ? "position" : "orientation",
							  residualMm, kFreshIkPositionRejectMm, orientDeg, kMaxFreshIkOrientDeg);
				lastErr = gateBuf;
				continue;
			}
			const double wristJump =
				RobotInstruction::wristConfigurationJumpRad(plan.jointTargetsRad, chainSeedStd, jointNames);
			const bool wristCont = wristJump <= RobotInstruction::kWristContinuityGateRad;
			if (collectNearest)
			{
				const double maxAbs = maxFoldedJointAbsDelta(resultQ, chainSeedQ);
				const double sse = unfoldedJointSse(resultQ, chainSeedQ);
				if (maxAbs > kFallbackConsistencyRad)
				{
					lastErr = "random fallback candidate exceeds continuity gate";
					continue;
				}
				nearestCands.push_back(NearestCand{sse, wristJump, std::move(plan), trySeed});
				if (maxAbs <= kFallbackNearEnoughRad && wristCont)
				{
					break;
				}
				continue;
			}
			// 确定性种子：先收齐过门候选，按腕连续优先再终接受（避免硬拒导致全败）
			nearestCands.push_back(NearestCand{0.0, wristJump, std::move(plan), trySeed});
		}
		if (!nearestCands.empty())
		{
			std::sort(nearestCands.begin(), nearestCands.end(),
					  [](const NearestCand& a, const NearestCand& b)
					  {
						  const bool aCont = a.wristJump <= RobotInstruction::kWristContinuityGateRad;
						  const bool bCont = b.wristJump <= RobotInstruction::kWristContinuityGateRad;
						  if (aCont != bCont)
						  {
							  return aCont;
						  }
						  if (std::abs(a.wristJump - b.wristJump) > 1e-9)
						  {
							  return a.wristJump < b.wristJump;
						  }
						  return a.sse < b.sse;
					  });
			for (NearestCand& cand : nearestCands)
			{
				if (finalizeAccept(cand.plan, cand.seed))
				{
					return true;
				}
			}
		}
		return false;
	};

	// PTP 的 lite 不减 DLS，只白付一遍；LINE/ARC lite 才压缩采样
	const bool tryLiteFirst = instruction.type() != RobotInstruction::Type::PTP;
	planned = (tryLiteFirst && trySeedListPass(seedOrder, true, false, 0)) ||
			  trySeedListPass(seedOrder, false, false, 0);
	if (!planned)
	{
		// 播放中禁止 UI 线程随机重启：280ms 盒会把整段回放卡死；失败留给 lookahead / 明确停机
		if (!m_programExecutor.isRunning())
		{
			planned = tryRandomRestartPass(trySeedListPass, nj, jointLowerRad, jointUpperRad, 400);
		}
	}
	RobotInstructionPlanning::restoreInstructionPose(instruction, backup);
	if (!planned)
	{
		if (planErr)
		{
			*planErr = lastErr.empty() ? "Instruction planning failed" : lastErr;
		}
		return false;
	}
	if (persistTaughtOnSuccess)
	{
		RobotInstructionPlanning::persistTaughtJointsAndToolContext(instruction, resultQ, frames);
		// AUTO 肘/腕时把观测构型写回，避免下次仍从错误构型种子收敛
		if (instruction.hasMotionAxisConfigurationProperty())
		{
			RobotInstruction::MotionAxisConfiguration cfg = instruction.motionAxisConfiguration();
			const QStringList jn = doc->robotRevoluteJointNamesForInstance(instanceIndex);
			std::vector<std::string> jointNames;
			jointNames.reserve(static_cast<size_t>(jn.size()));
			for (const QString& n : jn)
			{
				jointNames.push_back(n.toStdString());
			}
			std::vector<double> q(resultQ.begin(), resultQ.end());
			const RobotInstruction::JointConfigurationClass observed =
				RobotInstruction::classifyJointConfiguration(q, jointNames, &q);
			bool changed = false;
			if (cfg.elbow == RobotInstruction::ElbowPosture::Auto &&
				observed.elbow != RobotInstruction::ElbowPosture::Auto)
			{
				cfg.elbow = observed.elbow;
				changed = true;
			}
			if (cfg.wrist == RobotInstruction::WristPosture::Auto &&
				observed.wrist != RobotInstruction::WristPosture::Auto)
			{
				cfg.wrist = observed.wrist;
				changed = true;
			}
			if (cfg.arm == RobotInstruction::ArmPosture::Auto && observed.arm != RobotInstruction::ArmPosture::Auto)
			{
				cfg.arm = observed.arm;
				changed = true;
			}
			if (changed)
			{
				if (cfg.preset == "AUTO" || cfg.preset.empty())
				{
					cfg.preset = RobotInstruction::suggestMotionAxisPresetToken(observed);
				}
				instruction.setMotionAxisConfiguration(cfg);
			}
		}
	}
	{
		const QString storeFp = computePlanFingerprint(instruction, chainSeedQ, urdfPath, defaultTcpLinkName);
		m_planResultCache.store(insIdQ, storeFp, outPlan);
	}
	return true;
}

namespace
{
struct PlanJobPayload
{
	std::string instructionId;
	RobotInstruction::Type type = RobotInstruction::Type::PTP;
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 eulerDeg{};
	RobotInstruction::Vec3 viaPose{};
	RobotInstruction::Vec3 viaEulerDeg{};
	bool hasVia = false;
	double speed = 0.0;
	double accel = 0.0;
	double blendRadius = 0.0;
	RobotInstruction::MotionAxisConfiguration axisConfig{};
	std::unordered_map<std::string, std::string> extensions;
	QVector<double> seedJointRad;
	QVector<double> programStartQ;
	QString urdfPath;
	QString tcpLinkName;
	std::vector<robot_kinematics::DhRow> dhRows;
	RobotExternal::RobotExternalAxisConfigSet externalAxes;
	RobotInstruction::Controller::WorkpieceIkFrameContext workpieceIkFrame{};
	RobotCoordinate::RobotCoordinateFrameSet frames;
	bool hasFrames = false;
};

void attachWorkpieceIkFrameToPayload(PlanJobPayload& payload, IRobotDocumentHost* doc, const int instIdx)
{
	RobotInstruction::Controller tmp;
	fillWorkpieceIkFrameContext(tmp, doc, instIdx);
	payload.workpieceIkFrame = tmp.workpieceIkFrameContext();
}

PlanJobPayload makePlanJobPayload(const RobotInstruction::Base& ins, const QVector<double>& seedJointRad,
								  const QString& urdfPath, const QString& tcpLinkName,
								  const std::vector<robot_kinematics::DhRow>& dhRows,
								  const RobotExternal::RobotExternalAxisConfigSet* externalAxes = nullptr)
{
	PlanJobPayload payload;
	payload.instructionId = ins.id();
	payload.type = ins.type();
	if (ins.hasPoseProperty())
	{
		payload.pose = ins.pose();
		payload.eulerDeg = ins.eulerDeg();
	}
	if (ins.hasViaPoseProperty())
	{
		payload.hasVia = true;
		payload.viaPose = ins.viaPose();
		payload.viaEulerDeg = ins.viaEulerDeg();
	}
	if (ins.hasSpeedProperty())
	{
		payload.speed = ins.speed();
	}
	if (ins.hasAccelProperty())
	{
		payload.accel = ins.accel();
	}
	if (ins.hasBlendRadiusProperty())
	{
		payload.blendRadius = ins.blendRadius();
	}
	if (ins.hasMotionAxisConfigurationProperty())
	{
		payload.axisConfig = ins.motionAxisConfiguration();
	}
	payload.extensions = ins.extensionProperties();
	payload.seedJointRad = seedJointRad;
	payload.urdfPath = urdfPath;
	payload.tcpLinkName = tcpLinkName;
	payload.dhRows = dhRows;
	if (externalAxes)
	{
		payload.externalAxes = *externalAxes;
	}
	return payload;
}

std::shared_ptr<RobotInstruction::Base> instructionFromPlanJobPayload(const PlanJobPayload& payload)
{
	std::shared_ptr<RobotInstruction::Base> ins;
	if (payload.type == RobotInstruction::Type::LINE)
	{
		auto line = std::make_shared<RobotInstruction::LineInstruction>();
		line->setId(payload.instructionId);
		line->setPose(payload.pose);
		line->setEulerDeg(payload.eulerDeg);
		line->setSpeed(payload.speed);
		line->setAccel(payload.accel);
		line->setBlendRadius(payload.blendRadius);
		line->setMotionAxisConfiguration(payload.axisConfig);
		ins = line;
	}
	else if (payload.type == RobotInstruction::Type::ARC)
	{
		auto arc = std::make_shared<RobotInstruction::ArcInstruction>();
		arc->setId(payload.instructionId);
		arc->setPose(payload.pose);
		arc->setEulerDeg(payload.eulerDeg);
		if (payload.hasVia)
		{
			arc->setViaPose(payload.viaPose);
			arc->setViaEulerDeg(payload.viaEulerDeg);
		}
		arc->setSpeed(payload.speed);
		arc->setAccel(payload.accel);
		arc->setBlendRadius(payload.blendRadius);
		arc->setMotionAxisConfiguration(payload.axisConfig);
		ins = arc;
	}
	else if (payload.type == RobotInstruction::Type::PTP)
	{
		auto ptp = std::make_shared<RobotInstruction::PtpInstruction>();
		ptp->setId(payload.instructionId);
		ptp->setPose(payload.pose);
		ptp->setEulerDeg(payload.eulerDeg);
		ptp->setSpeed(payload.speed);
		ptp->setAccel(payload.accel);
		ptp->setMotionAxisConfiguration(payload.axisConfig);
		ins = ptp;
	}
	if (!ins)
	{
		return nullptr;
	}
	for (const auto& kv : payload.extensions)
	{
		ins->setExtensionProperty(kv.first, kv.second);
	}
	return ins;
}

bool seedsDifferPlan(const QVector<double>& a, const QVector<double>& b)
{
	if (a.size() != b.size())
	{
		return true;
	}
	for (int j = 0; j < a.size(); ++j)
	{
		if (std::abs(a[j] - b[j]) > 1e-9)
		{
			return true;
		}
	}
	return false;
}

/// Worker：对齐 Preview/Run 多种子 IK；LINE 可 lite
bool planMotionLikePreviewWorker(RobotInstruction::Base& ins, RobotInstruction::Controller& workerCtrl,
								 const QVector<double>& chainSeedQ, const QVector<double>& programStartQ,
								 const QString& urdfPath, const QString& tcpLinkName,
								 const RobotCoordinate::RobotCoordinateFrameSet* frames, QVector<double>& outJointQ,
								 RobotInstruction::PlanResult* outPlan,
								 const RobotExternal::RobotExternalAxisConfigSet* externalAxesForPlan = nullptr)
{
	const int nj = chainSeedQ.size() > 0 ? chainSeedQ.size() : programStartQ.size();
	if (nj <= 0)
	{
		return false;
	}

	QVector<QVector<double>> seedOrder;
	auto pushSeed = [&](const QVector<double>& s)
	{
		if (s.size() != nj)
		{
			return;
		}
		for (const QVector<double>& existing : seedOrder)
		{
			if (!seedsDifferPlan(existing, s))
			{
				return;
			}
		}
		seedOrder.push_back(s);
	};
	pushSeed(chainSeedQ);
	pushSeed(programStartQ);

	const RobotInstructionPlanning::MotionPoseBackup backup = RobotInstructionPlanning::backupInstructionPose(ins);
	// 接受写出：非链式种子获胜时笛卡尔轨迹起点锚定在错误位形上；worker 无文档/碰撞上下文，剥离轨迹降级为端点 lerp
	auto acceptPlan = [&](RobotInstruction::PlanResult& plan, const QVector<double>& trySeed,
						  const QVector<double>& acceptQ) -> bool
	{
		if (seedsDifferPlan(trySeed, chainSeedQ) && chainSeedQ.size() == nj && !plan.jointTrajectoryRad.empty())
		{
			plan.jointTrajectoryRad.clear();
			plan.summary += " | non-chain seed anchor, trajectory stripped";
		}
		outJointQ = acceptQ;
		if (outPlan)
		{
			*outPlan = std::move(plan);
		}
		return true;
	};
	// collectNearest：未折圈行程门 + 就近优选（R1 归一化由同步缓存接受点带限位完成）
	auto tryPass = [&](const QVector<QVector<double>>& seeds, const bool useLite, const bool collectNearest,
					   const qint64 budgetMs) -> bool
	{
		QElapsedTimer budget;
		budget.start();
		bool haveBest = false;
		double bestScore = 0.0;
		RobotInstruction::PlanResult bestPlan{};
		QVector<double> bestQ;
		QVector<double> bestSeed;
		for (const QVector<double>& trySeed : seeds)
		{
			if (collectNearest && budgetMs > 0 && budget.elapsed() >= budgetMs)
			{
				break;
			}
			RobotInstructionPlanning::prepareMotionInstructionForPlanning(
				ins, trySeed, nullptr, nullptr, 0, urdfPath, tcpLinkName.toStdString(), frames);
			if (useLite)
			{
				ins.setExtensionProperty("context.playbackPlanLite", "1");
			}
			else
			{
				ins.eraseExtensionProperty("context.playbackPlanLite");
			}
			std::string planErr;
			RobotInstruction::PlanResult plan{};
			const bool ok = workerCtrl.validate(ins, &planErr) && workerCtrl.plan(ins, plan, &planErr) && plan.ok &&
							plan.jointTargetsRad.size() == static_cast<size_t>(nj);
			ins.eraseExtensionProperty("context.playbackPlanLite");
			if (!ok)
			{
				continue;
			}
			QVector<double> resultQ(nj);
			for (int j = 0; j < nj; ++j)
			{
				resultQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
			}
			if (frames)
			{
				const double residualMm =
					targetResidualMmForInstruction(urdfPath, resultQ, *frames, tcpLinkName, ins);
				const double orientDeg =
					targetOrientationResidualDegForInstruction(urdfPath, resultQ, *frames, tcpLinkName, ins);
				if (!isFreshIkSolutionAcceptable(residualMm, orientDeg))
				{
					continue;
				}
			}
			if (collectNearest)
			{
				const double maxAbs = maxFoldedJointAbsDelta(resultQ, chainSeedQ);
				const double sse = unfoldedJointSse(resultQ, chainSeedQ);
				if (maxAbs > kFallbackConsistencyRad)
				{
					continue;
				}
				if (!haveBest || sse < bestScore)
				{
					haveBest = true;
					bestScore = sse;
					bestPlan = plan;
					bestQ = resultQ;
					bestSeed = trySeed;
				}
				if (maxAbs <= kFallbackNearEnoughRad)
				{
					break;
				}
				continue;
			}
			if (acceptPlan(plan, trySeed, resultQ))
			{
				return true;
			}
		}
		if (collectNearest && haveBest)
		{
			return acceptPlan(bestPlan, bestSeed, bestQ);
		}
		return false;
	};
	bool ok = false;
	if (ins.type() != RobotInstruction::Type::PTP)
	{
		ok = tryPass(seedOrder, true, false, 0);
	}
	ok = ok || tryPass(seedOrder, false, false, 0);
	if (!ok)
	{
		// worker 在后台线程：预算放宽保成功率，但必须有界——单难点长期占槽会加剧倍速下的前瞻饥饿
		ok = tryRandomRestartPass(tryPass, nj, QVector<double>(), QVector<double>(), 300);
	}
	RobotInstructionPlanning::restoreInstructionPose(ins, backup);
	return ok;
}

RobotInstruction::PlanResult planLookaheadMotion(const PlanJobPayload& payload)
{
	RobotInstruction::PlanResult plan{};
	if (!RobotInstruction::isMotionWaypointType(payload.type))
	{
		return plan;
	}
	const std::shared_ptr<RobotInstruction::Base> ins = instructionFromPlanJobPayload(payload);
	if (!ins)
	{
		return plan;
	}
	RobotInstruction::Controller workerCtrl;
	workerCtrl.buildDefaultPlanners();
	if (!payload.dhRows.empty())
	{
		workerCtrl.setDhRows(payload.dhRows);
	}
	workerCtrl.setExternalAxes(payload.externalAxes);
	if (payload.workpieceIkFrame.valid)
	{
		workerCtrl.setWorkpieceIkFrameContext(payload.workpieceIkFrame);
	}
	QVector<double> outQ;
	const QVector<double> programStart =
		!payload.programStartQ.isEmpty() ? payload.programStartQ : payload.seedJointRad;
	const RobotCoordinate::RobotCoordinateFrameSet* framesPtr = payload.hasFrames ? &payload.frames : nullptr;
	if (!planMotionLikePreviewWorker(*ins, workerCtrl, payload.seedJointRad, programStart, payload.urdfPath,
									 payload.tcpLinkName, framesPtr, outQ, &plan, &payload.externalAxes))
	{
		plan = {};
		return plan;
	}
	return plan;
}

struct FeasibleAxisJobPayload
{
	PlanJobPayload plan;
	RobotCoordinate::RobotCoordinateFrameSet coordinateFrames;
};

struct FeasibleAxisJobResult
{
	QString instructionId;
	QString fingerprint;
	RobotInstruction::FeasibleMotionAxisConfigurationOptions options;
	QVector<double> seedJointRad;
};

RobotInstruction::FeasibleMotionAxisConfigurationOptions runFeasibleAxisProbeJob(const FeasibleAxisJobPayload& payload)
{
	RobotInstruction::FeasibleMotionAxisConfigurationOptions out;
	const std::shared_ptr<RobotInstruction::Base> ins = instructionFromPlanJobPayload(payload.plan);
	if (!ins)
	{
		return out;
	}
	RobotInstruction::Controller workerCtrl;
	workerCtrl.buildDefaultPlanners();
	if (!payload.plan.dhRows.empty())
	{
		workerCtrl.setDhRows(payload.plan.dhRows);
	}
	workerCtrl.setExternalAxes(payload.plan.externalAxes);
	if (payload.plan.workpieceIkFrame.valid)
	{
		workerCtrl.setWorkpieceIkFrameContext(payload.plan.workpieceIkFrame);
	}
	const RobotInstructionPlanning::MotionPoseBackup backup = RobotInstructionPlanning::backupInstructionPose(*ins);
	RobotInstructionPlanning::prepareMotionInstructionForPlanning(
		*ins, payload.plan.seedJointRad, nullptr, nullptr, 0, payload.plan.urdfPath,
		payload.plan.tcpLinkName.toStdString(), &payload.coordinateFrames);
	out = workerCtrl.queryFeasibleMotionAxisConfigurationOptions(*ins);
	RobotInstructionPlanning::restoreInstructionPose(*ins, backup);
	return out;
}

} // namespace

void RobotSimulationController::scheduleDeferredFeasibleAxisProbe(
	const std::shared_ptr<RobotInstruction::Base>& instruction, const FeasibleAxisProbePurpose purpose)
{
	if (!instruction || !m_host || !RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		return;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return;
	}
	QVector<double> rollingQ;
	int targetMotionIndex = -1;
	if (!buildChainSeedJointRadForInstruction(instruction, rollingQ, &targetMotionIndex))
	{
		return;
	}
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath, m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	QString fingerprint = QString::fromStdString(instruction->id());
	if (instruction->hasPoseProperty())
	{
		const RobotInstruction::Vec3 p = instruction->pose();
		const RobotInstruction::Vec3 e = instruction->eulerDeg();
		fingerprint += QStringLiteral("|%1,%2,%3|%4,%5,%6")
						   .arg(p.x, 0, 'g', 8)
						   .arg(p.y, 0, 'g', 8)
						   .arg(p.z, 0, 'g', 8)
						   .arg(e.x, 0, 'g', 8)
						   .arg(e.y, 0, 'g', 8)
						   .arg(e.z, 0, 'g', 8);
	}
	fingerprint += QStringLiteral("|mi=%1").arg(targetMotionIndex);
	for (int j = 0; j < rollingQ.size(); ++j)
	{
		fingerprint += QLatin1Char(',') + QString::number(rollingQ[j], 'g', 8);
	}
	osg::Matrixd fpBaseWorld;
	fpBaseWorld.makeIdentity();
	if (RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, m_host->osgView(), instIdx, fpBaseWorld, &rollingQ))
	{
		fingerprint += QStringLiteral("|bw=%1,%2,%3")
						   .arg(fpBaseWorld(3, 0), 0, 'g', 8)
						   .arg(fpBaseWorld(3, 1), 0, 'g', 8)
						   .arg(fpBaseWorld(3, 2), 0, 'g', 8);
	}
	const QString instructionId = QString::fromStdString(instruction->id());
	if (const PlanResultCache::FeasibleAxisEntry* hit =
			m_planResultCache.fetchFeasibleAxis(instructionId, fingerprint))
	{
		m_lastFeasibleAxisInstructionId = instructionId;
		m_lastFeasibleAxisFingerprint = fingerprint;
		m_lastFeasibleAxisOptions = hit->options;
		m_host->applySuggestedAxisPresetFromSeedIfNeeded(instruction, hit->seedJointRad, hit->options);
		if (purpose == FeasibleAxisProbePurpose::SelectionAutoSeed)
		{
			instruction->setExtensionProperty("context.axisConfigSeeded", "1");
		}
		m_host->refreshInstructionPropertyPanel(instruction, false);
		return;
	}

	const std::vector<robot_kinematics::DhRow> dhRows; // 有 URDF 不建 DH，worker 走数值 IK

	FeasibleAxisJobPayload payload;
	const RobotExternal::RobotExternalAxisConfigSet& extAxes = doc->robotExternalAxesForInstance(instIdx);
	payload.plan = makePlanJobPayload(*instruction, rollingQ, urdfPath, defaultTcpLinkName, dhRows, &extAxes);
	attachWorkpieceIkFrameToPayload(payload.plan, doc, instIdx);
	payload.coordinateFrames = doc->robotCoordinateFramesForInstance(instIdx);

	const quint64 token = m_feasibleAxisJobToken;
	const auto jobResult = std::make_shared<FeasibleAxisJobResult>();
	jobResult->instructionId = QString::fromStdString(instruction->id());
	jobResult->fingerprint = fingerprint;
	jobResult->seedJointRad = rollingQ;
	QPointer<RobotSimulationController> guard(this);
	m_host->enqueueBackgroundJob(
		QStringLiteral("Feasible axis IK"),
		[jobResult, payload]() { jobResult->options = runFeasibleAxisProbeJob(payload); },
		[this, guard, token, jobResult, purpose](const bool threw, const QString&)
		{
			if (!guard || threw || token != m_feasibleAxisJobToken || !m_host)
			{
				return;
			}
			m_planResultCache.storeFeasibleAxis(jobResult->instructionId, jobResult->fingerprint, jobResult->options,
											   jobResult->seedJointRad);
			m_lastFeasibleAxisInstructionId = jobResult->instructionId;
			m_lastFeasibleAxisFingerprint = jobResult->fingerprint;
			m_lastFeasibleAxisOptions = jobResult->options;
			const std::shared_ptr<RobotInstruction::Base> active = m_host->activeInstructionForProperty();
			if (!active || QString::fromStdString(active->id()) != jobResult->instructionId)
			{
				return;
			}
			m_host->applySuggestedAxisPresetFromSeedIfNeeded(active, jobResult->seedJointRad, jobResult->options);
			if (purpose == FeasibleAxisProbePurpose::SelectionAutoSeed)
			{
				active->setExtensionProperty("context.axisConfigSeeded", "1");
			}
			m_host->refreshInstructionPropertyPanel(active, false);
		});
}

namespace
{
struct ReachabilityJobStep
{
	QString instructionId;
	PlanJobPayload planPayload;
};

struct ReachabilityJobInput
{
	QVector<ReachabilityJobStep> steps;
	QVector<double> programStartQ;
	RobotCoordinate::RobotCoordinateFrameSet frames;
	int batchStart = 0;
	bool moreRemaining = false;
};

struct ReachabilityJobOutput
{
	QHash<QString, bool> reachability;
	QVector<double> rollingEndQ;
};

ReachabilityJobOutput runReachabilityJob(const ReachabilityJobInput& input)
{
	ReachabilityJobOutput out;
	if (input.steps.isEmpty() || input.programStartQ.isEmpty())
	{
		return out;
	}
	QVector<double> rollingQ = input.programStartQ;
	const int nj = rollingQ.size();
	RobotInstruction::Controller workerCtrl;
	workerCtrl.buildDefaultPlanners();
	if (!input.steps.front().planPayload.dhRows.empty())
	{
		workerCtrl.setDhRows(input.steps.front().planPayload.dhRows);
	}
	workerCtrl.setExternalAxes(input.steps.front().planPayload.externalAxes);
	for (const ReachabilityJobStep& step : input.steps)
	{
		if (step.planPayload.workpieceIkFrame.valid)
		{
			workerCtrl.setWorkpieceIkFrameContext(step.planPayload.workpieceIkFrame);
		}
		else
		{
			workerCtrl.clearWorkpieceIkFrameContext();
		}
		const std::shared_ptr<RobotInstruction::Base> ins = instructionFromPlanJobPayload(step.planPayload);
		if (!ins)
		{
			out.reachability.insert(step.instructionId, false);
			continue;
		}
		QVector<double> resultQ;
		const bool ok = planMotionLikePreviewWorker(*ins, workerCtrl, rollingQ, input.programStartQ,
													step.planPayload.urdfPath, step.planPayload.tcpLinkName,
													&input.frames, resultQ, nullptr);
		out.reachability.insert(step.instructionId, ok);
		if (ok && resultQ.size() == nj)
		{
			rollingQ = resultQ;
		}
	}
	out.rollingEndQ = rollingQ;
	return out;
}
} // namespace

QString RobotSimulationController::computePlanFingerprint(const RobotInstruction::Base& instruction,
														  const QVector<double>& seedJointRad, const QString& urdfPath,
														  const QString& tcpLinkName) const
{
	QString fp;
	fp.reserve(256);
	fp += QString::fromStdString(instruction.id());
	if (instruction.hasPoseProperty())
	{
		const RobotInstruction::Vec3 p = instruction.pose();
		const RobotInstruction::Vec3 e = instruction.eulerDeg();
		fp += QStringLiteral("|p:");
		fp += QString::number(p.x, 'f', 3);
		fp += QLatin1Char(',');
		fp += QString::number(p.y, 'f', 3);
		fp += QLatin1Char(',');
		fp += QString::number(p.z, 'f', 3);
		fp += QStringLiteral("|e:");
		fp += QString::number(e.x, 'f', 2);
		fp += QLatin1Char(',');
		fp += QString::number(e.y, 'f', 2);
		fp += QLatin1Char(',');
		fp += QString::number(e.z, 'f', 2);
	}
	if (instruction.hasViaPoseProperty())
	{
		const RobotInstruction::Vec3 vp = instruction.viaPose();
		const RobotInstruction::Vec3 ve = instruction.viaEulerDeg();
		fp += QStringLiteral("|vp:");
		fp += QString::number(vp.x, 'f', 3);
		fp += QLatin1Char(',');
		fp += QString::number(vp.y, 'f', 3);
		fp += QLatin1Char(',');
		fp += QString::number(vp.z, 'f', 3);
		fp += QStringLiteral("|ve:");
		fp += QString::number(ve.x, 'f', 2);
		fp += QLatin1Char(',');
		fp += QString::number(ve.y, 'f', 2);
		fp += QLatin1Char(',');
		fp += QString::number(ve.z, 'f', 2);
	}
	const auto& extVia = instruction.extensionProperties();
	const auto itViaQ = extVia.find(RobotInstruction::kExtContextViaTransformQuatCsv);
	const auto itViaT = extVia.find(RobotInstruction::kExtContextViaTransformTransMmCsv);
	if (itViaQ != extVia.end() && !itViaQ->second.empty())
	{
		fp += QStringLiteral("|vq:");
		fp += QString::fromStdString(itViaQ->second);
	}
	if (itViaT != extVia.end() && !itViaT->second.empty())
	{
		fp += QStringLiteral("|vt:");
		fp += QString::fromStdString(itViaT->second);
	}
	if (instruction.hasSpeedProperty())
	{
		fp += QStringLiteral("|s:");
		fp += QString::number(instruction.speed(), 'f', 1);
	}
	if (instruction.hasAccelProperty())
	{
		fp += QStringLiteral("|a:");
		fp += QString::number(instruction.accel(), 'f', 1);
	}
	const auto& ext = instruction.extensionProperties();
	const auto itAx = ext.find("motion.axisConfig.preset");
	if (itAx != ext.end())
	{
		fp += QStringLiteral("|ax:");
		fp += QString::fromStdString(itAx->second);
	}
	const auto itToolMotion = ext.find(RobotCoordinate::kExtMotionToolFrameId);
	if (itToolMotion != ext.end() && !itToolMotion->second.empty())
	{
		fp += QStringLiteral("|tf:");
		fp += QString::fromStdString(itToolMotion->second);
	}
	const auto itToolMat = ext.find(RobotCoordinate::kExtContextToolFrameMat4);
	if (itToolMat != ext.end() && !itToolMat->second.empty())
	{
		fp += QStringLiteral("|tm:");
		fp += QString::fromStdString(itToolMat->second);
	}
	const auto itExtQ = ext.find(RobotExternal::kExtContextExternalAxisQMm);
	if (itExtQ != ext.end() && !itExtQ->second.empty())
	{
		fp += QStringLiteral("|eq:");
		fp += QString::fromStdString(itExtQ->second);
	}
	const auto itExtCsv = ext.find(RobotExternal::kExtContextExternalAxisQCsv);
	if (itExtCsv != ext.end() && !itExtCsv->second.empty())
	{
		fp += QStringLiteral("|eqs:");
		fp += QString::fromStdString(itExtCsv->second);
	}
	if (m_host && m_host->document() && m_host->simulationCommandPage())
	{
		const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
		if (instIdx >= 0)
		{
			const RobotExternal::RobotExternalAxisConfigSet& ea =
				m_host->document()->robotExternalAxesForInstance(instIdx);
			if (RobotExternal::hasEnabledExternalAxes(ea))
			{
				fp += QStringLiteral("|eacfg:");
				for (const RobotExternal::RobotExternalAxisConfig& a : ea.axes)
				{
					if (!a.enabled)
					{
						continue;
					}
					fp += QString::number(static_cast<int>(a.kind));
					fp += QLatin1Char(',');
					fp += QString::number(a.lower, 'f', 2);
					fp += QLatin1Char(',');
					fp += QString::number(a.upper, 'f', 2);
					fp += QLatin1Char(',');
					fp += QString::number(a.axis[0], 'f', 3);
					fp += QLatin1Char(',');
					fp += QString::number(a.axis[1], 'f', 3);
					fp += QLatin1Char(',');
					fp += QString::number(a.axis[2], 'f', 3);
					fp += QLatin1Char(';');
				}
			}
		}
	}
	fp += QStringLiteral("|q:");
	for (int i = 0; i < seedJointRad.size(); ++i)
	{
		if (i > 0)
		{
			fp += QLatin1Char(',');
		}
		fp += QString::number(seedJointRad[i], 'f', 4);
	}
	fp += QStringLiteral("|u:");
	fp += urdfPath;
	fp += QStringLiteral("|t:");
	fp += tcpLinkName;
	return QString::number(qHash(fp));
}

bool RobotSimulationController::trySeedJointRadForMotionIndex(const size_t targetMotionIndex,
															  const QVector<double>& programStartQ,
															  const QString& urdfPath, const QString& tcpLinkName,
															  const int jointCount, QVector<double>& outSeedQ) const
{
	if (jointCount <= 0)
	{
		return false;
	}
	if (targetMotionIndex == 0)
	{
		outSeedQ = !m_playbackProgramStartQ.isEmpty() ? m_playbackProgramStartQ : programStartQ;
		return outSeedQ.size() == jointCount;
	}
	if (targetMotionIndex > m_currentRunMotions.size())
	{
		return false;
	}

	auto acceptJoints = [&](const RobotInstruction::PlanResult& plan) -> bool
	{
		if (!plan.ok || plan.jointTargetsRad.size() != static_cast<size_t>(jointCount))
		{
			return false;
		}
		outSeedQ.resize(jointCount);
		for (int j = 0; j < jointCount; ++j)
		{
			outSeedQ[j] = plan.jointTargetsRad[static_cast<size_t>(j)];
		}
		return true;
	};
	// 前瞻只写 Cache 时执行器仍是 lazyPending：用缓存终点接着滚种子，才能在播放前缀时把后面段排上
	auto jointsFromMotion = [&](const RobotInstruction::Base* motion) -> bool
	{
		if (!motion)
		{
			return false;
		}
		if (const RobotInstruction::PlanResult* execPlan = m_programExecutor.motionPlanResult(motion))
		{
			if (acceptJoints(*execPlan))
			{
				return true;
			}
		}
		const QString fp = computePlanFingerprint(*motion, outSeedQ, urdfPath, tcpLinkName);
		if (const RobotInstruction::PlanResult* cached =
				m_planResultCache.fetch(QString::fromStdString(motion->id()), fp))
		{
			return acceptJoints(*cached);
		}
		return false;
	};

	if (m_playbackRollingSeedQ.size() != jointCount)
	{
		outSeedQ = programStartQ;
		if (outSeedQ.size() != jointCount)
		{
			return false;
		}
		for (size_t mi = 0; mi < targetMotionIndex; ++mi)
		{
			if (!jointsFromMotion(m_currentRunMotions[mi]))
			{
				return false;
			}
		}
		return true;
	}
	if (targetMotionIndex < m_playbackMotionIndex)
	{
		return false;
	}
	outSeedQ = m_playbackRollingSeedQ;
	for (size_t mi = m_playbackMotionIndex; mi < targetMotionIndex; ++mi)
	{
		if (!jointsFromMotion(m_currentRunMotions[mi]))
		{
			return false;
		}
	}
	return true;
}

void RobotSimulationController::stripPlanTrajectory(RobotInstruction::PlanResult& plan)
{
	// 播放依赖 jointTrajectoryRad 插帧；保留轨迹，仅作 API 兼容占位
	(void)plan;
}

void RobotSimulationController::commitPlaybackPlan(const RobotInstruction::Base* motion, const size_t motionIndex,
												   RobotInstruction::PlanResult plan)
{
	if (!motion)
	{
		return;
	}
	if (plan.hasExternalAxisQ)
	{
		IRobotDocumentHost* docForExt = m_host ? m_host->document() : nullptr;
		const int instForExt =
			(m_host && m_host->simulationCommandPage() && m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0)
				? m_host->simulationCommandPage()->currentRobotInstanceIndex()
				: 0;
		RobotExternal::RobotExternalAxisConfigSet extSet;
		if (docForExt)
		{
			extSet = docForExt->robotExternalAxesForInstance(instForExt);
		}
		std::vector<double> qeStart = toStdVector(m_playbackSegmentExternalAxisStart);
		if (motionIndex > 0 && motionIndex <= m_currentRunMotions.size())
		{
			const RobotInstruction::Base* prev = m_currentRunMotions[motionIndex - 1];
			if (const RobotInstruction::PlanResult* prevPlan = m_programExecutor.motionPlanResult(prev))
			{
				if (prevPlan->ok && prevPlan->hasExternalAxisQ)
				{
					if (!prevPlan->externalAxisQs.empty())
					{
						qeStart = padExternalAxisQToConfig(extSet, prevPlan->externalAxisQs);
					}
					else
					{
						qeStart = expandScalarExternalAxisQ(extSet, prevPlan->externalAxisQ);
					}
				}
			}
		}
		std::vector<double> qeTarget =
			!plan.externalAxisQs.empty() ? padExternalAxisQToConfig(extSet, plan.externalAxisQs)
										 : expandScalarExternalAxisQ(extSet, plan.externalAxisQ);
		plan.durationSec = std::max(plan.durationSec, externalAxisTravelDurationSec(extSet, qeStart, qeTarget));
	}
	const QString insIdQ = QString::fromStdString(motion->id());
	QVector<double> seedForFp = m_playbackRollingSeedQ;
	if (seedForFp.isEmpty() && !plan.jointTargetsRad.empty())
	{
		seedForFp = QVector<double>(plan.jointTargetsRad.begin(), plan.jointTargetsRad.end());
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	QString urdf;
	QString tcp;
	if (doc && m_host->simulationCommandPage())
	{
		const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
								? m_host->simulationCommandPage()->currentRobotInstanceIndex()
								: 0;
		urdf = doc->robotUrdfAbsolutePathForInstance(instIdx);
		tcp = RobotSimulationMath::defaultTcpLinkNameForUrdf(urdf, m_host->simulationCommandPage()->selectedTcpLink());
	}
	const QString fp = computePlanFingerprint(*motion, seedForFp, urdf, tcp);
	m_planResultCache.store(insIdQ, fp, plan, motionIndex);
	m_lookaheadFailedFingerprints.remove(fp);
	(void)m_programExecutor.updateMotionPlanResult(motion, plan);
}

bool RobotSimulationController::syncPlanMotionAtIndex(const size_t motionIndex, const bool allowHeavyPlan)
{
	if (!m_host || motionIndex >= m_currentRunMotions.size())
	{
		return false;
	}
	const RobotInstruction::Base* motionPtr = m_currentRunMotions[motionIndex];
	if (!motionPtr)
	{
		return false;
	}
	const RobotInstruction::PlanResult* existing = m_programExecutor.motionPlanResult(motionPtr);
	if (existing && existing->ok)
	{
		return true;
	}
	if (existing && !existing->ok && existing->plannerName != "lazyPending")
	{
		return false;
	}

	IRobotDocumentHost* doc = m_host->document();
	if (!doc || !m_host->simulationCommandPage() || !doc->hasRobotSimulationContext())
	{
		return false;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (urdfPath.isEmpty() || nj <= 0)
	{
		return false;
	}
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath, m_host->simulationCommandPage()->selectedTcpLink());
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const RobotCoordinate::RobotCoordinateFrameSet& framesForRun = doc->robotCoordinateFramesForInstance(instIdx);
	const QVector<double> programStartQ =
		!m_playbackProgramStartQ.isEmpty() ? m_playbackProgramStartQ : m_playbackRollingSeedQ;

	QVector<double> rollingQ;
	if (!trySeedJointRadForMotionIndex(motionIndex, programStartQ, urdfPath, defaultTcpLinkName, nj, rollingQ))
	{
		RobotInstruction::PlanResult failed{};
		failed.ok = false;
		failed.plannerName = "failed";
		failed.summary = "Lazy plan seed unavailable";
		(void)m_programExecutor.updateMotionPlanResult(motionPtr, failed);
		if (m_host->runInfoPage())
		{
			QString detail =
				m_host->i18n(QStringLiteral("Lazy plan seed unavailable for motion %1.")
								 .arg(static_cast<int>(motionIndex) + 1),
							 QStringLiteral("运动 %1 的懒加载规划种子不可用。").arg(static_cast<int>(motionIndex) + 1));
			// 常见连锁：前序段 IK/碰撞失败后本段无滚动种子
			if (motionIndex > 0 && motionIndex <= m_currentRunMotions.size())
			{
				const RobotInstruction::Base* prev = m_currentRunMotions[motionIndex - 1];
				if (const RobotInstruction::PlanResult* prevPlan = m_programExecutor.motionPlanResult(prev))
				{
					if (!prevPlan->ok)
					{
						const QString prevReason = QString::fromStdString(
							prevPlan->summary.empty() ? prevPlan->plannerName : prevPlan->summary);
						detail += m_host->i18n(QStringLiteral(" Prior motion %1 failed: %2")
												   .arg(static_cast<int>(motionIndex))
												   .arg(prevReason),
											   QStringLiteral(" 前序运动 %1 已失败：%2")
												   .arg(static_cast<int>(motionIndex))
												   .arg(prevReason));
					}
				}
			}
			m_host->appendRunWarning(detail);
		}
		return false;
	}

	RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motionPtr);
	const QString insIdQ = QString::fromStdString(ins->id());
	const QString fp = computePlanFingerprint(*ins, rollingQ, urdfPath, defaultTcpLinkName);

	if (const RobotInstruction::PlanResult* cached = m_planResultCache.fetch(insIdQ, fp))
	{
		const bool arcNeedsTraj =
			ins->type() == RobotInstruction::Type::ARC && cached->jointTrajectoryRad.size() < 2U;
		if (cached->ok && cached->jointTargetsRad.size() == static_cast<size_t>(nj) && !arcNeedsTraj)
		{
			QVector<double> cachedQ(nj);
			for (int j = 0; j < nj; ++j)
			{
				cachedQ[j] = cached->jointTargetsRad[static_cast<size_t>(j)];
			}
			const double residualMm =
				targetResidualMmForInstruction(urdfPath, cachedQ, framesForRun, defaultTcpLinkName, *ins);
			const double orientDeg = targetOrientationResidualDegForInstruction(urdfPath, cachedQ, framesForRun,
																				defaultTcpLinkName, *ins);
		if (isTaughtOrCacheReuseAcceptable(residualMm, orientDeg))
		{
			RobotInstruction::PlanResult plan = *cached;
			// R1：参照滚动链种子；归一失败则丢弃缓存重规划。示教 CSV 路径刻意不归一
			std::vector<double> refQ(rollingQ.begin(), rollingQ.end());
			QVector<double> limLo, limHi;
			doc->robotJointLimitsForInstance(instIdx, limLo, limHi);
			const std::vector<double> targetsBeforeNorm = plan.jointTargetsRad;
			const bool wrapOk = normalizeJointRevolutionsToReference(plan.jointTargetsRad, refQ, limLo, limHi);
			bool cacheOk = wrapOk;
			if (cacheOk)
			{
				alignTrajectoryAfterTargetNormalize(plan, targetsBeforeNorm);
				// 播放中缓存入库时已验碰，勿每 tick 重复 24 点 apply+重建
				const RobotCollision::Settings& colGate = doc->robotCollisionSettings();
				if (colGate.enabled && m_collisionWorld && !m_programExecutor.isRunning())
				{
					std::vector<std::vector<double>> seg = plan.jointTrajectoryRad;
					if (seg.empty())
					{
						seg.push_back(plan.jointTargetsRad);
					}
					std::string colErr;
					if (!BackendCollisionSync::validateJointTrajectory(*m_collisionWorld, doc, doc->backend(),
																	   instIdx, rollingQ, seg, colGate, &colErr,
																	   m_host->osgView(), /*rebuildWorldFirst=*/false))
					{
						cacheOk = false;
					}
				}
			}
			if (cacheOk)
			{
				(void)m_programExecutor.updateMotionPlanResult(motionPtr, plan);
				return true;
			}
			m_planResultCache.invalidateByInstruction(insIdQ);
			m_lookaheadFailedFingerprints.insert(fp);
			if (!wrapOk)
			{
				failPlaybackMotionPlan(motionPtr,
									   describeJointNormalizeFailure(targetsBeforeNorm, refQ, limLo, limHi));
				return false;
			}
		}
		}
	}

	if (!allowHeavyPlan)
	{
		return false; // 播放中禁 UI 现算
	}

	ins->eraseExtensionProperty("context.playbackPlanLite");
	std::string planErr;
	RobotInstruction::PlanResult plan{};
	QElapsedTimer syncPlanTimer;
	syncPlanTimer.start();
	const bool okPlan = planMotionConsistentWithPreview(*ins, rollingQ, programStartQ, instIdx, urdfPath,
														defaultTcpLinkName, robotBackendId, framesForRun, plan,
														&planErr, true);
	if (m_programExecutor.isRunning() && syncPlanTimer.elapsed() > 500 && RunLogger::isDiagnosticsEnabled())
	{
		RunLogger::info("Playback sync plan motion " + std::to_string(motionIndex + 1) + " took " +
						std::to_string(syncPlanTimer.elapsed()) + " ms");
	}
	if (!okPlan)
	{
		RobotInstruction::PlanResult failed{};
		failed.ok = false;
		failed.plannerName = "failed";
		failed.summary = planErr.empty() ? "Instruction planning failed" : planErr;
		(void)m_programExecutor.updateMotionPlanResult(motionPtr, failed);
		if (m_host->runInfoPage())
		{
			m_host->appendRunWarning(QString::fromStdString(failed.summary));
		}
		return false;
	}
	if (plan.durationSec > 1e-6)
	{
		ins->setExtensionProperty("motion.durationSec", QString::number(plan.durationSec, 'f', 3).toStdString());
	}
	commitPlaybackPlan(motionPtr, motionIndex, std::move(plan));
	return true;
}

void RobotSimulationController::ensurePlaybackPlansReady()
{
	if (!m_programExecutor.isRunning() || m_currentRunMotions.empty())
	{
		return;
	}

	size_t currentMi = m_playbackMotionIndex;
	if (const RobotInstruction::Base* active = m_programExecutor.activeMotion())
	{
		for (size_t i = 0; i < m_currentRunMotions.size(); ++i)
		{
			if (m_currentRunMotions[i] == active)
			{
				currentMi = i;
				break;
			}
		}
	}

	if (currentMi != m_playbackMotionIndex)
	{
		if (currentMi > m_playbackMotionIndex && currentMi >= 1)
		{
			const RobotInstruction::Base* prev = m_currentRunMotions[currentMi - 1];
			if (const RobotInstruction::PlanResult* prevPlan = m_programExecutor.motionPlanResult(prev))
			{
				if (prevPlan->ok && !prevPlan->jointTargetsRad.empty())
				{
					m_playbackRollingSeedQ =
						QVector<double>(prevPlan->jointTargetsRad.begin(), prevPlan->jointTargetsRad.end());
				}
			}
		}
		m_playbackMotionIndex = currentMi;
		m_planResultCache.evictFarBehind(currentMi, 64);
	}

	const size_t last = m_currentRunMotions.size() - 1;
	const double rate = m_programExecutor.playbackRate();
	const size_t prefetch =
		std::max<size_t>(2, static_cast<size_t>(std::lround(2.0 * rate)));
	const size_t needThrough = std::min(last, currentMi + prefetch);
	constexpr qint64 kHoldTimeoutMs = 1500;
	for (size_t mi = currentMi; mi <= needThrough; ++mi)
	{
		const RobotInstruction::Base* motion = m_currentRunMotions[mi];
		if (!motion)
		{
			continue;
		}
		const RobotInstruction::PlanResult* plan = m_programExecutor.motionPlanResult(motion);
		if (!plan || plan->plannerName != "lazyPending")
		{
			if (mi == currentMi && m_playbackHoldTimer.isValid())
			{
				m_playbackHoldTimer.invalidate();
			}
			continue;
		}
		if (syncPlanMotionAtIndex(mi, false))
		{
			if (mi == currentMi && m_playbackHoldTimer.isValid())
			{
				m_playbackHoldTimer.invalidate();
			}
			continue;
		}
		if (mi != currentMi)
		{
			break;
		}
		if (!m_playbackHoldTimer.isValid())
		{
			m_playbackHoldTimer.start();
		}
		if (m_playbackHoldTimer.elapsed() >= kHoldTimeoutMs)
		{
			(void)syncPlanMotionAtIndex(mi, true);
			m_playbackHoldTimer.invalidate();
		}
		break;
	}
}

bool RobotSimulationController::tryCommitLookaheadPlan(const RobotInstruction::Base* ins, const size_t motionIndex,
													   const QString& insIdQ, const QString& fingerprint,
													   const QVector<double>& seedJointRad,
													   RobotInstruction::PlanResult plan)
{
	if (!ins || fingerprint.isEmpty())
	{
		return false;
	}
	if (m_lookaheadFailedFingerprints.contains(fingerprint))
	{
		const RobotInstruction::PlanResult* existing = m_programExecutor.motionPlanResult(ins);
		if (existing && existing->plannerName == "lazyPending")
		{
			failPlaybackMotionPlan(ins, "关节转数折回失败");
		}
		return false;
	}
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc)
	{
		return false;
	}
	const int instIdx = m_host->simulationCommandPage() && m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	QVector<double> limLo, limHi;
	doc->robotJointLimitsForInstance(instIdx, limLo, limHi);
	if (!prepareWorkerPlanForCache(plan, seedJointRad, limLo, limHi))
	{
		m_lookaheadFailedFingerprints.insert(fingerprint);
		failPlaybackMotionPlan(ins, plan.summary);
		return false;
	}
	m_planResultCache.store(insIdQ, fingerprint, plan, motionIndex);
	return m_programExecutor.updateMotionPlanResult(ins, plan);
}

void RobotSimulationController::failPlaybackMotionPlan(const RobotInstruction::Base* ins, const std::string& summary)
{
	if (!ins)
	{
		return;
	}
	RobotInstruction::PlanResult failed{};
	failed.ok = false;
	failed.plannerName = "failed";
	failed.summary = summary.empty() ? "关节转数折回失败" : summary;
	(void)m_programExecutor.updateMotionPlanResult(ins, failed);
	RunLogger::warn(failed.summary);
}

void RobotSimulationController::tickLookaheadPlanning()
{
	if (!m_lookaheadConfig.enabled || !m_programExecutor.isRunning() || !m_host)
	{
		return;
	}
	// 倍率抬高段消耗速率，前瞻并发随倍率放大；封顶 8 避免后台线程与渲染线程争抢
	const int effectiveMaxJobs =
		std::clamp(static_cast<int>(std::ceil(m_lookaheadConfig.maxConcurrentJobs * m_programExecutor.playbackRate())),
				   m_lookaheadConfig.maxConcurrentJobs, 8);
	if (m_lookaheadPendingJobs >= effectiveMaxJobs)
	{
		return;
	}
	if (m_currentRunMotions.empty())
	{
		return;
	}

	size_t currentMi = m_playbackMotionIndex;
	if (const RobotInstruction::Base* active = m_programExecutor.activeMotion())
	{
		for (size_t i = 0; i < m_currentRunMotions.size(); ++i)
		{
			if (m_currentRunMotions[i] == active)
			{
				currentMi = i;
				break;
			}
		}
		m_playbackMotionIndex = currentMi;
	}

	IRobotDocumentHost* doc = m_host->document();
	if (!doc || !m_host->simulationCommandPage())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	const QString tcpLinkName =
		RobotSimulationMath::defaultTcpLinkNameForUrdf(urdfPath, m_host->simulationCommandPage()->selectedTcpLink());
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return;
	}

	const QVector<double> programStartQ =
		!m_playbackProgramStartQ.isEmpty() ? m_playbackProgramStartQ : m_playbackRollingSeedQ;
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	const std::vector<robot_kinematics::DhRow> dhRows; // 有 URDF 不建 DH，worker 走数值 IK

	const int maxAdvance =
		std::clamp(static_cast<int>(std::lround(m_lookaheadConfig.maxAdvanceBlocks * m_programExecutor.playbackRate())),
				   m_lookaheadConfig.maxAdvanceBlocks, 64);
	int jobsStarted = 0;
	for (int ahead = 0; ahead <= maxAdvance; ++ahead)
	{
		if (m_lookaheadPendingJobs >= effectiveMaxJobs)
		{
			break;
		}
		const size_t targetMi = currentMi + static_cast<size_t>(ahead);
		if (targetMi >= m_currentRunMotions.size())
		{
			break;
		}
		if (m_lookaheadInFlight.contains(targetMi))
		{
			continue;
		}
		const RobotInstruction::Base* ins = m_currentRunMotions[targetMi];
		if (!ins || !RobotInstruction::isMotionWaypointType(ins->type()))
		{
			continue;
		}
		if (const RobotInstruction::PlanResult* execPlan = m_programExecutor.motionPlanResult(ins))
		{
			if (execPlan->ok || execPlan->plannerName != "lazyPending")
			{
				continue;
			}
		}

		QVector<double> seedQ;
		if (!trySeedJointRadForMotionIndex(targetMi, programStartQ, urdfPath, tcpLinkName, nj, seedQ))
		{
			continue;
		}

		const QString insIdQ = QString::fromStdString(ins->id());
		const QString fp = computePlanFingerprint(*ins, seedQ, urdfPath, tcpLinkName);
		if (m_lookaheadFailedFingerprints.contains(fp))
		{
			const RobotInstruction::PlanResult* execPlan = m_programExecutor.motionPlanResult(ins);
			if (execPlan && execPlan->plannerName == "lazyPending")
			{
				failPlaybackMotionPlan(ins, "关节转数折回失败");
			}
			continue;
		}
		if (const RobotInstruction::PlanResult* cached = m_planResultCache.fetch(insIdQ, fp))
		{
			if (cached->ok)
			{
				(void)tryCommitLookaheadPlan(ins, targetMi, insIdQ, fp, seedQ, *cached);
			}
			continue;
		}

		PlanJobPayload payload = makePlanJobPayload(*ins, seedQ, urdfPath, tcpLinkName, dhRows,
													&doc->robotExternalAxesForInstance(instIdx));
		attachWorkpieceIkFrameToPayload(payload, doc, instIdx);
		payload.programStartQ = programStartQ;
		payload.frames = frames;
		payload.hasFrames = true;
		struct LookaheadJobResult
		{
			QString insId;
			QString fingerprint;
			size_t motionIndex = 0;
			int instanceIndex = 0;
			QVector<double> seedJointRad;
			RobotInstruction::PlanResult plan;
		};
		const auto jobResult = std::make_shared<LookaheadJobResult>();
		jobResult->insId = insIdQ;
		jobResult->fingerprint = fp;
		jobResult->motionIndex = targetMi;
		jobResult->instanceIndex = instIdx;
		jobResult->seedJointRad = seedQ;

		++m_lookaheadPendingJobs;
		++jobsStarted;
		m_lookaheadInFlight.insert(targetMi);
		m_host->enqueueBackgroundJob(
			QStringLiteral("Lookahead: %1").arg(insIdQ),
			[jobResult, payload]() { jobResult->plan = planLookaheadMotion(payload); },
			[this, jobResult](const bool threw, const QString&)
			{
				--m_lookaheadPendingJobs;
				m_lookaheadInFlight.remove(jobResult->motionIndex);
				if (threw || !jobResult->plan.ok)
				{
					return;
				}
				if (jobResult->motionIndex >= m_currentRunMotions.size())
				{
					return;
				}
				(void)tryCommitLookaheadPlan(m_currentRunMotions[jobResult->motionIndex], jobResult->motionIndex,
											 jobResult->insId, jobResult->fingerprint, jobResult->seedJointRad,
											 std::move(jobResult->plan));
			});
		if (jobsStarted >= effectiveMaxJobs)
		{
			break;
		}
	}
}

void RobotSimulationController::scheduleAsyncMotionReachabilityRefresh()
{
	++m_reachabilityJobToken;
	m_reachabilityNextBatchStart = 0;
	m_reachabilityBatchRollingQ.clear();
	enqueueReachabilityBatch(0);
}

void RobotSimulationController::enqueueReachabilityBatch(const int batchStart)
{
	if (!m_host || !m_host->simulationCommandPage())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc || !doc->hasRobotSimulationContext())
	{
		return;
	}
	const int instIdx = m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0
							? m_host->simulationCommandPage()->currentRobotInstanceIndex()
							: 0;
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (nj <= 0)
	{
		return;
	}
	const QString robotBackendId = m_host->simulationCommandPage()->currentRobotBackendId();
	const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
		m_host->simulationCommandPage()->instructions(robotBackendId);
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	if (motions.empty() || batchStart < 0 || batchStart >= static_cast<int>(motions.size()))
	{
		return;
	}
	constexpr int kReachabilityBatchSize = 64;
	const int end = std::min(static_cast<int>(motions.size()), batchStart + kReachabilityBatchSize);
	const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
	const QString defaultTcpLinkName = RobotSimulationMath::defaultTcpLinkNameForUrdf(
		urdfPath, m_host->simulationCommandPage() ? m_host->simulationCommandPage()->selectedTcpLink() : QString());
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	const std::vector<robot_kinematics::DhRow> dhRows; // 有 URDF 不建 DH，worker 走数值 IK

	ReachabilityJobInput input;
	input.frames = frames;
	input.batchStart = batchStart;
	if (batchStart == 0 || m_reachabilityBatchRollingQ.size() != nj)
	{
		input.programStartQ = motionPreviewProgramStartJointsLocal(nj, jointOffset);
	}
	else
	{
		input.programStartQ = m_reachabilityBatchRollingQ;
	}
	input.steps.reserve(end - batchStart);
	for (int i = batchStart; i < end; ++i)
	{
		const RobotInstruction::Base* motionPtr = motions[static_cast<size_t>(i)];
		if (!motionPtr)
		{
			continue;
		}
		ReachabilityJobStep step;
		step.instructionId = QString::fromStdString(motionPtr->id());
		step.planPayload = makePlanJobPayload(*motionPtr, input.programStartQ, urdfPath, defaultTcpLinkName, dhRows,
											  &doc->robotExternalAxesForInstance(instIdx));
		attachWorkpieceIkFrameToPayload(step.planPayload, doc, instIdx);
		input.steps.push_back(std::move(step));
	}
	if (input.steps.isEmpty())
	{
		if (end < static_cast<int>(motions.size()))
		{
			enqueueReachabilityBatch(end);
		}
		return;
	}

	const quint64 token = m_reachabilityJobToken;
	const auto jobResult = std::make_shared<ReachabilityJobOutput>();
	QPointer<RobotSimulationController> guard(this);
	++m_reachabilityPendingJobs;
	m_host->enqueueBackgroundJob(
		QStringLiteral("Motion reachability"),
		[input = std::move(input), jobResult]() { *jobResult = runReachabilityJob(input); },
		[this, guard, token, jobResult, end, nj, motionCount = static_cast<int>(motions.size())](const bool threw,
																								 const QString&)
		{
			--m_reachabilityPendingJobs;
			if (!guard || threw || token != m_reachabilityJobToken)
			{
				return;
			}
			for (auto it = jobResult->reachability.constBegin(); it != jobResult->reachability.constEnd(); ++it)
			{
				m_motionReachabilityCache.insert(it.key(), it.value());
			}
			if (jobResult->rollingEndQ.size() == nj)
			{
				m_reachabilityBatchRollingQ = jobResult->rollingEndQ;
			}
			refreshInstructionPoseAxesWithReachability(m_motionReachabilityCache);
			if (end < motionCount)
			{
				enqueueReachabilityBatch(end);
			}
		});
}

bool RobotSimulationController::resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath)
{
	outBackendId.clear();
	outStepPath.clear();
	if (!m_simulationDock || !m_simulationDock->featureTrajectoryPage())
	{
		return false;
	}
	FeatureTrajectoryPageWidget* feat = m_simulationDock->featureTrajectoryPage();
	if (!feat->currentWorkpiece(outBackendId, outStepPath))
	{
		return false;
	}
	(void)feat->ensureFeatureCatalogEnumerated(nullptr);
	return true;
}

bool RobotSimulationController::showAiFeatureCandidatePreview(const QByteArray& catalogSliceUtf8, QString* err)
{
	if (!m_simulationDock || !m_simulationDock->featureTrajectoryPage())
	{
		if (err)
		{
			*err = QStringLiteral("特征轨迹页不可用");
		}
		return false;
	}
	if (!m_simulationDock->featureTrajectoryPage()->buildAndShowCandidatePreview(catalogSliceUtf8))
	{
		if (err)
		{
			*err = QStringLiteral("候选预览构建失败");
		}
		return false;
	}
	showRobotDockTab(RobotSimulationDockWidget::kTabIndexTrajectoryGeneration);
	return true;
}

void RobotSimulationController::clearAiFeatureCandidatePreview()
{
	if (m_simulationDock && m_simulationDock->featureTrajectoryPage())
	{
		m_simulationDock->featureTrajectoryPage()->clearCandidatePreview();
	}
}

bool RobotSimulationController::commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* summary,
														   QString* err)
{
	if (!m_simulationDock || !m_simulationDock->featureTrajectoryPage())
	{
		if (err)
		{
			*err = QStringLiteral("特征轨迹页不可用");
		}
		return false;
	}
	return m_simulationDock->featureTrajectoryPage()->commitFeaturePlanFromAi(featurePlanJsonUtf8, summary, err);
}

int RobotSimulationController::proposeAndConfirmTrajectoryPlan(const QByteArray& planIn, QByteArray& planOut,
															   QString* err, const bool showRetry)
{
	if (!m_simulationDock || !m_simulationDock->featureTrajectoryPage())
	{
		if (err)
			*err = QStringLiteral("特征轨迹页不可用");
		return 0;
	}
	return m_simulationDock->featureTrajectoryPage()->proposeAndConfirmTrajectoryPlan(planIn, planOut, err, showRetry);
}

bool RobotSimulationController::loadBoundTrajectoryPlanForAi(QByteArray& planOut, QString* err)
{
	if (!m_simulationDock || !m_simulationDock->featureTrajectoryPage())
	{
		if (err)
			*err = QStringLiteral("特征轨迹页不可用");
		return false;
	}
	return m_simulationDock->featureTrajectoryPage()->loadBoundTrajectoryPlanJson(planOut, err);
}

bool RobotSimulationController::reviseAiTrajectoryPlan(const QByteArray& planJsonUtf8, QString* summary, QString* err)
{
	if (!m_simulationDock || !m_simulationDock->featureTrajectoryPage())
	{
		if (err)
			*err = QStringLiteral("特征轨迹页不可用");
		return false;
	}
	return m_simulationDock->featureTrajectoryPage()->reviseFeaturePlanFromAi(planJsonUtf8, summary, err);
}
