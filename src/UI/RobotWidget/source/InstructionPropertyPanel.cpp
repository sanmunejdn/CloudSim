/// @file InstructionPropertyPanel.cpp
/// @brief InstructionPropertyPanel 实现

#include "InstructionPropertyPanel.h"

#include "BackendPropertyRow.h"
#include "CoreTypes.h"
#include "IRobotDocumentHost.h"
#include "IRobotInstructionPropertyUiHost.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionController.h"
#include "RobotInstructionPlanningHelpers.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionPropertySchema.h"
#include "SimulationCommandWidget.h"
#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

#include <QMetaObject>
#include <QPointer>
#include <algorithm>
#include <memory>

namespace
{
bool instructionUsesActiveUserFrame(const RobotInstruction::Base& ins)
{
	return RobotCoordinate::instructionTargetDisplayUsesUserFrame(ins.extensionProperties());
}

void instructionTcpInBase(const RobotInstruction::Base& ins, BackendMat4& out)
{
	const RobotInstruction::Vec3 p = ins.pose();
	const RobotInstruction::Vec3 e = ins.eulerDeg();
	out = RobotCoordinate::tcpInBaseFromPose(p.x, p.y, p.z, e.x, e.y, e.z);
}

void setInstructionTcpInBase(RobotInstruction::Base& ins, const BackendMat4& T_base_tcp)
{
	double pos[3]{};
	double euler[3]{};
	RobotCoordinate::poseEulerFromTcpInBase(T_base_tcp, pos, euler);
	RobotInstruction::Vec3 p{};
	p.x = pos[0];
	p.y = pos[1];
	p.z = pos[2];
	ins.setPose(p);
	RobotInstruction::Vec3 e{};
	e.x = euler[0];
	e.y = euler[1];
	e.z = euler[2];
	ins.setEulerDeg(e);
}

BackendMat4 instructionTcpForDisplay(IRobotInstructionPropertyUiHost& host, const RobotInstruction::Base& ins)
{
	const BackendMat4 T_base_tcp = [&]()
	{
		BackendMat4 t{};
		instructionTcpInBase(ins, t);
		return t;
	}();
	if (!instructionUsesActiveUserFrame(ins))
	{
		return T_base_tcp;
	}
	IRobotDocumentHost* doc = host.currentRobotDocument();
	if (!doc)
	{
		return T_base_tcp;
	}
	const int instIdx = host.currentSimulationRobotInstanceIndex();
	if (instIdx < 0)
	{
		return T_base_tcp;
	}
	const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
	const RobotCoordinate::RobotUserFrame* uf =
		RobotCoordinate::resolveUserFrameForExtension(frames, ins.extensionProperties());
	if (!uf)
	{
		return T_base_tcp;
	}
	const BackendMat4 T_base_user = RobotCoordinate::frameToMat4(uf->T_base_user);
	return RobotCoordinate::tcpInUserFromBaseTcp(T_base_user, T_base_tcp);
}

void applyInstructionPropertyViaService(IRobotInstructionPropertyUiHost& host,
										const std::shared_ptr<RobotInstruction::Base>& instruction, const char* key,
										const std::string& value)
{
	if (!instruction)
	{
		return;
	}
	if (host.applyInstructionPropertyChange(QString::fromStdString(instruction->id()), QString::fromLatin1(key),
											QString::fromStdString(value), nullptr))
	{
		return;
	}
	instruction->applyPropertyChange(key, value, nullptr);
}

bool isMotionTargetPoseKey(const QString& key)
{
	return key == QStringLiteral("motion.target.pose.x") || key == QStringLiteral("motion.target.pose.y") ||
		   key == QStringLiteral("motion.target.pose.z") || key == QStringLiteral("motion.target.euler.rx") ||
		   key == QStringLiteral("motion.target.euler.ry") || key == QStringLiteral("motion.target.euler.rz");
}

bool isInstructionPanelManagedExtensionKey(const std::string& keyStr)
{
	if (keyStr == RobotCoordinate::kExtMotionToolFrameId || keyStr == RobotCoordinate::kExtMotionUserFrameId ||
		keyStr == RobotCoordinate::kExtMotionTargetFrame)
	{
		return true;
	}
	if (keyStr.rfind("render.", 0) == 0 || keyStr.rfind("context.", 0) == 0)
	{
		return true;
	}
	return false;
}

QString snapshotPropertyValueForKey(const nlohmann::json& rows, const QString& key)
{
	if (!rows.is_array() || key.isEmpty())
	{
		return {};
	}
	const std::string want = key.toStdString();
	for (const auto& r : rows)
	{
		if (!r.is_object())
		{
			continue;
		}
		if (r.value("key", std::string()) == want)
		{
			return QString::fromStdString(r.value("value", std::string()));
		}
	}
	return {};
}

cloudsim::core::FeasibleMotionAxisOptionsDto
toFeasibleAxisDto(const RobotInstruction::FeasibleMotionAxisConfigurationOptions& engine)
{
	auto toList = [](const std::vector<std::string>& tokens)
	{
		QStringList list;
		for (const std::string& t : tokens)
		{
			list.push_back(QString::fromStdString(t));
		}
		return list;
	};
	cloudsim::core::FeasibleMotionAxisOptionsDto dto;
	dto.presetTokens = toList(engine.presetTokens);
	dto.elbowTokens = toList(engine.elbowTokens);
	dto.wristTokens = toList(engine.wristTokens);
	dto.armTokens = toList(engine.armTokens);
	dto.turnJ1Tokens = toList(engine.turnJ1Tokens);
	dto.turnJ4Tokens = toList(engine.turnJ4Tokens);
	dto.turnJ6Tokens = toList(engine.turnJ6Tokens);
	return dto;
}

RobotInstruction::FeasibleMotionAxisConfigurationOptions
fromFeasibleAxisDto(const cloudsim::core::FeasibleMotionAxisOptionsDto& dto)
{
	auto fill = [](std::vector<std::string>& dest, const QStringList& src)
	{
		dest.reserve(static_cast<size_t>(src.size()));
		for (const QString& t : src)
		{
			dest.push_back(t.toStdString());
		}
	};
	RobotInstruction::FeasibleMotionAxisConfigurationOptions out;
	fill(out.presetTokens, dto.presetTokens);
	fill(out.elbowTokens, dto.elbowTokens);
	fill(out.wristTokens, dto.wristTokens);
	fill(out.armTokens, dto.armTokens);
	fill(out.turnJ1Tokens, dto.turnJ1Tokens);
	fill(out.turnJ4Tokens, dto.turnJ4Tokens);
	fill(out.turnJ6Tokens, dto.turnJ6Tokens);
	return out;
}

} // namespace
void InstructionPropertyPanel::applySuggestedAxisPresetFromSeedIfNeeded(
	IRobotInstructionPropertyUiHost& host, const std::shared_ptr<RobotInstruction::Base>& instruction,
	const QVector<double>& seedJointRad, const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible)
{
	if (!instruction || !instruction->hasMotionAxisConfigurationProperty() || feasible.presetTokens.empty() ||
		seedJointRad.isEmpty())
	{
		return;
	}
	const auto tokenAllowed = [&feasible](const std::string& token) {
		return std::find(feasible.presetTokens.begin(), feasible.presetTokens.end(), token) !=
			   feasible.presetTokens.end();
	};
	const auto pickFallback = [&]() -> std::string
	{
		if (tokenAllowed("AUTO"))
		{
			return "AUTO";
		}
		return feasible.presetTokens.front();
	};

	RobotInstruction::MotionAxisConfiguration cur = instruction->motionAxisConfiguration();
	const std::string curPreset = cur.preset;
	const bool presetLocked = !curPreset.empty() && curPreset != "AUTO" && tokenAllowed(curPreset);

	IRobotDocumentHost* doc = host.currentRobotDocument();
	if (!doc || !host.simulationCommandPage())
	{
		return;
	}
	const int instIdx = host.simulationCommandPage()->currentRobotInstanceIndex();
	if (instIdx < 0)
	{
		return;
	}
	const QStringList jnames = doc->robotRevoluteJointNamesForInstance(instIdx);
	std::vector<std::string> jointNames;
	jointNames.reserve(jnames.size());
	for (const QString& jn : jnames)
	{
		jointNames.push_back(jn.toStdString());
	}
	std::vector<double> seed(seedJointRad.size());
	for (int i = 0; i < seedJointRad.size(); ++i)
	{
		seed[static_cast<size_t>(i)] = seedJointRad[i];
	}
	const RobotInstruction::JointConfigurationClass observed =
		RobotInstruction::classifyJointConfiguration(seed, jointNames, &seed);
	if (!presetLocked)
	{
		std::string suggested = RobotInstruction::suggestMotionAxisPresetToken(observed);
		if (!tokenAllowed(suggested))
		{
			suggested = pickFallback();
		}
		if (suggested != curPreset)
		{
			applyInstructionPropertyViaService(host, instruction, "motion.axisConfig.preset", suggested);
			cur = instruction->motionAxisConfiguration();
		}
	}
	const auto turnAllowed = [](const std::vector<std::string>& allowed, const std::string& tok)
	{ return std::find(allowed.begin(), allowed.end(), tok) != allowed.end(); };
	const auto applyTurnIfAuto =
		[&](const char* key, const int currentTurn, const int observedTurn, const std::vector<std::string>& allowed)
	{
		if (currentTurn != RobotInstruction::kMotionAxisTurnAuto)
		{
			return;
		}
		const std::string tok = RobotInstruction::jointTurnToToken(observedTurn);
		if (!turnAllowed(allowed, tok))
		{
			return;
		}
		applyInstructionPropertyViaService(host, instruction, key, tok);
	};
	applyTurnIfAuto("motion.axisConfig.turn.j1", cur.turnJ1, observed.turnJ1, feasible.turnJ1Tokens);
	applyTurnIfAuto("motion.axisConfig.turn.j4", cur.turnJ4, observed.turnJ4, feasible.turnJ4Tokens);
	applyTurnIfAuto("motion.axisConfig.turn.j6", cur.turnJ6, observed.turnJ6, feasible.turnJ6Tokens);
}

void InstructionPropertyPanel::update(IRobotInstructionPropertyUiHost& host,
									  const std::shared_ptr<RobotInstruction::Base>& instruction,
									  const bool refreshFeasibleAxisOptions)
{
	if (!host.propertyBrowser() || !host.variantManager())
	{
		return;
	}
	host.updatingPropertyBrowserFlag() = true;
	host.propertyEnumTokens().clear();
	host.clearPropertyKeyVariantMap();
	host.variantManager()->clear();
	if (!instruction)
	{
		host.setActiveInstructionForProperty(nullptr);
		host.updatingPropertyBrowserFlag() = false;
		return;
	}
	host.setActiveInstructionForProperty(instruction);

	host.appendPropertyBrowserRow(QStringLiteral("core.id"),
								  host.propertyDisplayLabelForKey(QStringLiteral("core.id"), QStringLiteral("ID")),
								  QString::fromStdString(instruction->id()), false);
	host.appendPropertyBrowserRow(QStringLiteral("core.name"),
								  host.propertyDisplayLabelForKey(QStringLiteral("core.name"), QStringLiteral("Name")),
								  QString::fromStdString(instruction->name()), false);

	if (RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		const int pointIndex = RobotInstruction::motionPointIndex(*instruction);
		QString pointValue = QStringLiteral("-");
		if (pointIndex > 0)
		{
			pointValue = QString::fromStdString(RobotInstruction::formatMotionPointName(pointIndex));
			pointValue += host.i18n(QStringLiteral(" (Point %1)"), QStringLiteral("（第 %1 点）")).arg(pointIndex);
		}
		host.appendPropertyBrowserRow(
			QStringLiteral("motion.pointIndex"),
			host.propertyDisplayLabelForKey(QStringLiteral("motion.pointIndex"), QStringLiteral("Waypoint index")),
			pointValue, false);
	}

	const nlohmann::json rows = instruction->snapshotPropertyRows();

	RobotInstruction::FeasibleMotionAxisConfigurationOptions feasibleAxis;
	QVector<double> seedJointRad;
	bool deferFullAxisProbe = false;
	if (RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		if (refreshFeasibleAxisOptions)
		{
			feasibleAxis = fromFeasibleAxisDto(host.cachedFeasibleMotionAxisOptionsDto());
			deferFullAxisProbe = true;
		}
		else
		{
			feasibleAxis = fromFeasibleAxisDto(host.cachedFeasibleMotionAxisOptionsDto());
		}
		const auto ensureToken = [&](const char* key, const std::vector<std::string>& allowed, const QString& current)
		{
			if (allowed.empty() || !refreshFeasibleAxisOptions)
			{
				return;
			}
			const std::string cur = current.trimmed().toUpper().toStdString();
			if (cur.empty() || std::find(allowed.begin(), allowed.end(), cur) != allowed.end())
			{
				return;
			}
			std::string fallback = allowed.front();
			if (key == std::string("motion.axisConfig.preset") && !seedJointRad.isEmpty())
			{
				IRobotDocumentHost* doc = host.currentRobotDocument();
				if (doc && host.simulationCommandPage())
				{
					const int instIdx = host.simulationCommandPage()->currentRobotInstanceIndex();
					if (instIdx >= 0)
					{
						const QStringList jnames = doc->robotRevoluteJointNamesForInstance(instIdx);
						std::vector<std::string> jointNames;
						std::vector<double> seed(seedJointRad.size());
						for (int i = 0; i < seedJointRad.size(); ++i)
						{
							seed[static_cast<size_t>(i)] = seedJointRad[i];
						}
						for (const QString& jn : jnames)
						{
							jointNames.push_back(jn.toStdString());
						}
						const std::string suggested = RobotInstruction::suggestMotionAxisPresetToken(
							RobotInstruction::classifyJointConfiguration(seed, jointNames, &seed));
						if (std::find(allowed.begin(), allowed.end(), suggested) != allowed.end())
						{
							fallback = suggested;
						}
						else if (std::find(allowed.begin(), allowed.end(), std::string("AUTO")) != allowed.end())
						{
							fallback = "AUTO";
						}
					}
				}
			}
			applyInstructionPropertyViaService(host, instruction, key, fallback);
		};
		ensureToken("motion.axisConfig.preset", feasibleAxis.presetTokens,
					snapshotPropertyValueForKey(rows, QStringLiteral("motion.axisConfig.preset")));
		ensureToken("motion.axisConfig.turn.j1", feasibleAxis.turnJ1Tokens,
					snapshotPropertyValueForKey(rows, QStringLiteral("motion.axisConfig.turn.j1")));
		ensureToken("motion.axisConfig.turn.j4", feasibleAxis.turnJ4Tokens,
					snapshotPropertyValueForKey(rows, QStringLiteral("motion.axisConfig.turn.j4")));
		ensureToken("motion.axisConfig.turn.j6", feasibleAxis.turnJ6Tokens,
					snapshotPropertyValueForKey(rows, QStringLiteral("motion.axisConfig.turn.j6")));
	}

	nlohmann::json rowsAfter = instruction->snapshotPropertyRows();
	if (RobotInstruction::isMotionWaypointType(instruction->type()))
	{
		const QString presetMid = snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.preset"));
		const bool customAxisMode = presetMid.compare(QStringLiteral("CUSTOM"), Qt::CaseInsensitive) == 0;
		if (customAxisMode)
		{
			const auto ensureToken =
				[&](const char* key, const std::vector<std::string>& allowed, const QString& current)
			{
				if (allowed.empty())
				{
					return;
				}
				const std::string cur = current.trimmed().toUpper().toStdString();
				if (cur.empty() || std::find(allowed.begin(), allowed.end(), cur) != allowed.end())
				{
					return;
				}
				applyInstructionPropertyViaService(host, instruction, key, allowed.front());
			};
			ensureToken("motion.axisConfig.elbow", feasibleAxis.elbowTokens,
						snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.elbow")));
			ensureToken("motion.axisConfig.wrist", feasibleAxis.wristTokens,
						snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.wrist")));
			ensureToken("motion.axisConfig.arm", feasibleAxis.armTokens,
						snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.arm")));
			rowsAfter = instruction->snapshotPropertyRows();
		}
	}

	const QString presetAfter = snapshotPropertyValueForKey(rowsAfter, QStringLiteral("motion.axisConfig.preset"));
	const bool customAxisModeAfter = presetAfter.compare(QStringLiteral("CUSTOM"), Qt::CaseInsensitive) == 0;

	if (instruction->hasPoseProperty())
	{
		const auto& ext = instruction->extensionProperties();
		IRobotDocumentHost* doc = host.currentRobotDocument();
		const int instIdx =
			host.simulationCommandPage() ? host.simulationCommandPage()->currentRobotInstanceIndex() : -1;
		std::vector<std::string> toolTokens = {"active"};
		std::vector<std::string> userTokens = {"active"};
		QStringList toolEnumNames;
		QStringList userEnumNames;
		toolEnumNames << host.i18n(QStringLiteral("Active (follow robot)"), QStringLiteral("跟随当前工具"));
		userEnumNames << host.i18n(QStringLiteral("Active (follow robot)"), QStringLiteral("跟随当前用户系"));
		if (doc && instIdx >= 0)
		{
			const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
			for (const RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
			{
				toolTokens.push_back(tf.id);
				QString label = QString::fromStdString(tf.name);
				if (!tf.id.empty() && label != QString::fromStdString(tf.id))
				{
					label += QStringLiteral(" (") + QString::fromStdString(tf.id) + QLatin1Char(')');
				}
				toolEnumNames << label;
			}
			for (const RobotCoordinate::RobotUserFrame& uf : frames.userFrames)
			{
				userTokens.push_back(uf.id);
				QString label = QString::fromStdString(uf.name);
				if (!uf.id.empty() && label != QString::fromStdString(uf.id))
				{
					label += QStringLiteral(" (") + QString::fromStdString(uf.id) + QLatin1Char(')');
				}
				userEnumNames << label;
			}
		}
		QString toolVal = QStringLiteral("active");
		const auto itTool = ext.find(RobotCoordinate::kExtMotionToolFrameId);
		if (itTool != ext.end() && !itTool->second.empty())
		{
			toolVal = QString::fromStdString(itTool->second);
		}
		host.appendPropertyBrowserRow(
			QStringLiteral("motion.tool.frameId"),
			host.propertyDisplayLabelForKey(QStringLiteral("motion.tool.frameId"), QStringLiteral("Tool frame")),
			toolVal, true, &toolTokens, &toolEnumNames,
			host.i18n(
				QStringLiteral("When changing the tool frame, the TCP position in space is kept; joint angles are "
							   "recomputed automatically."),
				QStringLiteral("切换工具系时，系统将保持工具尖端（TCP）空间位置不变，自动重新计算关节角度。")));
		QString userVal = QStringLiteral("active");
		const auto itUser = ext.find(RobotCoordinate::kExtMotionUserFrameId);
		if (itUser != ext.end() && !itUser->second.empty())
		{
			userVal = QString::fromStdString(itUser->second);
		}
		host.appendPropertyBrowserRow(
			QStringLiteral("motion.user.frameId"),
			host.propertyDisplayLabelForKey(QStringLiteral("motion.user.frameId"), QStringLiteral("User frame")),
			userVal, true, &userTokens, &userEnumNames);
		QString frameVal = QStringLiteral("base");
		const auto itFr = ext.find(RobotCoordinate::kExtMotionTargetFrame);
		if (itFr != ext.end() && !itFr->second.empty())
		{
			frameVal = QString::fromStdString(itFr->second);
			if (frameVal == QStringLiteral("active_user"))
			{
				frameVal = QStringLiteral("user");
			}
		}
		static const std::vector<std::string> frameTokens = {"base", "user"};
		const QStringList frameEnumNames = {
			host.i18n(QStringLiteral("Robot base (TCP)"), QStringLiteral("机器人基座 (TCP)")),
			host.i18n(QStringLiteral("User frame (TCP)"), QStringLiteral("用户坐标系 (TCP)")),
		};
		host.appendPropertyBrowserRow(
			QStringLiteral("motion.target.frame"),
			host.propertyDisplayLabelForKey(QStringLiteral("motion.target.frame"), QStringLiteral("Target frame")),
			frameVal, true, &frameTokens, &frameEnumNames);
	}

	const BackendMat4 T_display = instructionTcpForDisplay(host, *instruction);

	if (rowsAfter.is_array())
	{
		for (const auto& r : rowsAfter)
		{
			if (!r.is_object())
			{
				continue;
			}
			const std::string keyStr = r.value(backend_property_json::kKey, std::string());
			if (isInstructionPanelManagedExtensionKey(keyStr) || keyStr.rfind("legacy.", 0) == 0 ||
				keyStr == "motion.durationSec" || keyStr == RobotInstruction::kMotionPointIndexKey)
			{
				continue;
			}
			if (!customAxisModeAfter && (keyStr == "motion.axisConfig.elbow" || keyStr == "motion.axisConfig.wrist" ||
										 keyStr == "motion.axisConfig.arm"))
			{
				continue;
			}
			const std::string labelStr = r.value(backend_property_json::kLabelEn, std::string());
			const bool editable = r.value(backend_property_json::kEditable, false);
			std::string valueStr = r.value(backend_property_json::kValue, std::string());
			const QString key = QString::fromStdString(keyStr);
			if (isMotionTargetPoseKey(key))
			{
				const RobotCoordinate::RobotRigidFrame disp = RobotCoordinate::mat4ToFrame(T_display);
				if (key == QStringLiteral("motion.target.pose.x"))
				{
					valueStr = std::to_string(disp.positionMm[0]);
				}
				else if (key == QStringLiteral("motion.target.pose.y"))
				{
					valueStr = std::to_string(disp.positionMm[1]);
				}
				else if (key == QStringLiteral("motion.target.pose.z"))
				{
					valueStr = std::to_string(disp.positionMm[2]);
				}
				else if (key == QStringLiteral("motion.target.euler.rx"))
				{
					valueStr = std::to_string(disp.eulerDeg[0]);
				}
				else if (key == QStringLiteral("motion.target.euler.ry"))
				{
					valueStr = std::to_string(disp.eulerDeg[1]);
				}
				else if (key == QStringLiteral("motion.target.euler.rz"))
				{
					valueStr = std::to_string(disp.eulerDeg[2]);
				}
			}
			QString label = host.propertyDisplayLabelForKey(key, QString::fromStdString(labelStr));
			if (instructionUsesActiveUserFrame(*instruction) && isMotionTargetPoseKey(key))
			{
				IRobotDocumentHost* doc = host.currentRobotDocument();
				if (doc && host.simulationCommandPage())
				{
					const int instIdx = host.simulationCommandPage()->currentRobotInstanceIndex();
					if (instIdx >= 0)
					{
						const RobotCoordinate::RobotCoordinateFrameSet& frames =
							doc->robotCoordinateFramesForInstance(instIdx);
						if (const RobotCoordinate::RobotUserFrame* uf = RobotCoordinate::resolveUserFrameForExtension(
								frames, instruction->extensionProperties()))
						{
							label += QStringLiteral(" (%1)").arg(QString::fromStdString(uf->name));
						}
					}
				}
			}
			else if (isMotionTargetPoseKey(key))
			{
				const auto& extPose = instruction->extensionProperties();
				const auto itToolId = extPose.find(RobotCoordinate::kExtMotionToolFrameId);
				if (itToolId != extPose.end() && !itToolId->second.empty() && itToolId->second != "active")
				{
					label += host.i18n(QStringLiteral(" (TCP mm)"), QStringLiteral(" (TCP mm)"));
				}
			}
			const std::vector<std::string>* enumOverride = nullptr;
			if (key == QStringLiteral("motion.axisConfig.preset") && !feasibleAxis.presetTokens.empty())
			{
				enumOverride = &feasibleAxis.presetTokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.elbow") && !feasibleAxis.elbowTokens.empty())
			{
				enumOverride = &feasibleAxis.elbowTokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.wrist") && !feasibleAxis.wristTokens.empty())
			{
				enumOverride = &feasibleAxis.wristTokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.arm") && !feasibleAxis.armTokens.empty())
			{
				enumOverride = &feasibleAxis.armTokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.turn.j1") && !feasibleAxis.turnJ1Tokens.empty())
			{
				enumOverride = &feasibleAxis.turnJ1Tokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.turn.j4") && !feasibleAxis.turnJ4Tokens.empty())
			{
				enumOverride = &feasibleAxis.turnJ4Tokens;
			}
			else if (key == QStringLiteral("motion.axisConfig.turn.j6") && !feasibleAxis.turnJ6Tokens.empty())
			{
				enumOverride = &feasibleAxis.turnJ6Tokens;
			}
			host.appendPropertyBrowserRow(key, label, QString::fromStdString(valueStr), editable, enumOverride);
		}
	}

	host.updatingPropertyBrowserFlag() = false;

	if (deferFullAxisProbe)
	{
		host.scheduleDeferredFeasibleAxisProbe(instruction);
	}
}

bool InstructionPropertyPanel::handleVariantPropertyValueChanged(IRobotInstructionPropertyUiHost& host,
																 QtProperty* property, const QVariant& value)
{
	const std::shared_ptr<RobotInstruction::Base> instruction = host.activeInstructionForProperty();
	if (!instruction)
	{
		return false;
	}
	const QString propertyKey = property->whatsThis();
	if (propertyKey.isEmpty() || propertyKey.startsWith(QStringLiteral("core.")))
	{
		return true;
	}
	QString valueText = host.instructionEnumTokenFromProperty(property, value);
	if (propertyKey == QStringLiteral("motion.target.frame"))
	{
		std::string tok = valueText.toStdString();
		if (tok == "active_user")
		{
			tok = "user";
		}
		instruction->setExtensionProperty(RobotCoordinate::kExtMotionTargetFrame, tok);
		host.invalidateFeasibleAxisConfigurationCache();
		host.scheduleInstructionPropertyRefresh(instruction, false);
		host.refreshInstructionPoseAxes();
		return true;
	}
	if (propertyKey == QStringLiteral("motion.tool.frameId"))
	{
		instruction->setExtensionProperty(RobotCoordinate::kExtMotionToolFrameId, valueText.toStdString());
		if (IRobotDocumentHost* doc = host.currentRobotDocument())
		{
			const int instIdx =
				host.simulationCommandPage() ? host.simulationCommandPage()->currentRobotInstanceIndex() : -1;
			if (instIdx >= 0)
			{
				const RobotCoordinate::RobotCoordinateFrameSet& frames = doc->robotCoordinateFramesForInstance(instIdx);
				RobotInstructionPlanning::syncInstructionToolContextFromFrames(*instruction, frames);
			}
		}
		int changedMotionIndex = -1;
		if (host.simulationCommandPage())
		{
			const std::vector<std::shared_ptr<RobotInstruction::Base>> program =
				host.simulationCommandPage()->instructions(host.simulationCommandPage()->currentRobotBackendId());
			const std::vector<const RobotInstruction::Base*> motions =
				RobotInstruction::collectMotionInstructions(program);
			for (size_t i = 0; i < motions.size(); ++i)
			{
				if (motions[i] && motions[i]->id() == instruction->id())
				{
					changedMotionIndex = static_cast<int>(i);
					break;
				}
			}
			if (changedMotionIndex >= 0)
			{
				RobotInstructionPlanning::invalidateTaughtJointsFromMotionIndexForward(motions, changedMotionIndex);
			}
			else
			{
				RobotInstructionPlanning::invalidateTaughtJointsForToolFrameChange(*instruction);
			}
		}
		else
		{
			RobotInstructionPlanning::invalidateTaughtJointsForToolFrameChange(*instruction);
		}
		host.syncInstructionRenderMatricesFromPose(instruction);
		host.invalidateFeasibleAxisConfigurationCache();
		host.applyRobotPoseForInstructionPreview(instruction);
		host.scheduleInstructionPropertyRefresh(instruction, false);
		host.refreshRobotCoordinateFrameOverlays(instruction);
		host.refreshInstructionPoseAxes();
		{
			QString toolLabel = valueText;
			if (IRobotDocumentHost* doc = host.currentRobotDocument())
			{
				const int instIdx =
					host.simulationCommandPage() ? host.simulationCommandPage()->currentRobotInstanceIndex() : -1;
				if (instIdx >= 0)
				{
					const RobotCoordinate::RobotCoordinateFrameSet& frames =
						doc->robotCoordinateFramesForInstance(instIdx);
					if (const RobotCoordinate::RobotToolFrame* tool =
							RobotCoordinate::resolveToolFrameForExtension(frames, instruction->extensionProperties()))
					{
						toolLabel = QString::fromStdString(tool->name);
					}
				}
			}
			host.appendRunInfoMessage(
				QStringLiteral("Tool frame → %1; TCP target unchanged, flange IK and preview updated.").arg(toolLabel),
				QStringLiteral("工具系 → %1；TCP 目标未变，已重算法兰 IK 与预览。").arg(toolLabel));
		}
		return true;
	}
	if (propertyKey == QStringLiteral("motion.user.frameId"))
	{
		instruction->setExtensionProperty(RobotCoordinate::kExtMotionUserFrameId, valueText.toStdString());
		host.scheduleInstructionPropertyRefresh(instruction, false);
		host.refreshInstructionPoseAxes();
		return true;
	}
	if (isMotionTargetPoseKey(propertyKey))
	{
		BackendMat4 T_disp = instructionTcpForDisplay(host, *instruction);
		RobotCoordinate::RobotRigidFrame disp = RobotCoordinate::mat4ToFrame(T_disp);
		bool ok = false;
		const double v = valueText.toDouble(&ok);
		if (ok)
		{
			if (propertyKey == QStringLiteral("motion.target.pose.x"))
			{
				disp.positionMm[0] = v;
			}
			else if (propertyKey == QStringLiteral("motion.target.pose.y"))
			{
				disp.positionMm[1] = v;
			}
			else if (propertyKey == QStringLiteral("motion.target.pose.z"))
			{
				disp.positionMm[2] = v;
			}
			else if (propertyKey == QStringLiteral("motion.target.euler.rx"))
			{
				disp.eulerDeg[0] = v;
			}
			else if (propertyKey == QStringLiteral("motion.target.euler.ry"))
			{
				disp.eulerDeg[1] = v;
			}
			else if (propertyKey == QStringLiteral("motion.target.euler.rz"))
			{
				disp.eulerDeg[2] = v;
			}
		}
		BackendMat4 T_base_tcp;
		if (instructionUsesActiveUserFrame(*instruction))
		{
			IRobotDocumentHost* doc = host.currentRobotDocument();
			BackendMat4 T_base_user = BackendMat4::identity();
			if (doc && host.simulationCommandPage())
			{
				const int instIdx = host.simulationCommandPage()->currentRobotInstanceIndex();
				if (instIdx >= 0)
				{
					const RobotCoordinate::RobotCoordinateFrameSet& frames =
						doc->robotCoordinateFramesForInstance(instIdx);
					if (const RobotCoordinate::RobotUserFrame* uf =
							RobotCoordinate::resolveUserFrameForExtension(frames, instruction->extensionProperties()))
					{
						T_base_user = RobotCoordinate::frameToMat4(uf->T_base_user);
					}
				}
			}
			const BackendMat4 T_user_tcp = RobotCoordinate::frameToMat4(disp);
			T_base_tcp = RobotCoordinate::tcpInBaseFromUserTcp(T_base_user, T_user_tcp);
		}
		else
		{
			T_base_tcp = RobotCoordinate::frameToMat4(disp);
		}
		setInstructionTcpInBase(*instruction, T_base_tcp);
		host.syncInstructionRenderMatricesFromPose(instruction);
	}
	else
	{
		std::string err;
		if (!instruction->applyPropertyChange(propertyKey.toStdString(), valueText.toStdString(), &err))
		{
			host.scheduleInstructionPropertyRefresh(instruction, true);
			return true;
		}
	}
	if (host.simulationCommandPage())
	{
		host.simulationCommandPage()->refreshInstructionList();
	}
	host.refreshInstructionPoseAxes();
	const bool axisConfigOnly = propertyKey.startsWith(QStringLiteral("motion.axisConfig"));
	if (!axisConfigOnly)
	{
		host.invalidateFeasibleAxisConfigurationCache();
	}
	if (isMotionTargetPoseKey(propertyKey))
	{
		const QString instructionId = QString::fromStdString(instruction->id());
		host.notifyPropertyPanelNumericEditStarted(instructionId, propertyKey);
	}
	else
	{
		host.applyRobotPoseForInstructionPreview(instruction);
		host.scheduleInstructionPropertyRefresh(instruction, !axisConfigOnly);
	}
	return true;
}
