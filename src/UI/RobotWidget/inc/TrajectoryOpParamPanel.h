#ifndef ROBOTWIDGET_TRAJECTORYOPPARAMPANEL_H
#define ROBOTWIDGET_TRAJECTORYOPPARAMPANEL_H

/// @file TrajectoryOpParamPanel.h
/// @brief TrajectoryOpParamPanel 接口

#include "robotwidget_global.h"

#include "TrajectoryParamWidgetFactory.h"

#include <QWidget>
#include <string>
#include <vector>

#include <ITrajectoryOp.h>
#include <TrajectoryPipelineTypes.h>

class QComboBox;
class QFormLayout;
class QLabel;

class ROBOTWIDGET_EXPORT TrajectoryOpParamPanel : public QWidget
{
	Q_OBJECT

public:
	explicit TrajectoryOpParamPanel(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setLoading(bool loading);
	void setScopeGroupCombo(QComboBox* combo);
	void setGeometryBackendCombo(QComboBox* combo);
	void setNonRigidSourceBackendCombo(QComboBox* combo);
	void setNonRigidTargetBackendCombo(QComboBox* combo);
	void setExternalTcpBackendCombo(QComboBox* combo);

	void rebuildForOp(const RobotInstruction::TrajectoryOpDescriptor& op, const trajectory_algo::ITrajectoryOp* algo);
	bool applyTo(RobotInstruction::TrajectoryOpDescriptor& op, const trajectory_algo::ITrajectoryOp* algo,
				 std::string* errMsg);

	/// 离散点云规模；用于 P 起/止上限与切到 P 范围时默认填满
	void setPointIndexLimit(int pointCount);
	void setEditingRawCloud(bool editingRaw);

	void clear();

	bool isRebuilding() const { return m_rebuilding; }

signals:
	void paramsChanged();

private:
	void clearRows();
	void updateFieldVisibility();
	void applyPointIndexLimitToSpins();
	void fillPointRangeIfNeeded();
	void updateScopeHint();
	std::string currentScopeKindToken() const;
	int currentIntFieldValue(const std::string& key) const;

	bool m_useChinese = true;
	bool m_loading = false;
	bool m_clearingRows = false;
	bool m_rebuilding = false;
	bool m_editingRawCloud = false;
	int m_pointIndexLimit = 0;
	QLabel* m_scopeHintLabel = nullptr;
	QComboBox* m_scopeGroupCombo = nullptr;
	QWidget* m_scopeGroupComboParent = nullptr;
	QComboBox* m_geometryBackendCombo = nullptr;
	QWidget* m_geometryBackendComboParent = nullptr;
	QComboBox* m_nonRigidSourceCombo = nullptr;
	QWidget* m_nonRigidSourceComboParent = nullptr;
	QComboBox* m_nonRigidTargetCombo = nullptr;
	QWidget* m_nonRigidTargetComboParent = nullptr;
	QComboBox* m_externalTcpBackendCombo = nullptr;
	QWidget* m_externalTcpBackendComboParent = nullptr;
	QFormLayout* m_form = nullptr;
	std::vector<trajectory_algo::TrajectoryParamBinding> m_rows;
};

#endif // ROBOTWIDGET_TRAJECTORYOPPARAMPANEL_H
