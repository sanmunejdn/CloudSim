#pragma once

#include "TrajectoryParamWidgetFactory.h"
#include "robotwidget_global.h"

#include <ITrajectoryOp.h>
#include <TrajectoryPipelineTypes.h>

#include <QWidget>

#include <string>
#include <vector>

class QComboBox;
class QFormLayout;

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

	void rebuildForOp(
		const RobotInstruction::TrajectoryOpDescriptor& op,
		const trajectory_algo::ITrajectoryOp* algo);
	bool applyTo(
		RobotInstruction::TrajectoryOpDescriptor& op,
		const trajectory_algo::ITrajectoryOp* algo,
		std::string* errMsg);

	void clear();

	bool isRebuilding() const { return m_rebuilding; }

signals:
	void paramsChanged();

private:
	void clearRows();
	void updateFieldVisibility();
	std::string currentScopeKindToken() const;
	int currentIntFieldValue(const std::string& key) const;

	bool m_useChinese = true;
	bool m_loading = false;
	bool m_clearingRows = false;
	bool m_rebuilding = false;
	QComboBox* m_scopeGroupCombo = nullptr;
	QWidget* m_scopeGroupComboParent = nullptr;
	QComboBox* m_geometryBackendCombo = nullptr;
	QWidget* m_geometryBackendComboParent = nullptr;
	QComboBox* m_nonRigidSourceCombo = nullptr;
	QWidget* m_nonRigidSourceComboParent = nullptr;
	QComboBox* m_nonRigidTargetCombo = nullptr;
	QWidget* m_nonRigidTargetComboParent = nullptr;
	QFormLayout* m_form = nullptr;
	std::vector<trajectory_algo::TrajectoryParamBinding> m_rows;
};
