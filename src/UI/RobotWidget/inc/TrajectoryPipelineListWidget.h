#ifndef ROBOTWIDGET_TRAJECTORYPIPELINELISTWIDGET_H
#define ROBOTWIDGET_TRAJECTORYPIPELINELISTWIDGET_H

/// @file TrajectoryPipelineListWidget.h
/// @brief 轨迹操作流水线列表，支持拖放排序；行末「启用」勾选控制是否参与引擎

#include "robotwidget_global.h"

#include "TrajectoryPipelineTypes.h"

#include <QListWidget>
#include <functional>
#include <vector>

/// 轨迹操作流水线列表，支持拖放排序；行末「启用」勾选控制是否参与引擎
class ROBOTWIDGET_EXPORT TrajectoryPipelineListWidget : public QListWidget
{
	Q_OBJECT

public:
	static constexpr const char* kMimeType = "application/x-cloudsim-trajectory-op";

	explicit TrajectoryPipelineListWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setOps(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops);
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops() const;

	int selectedOpIndex() const;
	RobotInstruction::TrajectoryOpDescriptor selectedOp() const;
	RobotInstruction::TrajectoryOpDescriptor opAt(int index) const;

	void appendOp(RobotInstruction::TrajectoryOpDescriptor op);
	void removeSelectedOp();
	void moveSelectedOp(int delta);
	void updateSelectedOp(const RobotInstruction::TrajectoryOpDescriptor& op);
	void updateOpAt(int index, const RobotInstruction::TrajectoryOpDescriptor& op);

	using DefaultOpFactory =
		std::function<RobotInstruction::TrajectoryOpDescriptor(RobotInstruction::TrajectoryOpKind)>;
	void setDefaultOpFactory(DefaultOpFactory factory);

signals:
	void opsChanged();
	void selectedOpChanged(int index);

protected:
	void dragEnterEvent(QDragEnterEvent* event) override;
	void dragMoveEvent(QDragMoveEvent* event) override;
	void dropEvent(QDropEvent* event) override;
	void startDrag(Qt::DropActions supportedActions) override;

private:
	void rebuildItems();
	void refreshRowWidget(int index);
	QWidget* createRowWidget(int index, const RobotInstruction::TrajectoryOpDescriptor& op);
	void onEnableCheckToggled(bool checked);
	QString formatOpSummary(const RobotInstruction::TrajectoryOpDescriptor& op) const;
	RobotInstruction::TrajectoryOpDescriptor opFromMime(const QMimeData* mime) const;
	QByteArray mimeFromOp(const RobotInstruction::TrajectoryOpDescriptor& op) const;

	bool m_useChinese = true;
	DefaultOpFactory m_defaultOpFactory;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> m_ops;
};

#endif // ROBOTWIDGET_TRAJECTORYPIPELINELISTWIDGET_H
