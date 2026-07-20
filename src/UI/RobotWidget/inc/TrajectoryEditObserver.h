#ifndef ROBOTWIDGET_TRAJECTORYEDITOBSERVER_H
#define ROBOTWIDGET_TRAJECTORYEDITOBSERVER_H

/// @file TrajectoryEditObserver.h
/// @brief 轨迹编辑 Observer：收敛 Page 对 Session/管道的调用

#include "robotwidget_global.h"

#include "TrajectoryPipelineTypes.h"

#include <QObject>
#include <cstddef>
#include <vector>

class TrajectoryEditSession;
class ProgramEditService;

/// 轨迹编辑 Observer：收敛 Page 对 Session/管道的调用
class ROBOTWIDGET_EXPORT TrajectoryEditObserver : public QObject
{
	Q_OBJECT

public:
	explicit TrajectoryEditObserver(QObject* parent = nullptr);

	void bindSession(TrajectoryEditSession* session);
	void bindEditService(ProgramEditService* service);

	void loadPipeline(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops);
	void updateNodeParams(std::size_t nodeIndex, const RobotInstruction::TrajectoryOpDescriptor& descriptor);
	void moveNodeUp(std::size_t nodeIndex);
	void moveNodeDown(std::size_t nodeIndex);
	void removeNode(std::size_t nodeIndex);

	bool preview(QString* outError = nullptr);
	bool apply(QString* outError = nullptr);

	TrajectoryEditSession* session() const { return m_session; }

signals:
	void pipelineStructureChanged();
	void previewRequested();

private:
	bool syncDraftOps(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops);
	bool runPreviewIfRaw(QString* outError);

	TrajectoryEditSession* m_session = nullptr;
	ProgramEditService* m_editService = nullptr;
	std::vector<RobotInstruction::TrajectoryOpDescriptor> m_draftOps;
};

#endif // ROBOTWIDGET_TRAJECTORYEDITOBSERVER_H
