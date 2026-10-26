#ifndef ROBOTWIDGET_SIMULATIONTIMELINEPAGEWIDGET_H
#define ROBOTWIDGET_SIMULATIONTIMELINEPAGEWIDGET_H

/// @file SimulationTimelinePageWidget.h
/// @brief Study 多轨时间轴编辑与 scrub UI

#include "robotwidget_global.h"

#include "TimelineStudyTypes.h"

#include <QWidget>

#include <functional>

class QLabel;
class QPushButton;
class QDoubleSpinBox;
class QSlider;
class TimelineTrackCanvas;

class ROBOTWIDGET_EXPORT SimulationTimelinePageWidget : public QWidget
{
	Q_OBJECT

public:
	explicit SimulationTimelinePageWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setStudy(const TimelineStudy& study);
	TimelineStudy study() const { return m_study; }
	void setPlayheadSec(double t);
	double playheadSec() const { return m_playheadSec; }
	void setReadOnly(bool readOnly);
	bool isReadOnly() const { return m_readOnly; }
	void setStatusHint(const QString& text);
	/// 机器人轨加片段：按 programId 估算默认时长（秒）
	void setProgramDurationEstimator(std::function<double(int instanceIndex, const QString& programId)> fn);

signals:
	void runRequested();
	void stopRequested();
	void studyEdited(const TimelineStudy& study);
	void playheadScrubbed(double tSec);

private slots:
	void onAddRobotTrack();
	void onAddDeviceTrack();
	void onAddSignalTrack();
	void onAddMaterialTrack();
	void onAddClip();
	void onRemoveSelected();
	void onDurationChanged(double v);
	void onPlayheadSlider(int v);
	void onCanvasClipMoved(const QString& trackId, const QString& clipId, double tStart, double tEnd);
	void onCanvasSelectionChanged(const QString& trackId, const QString& clipId);

private:
	void rebuildCanvas();
	void emitStudy();
	void syncPlayheadUi();
	void retranslate();

	TimelineStudy m_study;
	double m_playheadSec = 0;
	bool m_readOnly = false;
	bool m_useChinese = false;
	QString m_selTrackId;
	QString m_selClipId;
	std::function<double(int, const QString&)> m_programDurationEstimator;

	QPushButton* m_runBtn = nullptr;
	QPushButton* m_stopBtn = nullptr;
	QPushButton* m_addRobotBtn = nullptr;
	QPushButton* m_addDeviceBtn = nullptr;
	QPushButton* m_addSignalBtn = nullptr;
	QPushButton* m_addMaterialBtn = nullptr;
	QPushButton* m_addClipBtn = nullptr;
	QPushButton* m_removeBtn = nullptr;
	QDoubleSpinBox* m_durationSpin = nullptr;
	QSlider* m_playheadSlider = nullptr;
	QLabel* m_timeLabel = nullptr;
	QLabel* m_hintLabel = nullptr;
	TimelineTrackCanvas* m_canvas = nullptr;
};

#endif // ROBOTWIDGET_SIMULATIONTIMELINEPAGEWIDGET_H
