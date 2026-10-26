/// @file SimulationTimelinePageWidget.cpp

#include "SimulationTimelinePageWidget.h"

#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QSlider>
#include <QUuid>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <functional>

namespace
{
constexpr int kLabelW = 110;
constexpr int kRowH = 28;

QColor trackColor(TimelineTrackType t)
{
	switch (t)
	{
	case TimelineTrackType::Device:
		return QColor(70, 140, 200);
	case TimelineTrackType::Signal:
		return QColor(200, 140, 60);
	case TimelineTrackType::Material:
		return QColor(120, 170, 90);
	case TimelineTrackType::Robot:
	default:
		return QColor(160, 90, 180);
	}
}
} // namespace

class TimelineTrackCanvas : public QWidget
{
public:
	explicit TimelineTrackCanvas(QWidget* parent = nullptr) : QWidget(parent)
	{
		setMinimumHeight(160);
		setMouseTracking(true);
	}

	void setStudy(const TimelineStudy& s)
	{
		m_study = s;
		update();
	}
	void setPlayhead(double t)
	{
		m_playhead = t;
		update();
	}
	void setReadOnly(bool v) { m_readOnly = v; }
	void setSelection(const QString& trackId, const QString& clipId)
	{
		m_selTrack = trackId;
		m_selClip = clipId;
		update();
	}

	std::function<void(const QString&, const QString&, double, double)> onClipMoved;
	std::function<void(const QString&, const QString&)> onSelectionChanged;
	std::function<void(double)> onPlayheadClicked;

protected:
	void paintEvent(QPaintEvent*) override
	{
		QPainter p(this);
		p.fillRect(rect(), QColor(36, 38, 42));
		const double dur = std::max(0.1, m_study.durationSec);
		const int chartW = std::max(1, width() - kLabelW - 8);
		const auto xOf = [&](double t) { return kLabelW + static_cast<int>(t / dur * chartW); };

		p.setPen(QColor(90, 90, 95));
		for (int i = 0; i <= 10; ++i)
		{
			const double t = dur * i / 10.0;
			const int x = xOf(t);
			p.drawLine(x, 0, x, height());
			p.drawText(x + 2, 12, QString::number(t, 'f', 1));
		}

		int row = 0;
		for (const TimelineTrack& tr : m_study.tracks)
		{
			const int y = 18 + row * kRowH;
			p.setPen(QColor(210, 210, 210));
			p.drawText(4, y + 18, tr.name.isEmpty() ? tr.id.left(8) : tr.name);
			const QColor col = trackColor(tr.type);
			for (const TimelineClip& c : tr.clips)
			{
				int x0 = xOf(c.tStart);
				int x1 = xOf(std::max(c.tEnd, c.tStart + 0.05));
				if (tr.type == TimelineTrackType::Material)
				{
					x0 = xOf(c.tStart) - 4;
					x1 = x0 + 8;
				}
				QRect r(x0, y + 4, std::max(4, x1 - x0), kRowH - 8);
				const bool sel = (tr.id == m_selTrack && c.id == m_selClip);
				p.fillRect(r, sel ? col.lighter(120) : col);
				p.setPen(sel ? Qt::white : col.darker(140));
				p.drawRect(r);
			}
			++row;
		}

		const int px = xOf(m_playhead);
		p.setPen(QPen(QColor(255, 80, 80), 2));
		p.drawLine(px, 0, px, height());
	}

	void mousePressEvent(QMouseEvent* e) override
	{
		const double dur = std::max(0.1, m_study.durationSec);
		const int chartW = std::max(1, width() - kLabelW - 8);
		if (e->pos().x() >= kLabelW && onPlayheadClicked)
		{
			const double t = (e->pos().x() - kLabelW) * dur / chartW;
			onPlayheadClicked(std::clamp(t, 0.0, dur));
		}
		int row = 0;
		for (const TimelineTrack& tr : m_study.tracks)
		{
			const int y = 18 + row * kRowH;
			for (const TimelineClip& c : tr.clips)
			{
				int x0 = kLabelW + static_cast<int>(c.tStart / dur * chartW);
				int x1 = kLabelW + static_cast<int>(std::max(c.tEnd, c.tStart + 0.05) / dur * chartW);
				if (tr.type == TimelineTrackType::Material)
				{
					x0 = kLabelW + static_cast<int>(c.tStart / dur * chartW) - 4;
					x1 = x0 + 8;
				}
				QRect r(x0, y + 4, std::max(4, x1 - x0), kRowH - 8);
				if (r.contains(e->pos()))
				{
					m_selTrack = tr.id;
					m_selClip = c.id;
					m_drag = !m_readOnly;
					m_dragTrack = tr.id;
					m_dragClip = c.id;
					m_dragDur = std::max(0.05, c.tEnd - c.tStart);
					m_pressT = (e->pos().x() - kLabelW) * dur / chartW;
					m_origStart = c.tStart;
					if (onSelectionChanged)
					{
						onSelectionChanged(tr.id, c.id);
					}
					update();
					return;
				}
			}
			++row;
		}
	}

	void mouseMoveEvent(QMouseEvent* e) override
	{
		if (!m_drag || !onClipMoved)
		{
			return;
		}
		const double dur = std::max(0.1, m_study.durationSec);
		const int chartW = std::max(1, width() - kLabelW - 8);
		const double t = (e->pos().x() - kLabelW) * dur / chartW;
		const double dt = t - m_pressT;
		double ns = std::clamp(m_origStart + dt, 0.0, dur);
		double ne = ns + m_dragDur;
		if (ne > dur)
		{
			ne = dur;
			ns = std::max(0.0, ne - m_dragDur);
		}
		onClipMoved(m_dragTrack, m_dragClip, ns, ne);
	}

	void mouseReleaseEvent(QMouseEvent*) override { m_drag = false; }

private:
	TimelineStudy m_study;
	double m_playhead = 0;
	bool m_readOnly = false;
	QString m_selTrack;
	QString m_selClip;
	bool m_drag = false;
	QString m_dragTrack;
	QString m_dragClip;
	double m_dragDur = 1;
	double m_pressT = 0;
	double m_origStart = 0;
};

SimulationTimelinePageWidget::SimulationTimelinePageWidget(QWidget* parent) : QWidget(parent)
{
	m_study.durationSec = 30.0;
	m_study.sampleHz = 20.0;

	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(4);

	auto* row1 = new QHBoxLayout;
	m_runBtn = new QPushButton(this);
	m_stopBtn = new QPushButton(this);
	m_durationSpin = new QDoubleSpinBox(this);
	m_durationSpin->setRange(1.0, 600.0);
	m_durationSpin->setDecimals(2);
	m_durationSpin->setValue(m_study.durationSec);
	m_timeLabel = new QLabel(this);
	m_playheadSlider = new QSlider(Qt::Horizontal, this);
	m_playheadSlider->setRange(0, 1000);
	row1->addWidget(m_runBtn);
	row1->addWidget(m_stopBtn);
	row1->addWidget(new QLabel(QStringLiteral("Dur"), this));
	row1->addWidget(m_durationSpin);
	row1->addWidget(m_timeLabel, 1);
	root->addLayout(row1);
	root->addWidget(m_playheadSlider);

	auto* row2 = new QHBoxLayout;
	m_addRobotBtn = new QPushButton(this);
	m_addDeviceBtn = new QPushButton(this);
	m_addSignalBtn = new QPushButton(this);
	m_addMaterialBtn = new QPushButton(this);
	m_addClipBtn = new QPushButton(this);
	m_removeBtn = new QPushButton(this);
	row2->addWidget(m_addRobotBtn);
	row2->addWidget(m_addDeviceBtn);
	row2->addWidget(m_addSignalBtn);
	row2->addWidget(m_addMaterialBtn);
	row2->addWidget(m_addClipBtn);
	row2->addWidget(m_removeBtn);
	row2->addStretch(1);
	root->addLayout(row2);

	m_canvas = new TimelineTrackCanvas(this);
	root->addWidget(m_canvas, 1);
	m_hintLabel = new QLabel(this);
	m_hintLabel->setWordWrap(true);
	root->addWidget(m_hintLabel);

	connect(m_runBtn, &QPushButton::clicked, this, &SimulationTimelinePageWidget::runRequested);
	connect(m_stopBtn, &QPushButton::clicked, this, &SimulationTimelinePageWidget::stopRequested);
	connect(m_addRobotBtn, &QPushButton::clicked, this, &SimulationTimelinePageWidget::onAddRobotTrack);
	connect(m_addDeviceBtn, &QPushButton::clicked, this, &SimulationTimelinePageWidget::onAddDeviceTrack);
	connect(m_addSignalBtn, &QPushButton::clicked, this, &SimulationTimelinePageWidget::onAddSignalTrack);
	connect(m_addMaterialBtn, &QPushButton::clicked, this, &SimulationTimelinePageWidget::onAddMaterialTrack);
	connect(m_addClipBtn, &QPushButton::clicked, this, &SimulationTimelinePageWidget::onAddClip);
	connect(m_removeBtn, &QPushButton::clicked, this, &SimulationTimelinePageWidget::onRemoveSelected);
	connect(m_durationSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&SimulationTimelinePageWidget::onDurationChanged);
	connect(m_playheadSlider, &QSlider::valueChanged, this, &SimulationTimelinePageWidget::onPlayheadSlider);

	m_canvas->onClipMoved = [this](const QString& trackId, const QString& clipId, double tStart, double tEnd)
	{ onCanvasClipMoved(trackId, clipId, tStart, tEnd); };
	m_canvas->onSelectionChanged = [this](const QString& trackId, const QString& clipId)
	{ onCanvasSelectionChanged(trackId, clipId); };
	m_canvas->onPlayheadClicked = [this](double t)
	{
		setPlayheadSec(t);
		emit playheadScrubbed(t);
	};

	retranslate();
	rebuildCanvas();
	syncPlayheadUi();
}

void SimulationTimelinePageWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	retranslate();
}

void SimulationTimelinePageWidget::setStudy(const TimelineStudy& study)
{
	m_study = study;
	if (m_durationSpin)
	{
		const QSignalBlocker b(m_durationSpin);
		m_durationSpin->setValue(m_study.durationSec);
	}
	rebuildCanvas();
	syncPlayheadUi();
}

void SimulationTimelinePageWidget::setPlayheadSec(double t)
{
	m_playheadSec = std::clamp(t, 0.0, std::max(0.1, m_study.durationSec));
	syncPlayheadUi();
	if (m_canvas)
	{
		m_canvas->setPlayhead(m_playheadSec);
	}
}

void SimulationTimelinePageWidget::setReadOnly(bool readOnly)
{
	m_readOnly = readOnly;
	for (QWidget* w : {static_cast<QWidget*>(m_addRobotBtn), static_cast<QWidget*>(m_addDeviceBtn),
					   static_cast<QWidget*>(m_addSignalBtn), static_cast<QWidget*>(m_addMaterialBtn),
					   static_cast<QWidget*>(m_addClipBtn), static_cast<QWidget*>(m_removeBtn),
					   static_cast<QWidget*>(m_durationSpin), static_cast<QWidget*>(m_runBtn)})
	{
		if (w)
		{
			w->setEnabled(!readOnly);
		}
	}
	if (m_canvas)
	{
		m_canvas->setReadOnly(readOnly);
	}
}

void SimulationTimelinePageWidget::setProgramDurationEstimator(
	std::function<double(int instanceIndex, const QString& programId)> fn)
{
	m_programDurationEstimator = std::move(fn);
}

void SimulationTimelinePageWidget::setStatusHint(const QString& text)
{
	if (m_hintLabel)
	{
		m_hintLabel->setText(text);
	}
}

void SimulationTimelinePageWidget::rebuildCanvas()
{
	if (m_canvas)
	{
		m_canvas->setStudy(m_study);
		m_canvas->setPlayhead(m_playheadSec);
		m_canvas->setSelection(m_selTrackId, m_selClipId);
		m_canvas->setMinimumHeight(std::max(160, 24 + m_study.tracks.size() * kRowH));
	}
}

void SimulationTimelinePageWidget::emitStudy()
{
	QString err;
	if (!timelineStudyValidate(m_study, &err))
	{
		setStatusHint(err);
		rebuildCanvas();
		return;
	}
	emit studyEdited(m_study);
	rebuildCanvas();
}

void SimulationTimelinePageWidget::syncPlayheadUi()
{
	const double dur = std::max(0.1, m_study.durationSec);
	if (m_playheadSlider)
	{
		const QSignalBlocker b(m_playheadSlider);
		m_playheadSlider->setValue(static_cast<int>(std::lround(m_playheadSec / dur * 1000.0)));
	}
	if (m_timeLabel)
	{
		m_timeLabel->setText(QStringLiteral("t=%1 / %2")
								 .arg(m_playheadSec, 0, 'f', 2)
								 .arg(m_study.durationSec, 0, 'f', 2));
	}
}

void SimulationTimelinePageWidget::retranslate()
{
	const bool zh = m_useChinese;
	if (m_runBtn)
	{
		m_runBtn->setText(zh ? QStringLiteral("运行") : QStringLiteral("Run"));
	}
	if (m_stopBtn)
	{
		m_stopBtn->setText(zh ? QStringLiteral("停止") : QStringLiteral("Stop"));
	}
	if (m_addRobotBtn)
	{
		m_addRobotBtn->setText(zh ? QStringLiteral("+机器人轨") : QStringLiteral("+Robot"));
	}
	if (m_addDeviceBtn)
	{
		m_addDeviceBtn->setText(zh ? QStringLiteral("+设备轨") : QStringLiteral("+Device"));
	}
	if (m_addSignalBtn)
	{
		m_addSignalBtn->setText(zh ? QStringLiteral("+信号轨") : QStringLiteral("+Signal"));
	}
	if (m_addMaterialBtn)
	{
		m_addMaterialBtn->setText(zh ? QStringLiteral("+物料轨") : QStringLiteral("+Material"));
	}
	if (m_addClipBtn)
	{
		m_addClipBtn->setText(zh ? QStringLiteral("加片段") : QStringLiteral("Add clip"));
	}
	if (m_removeBtn)
	{
		m_removeBtn->setText(zh ? QStringLiteral("删除") : QStringLiteral("Remove"));
	}
}

void SimulationTimelinePageWidget::onAddRobotTrack()
{
	if (m_readOnly)
	{
		return;
	}
	TimelineTrack tr;
	tr.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
	tr.type = TimelineTrackType::Robot;
	tr.name = QStringLiteral("R%1").arg(m_study.tracks.size());
	tr.instanceIndex = 0;
	m_study.tracks.push_back(tr);
	m_selTrackId = tr.id;
	emitStudy();
}

void SimulationTimelinePageWidget::onAddDeviceTrack()
{
	if (m_readOnly)
	{
		return;
	}
	bool ok = false;
	const QString owner = QInputDialog::getText(this, QStringLiteral("Device track"), QStringLiteral("ownerId"),
												QLineEdit::Normal, QString(), &ok);
	if (!ok || owner.isEmpty())
	{
		return;
	}
	TimelineTrack tr;
	tr.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
	tr.type = TimelineTrackType::Device;
	tr.name = owner.left(12);
	tr.ownerId = owner;
	m_study.tracks.push_back(tr);
	m_selTrackId = tr.id;
	emitStudy();
}

void SimulationTimelinePageWidget::onAddSignalTrack()
{
	if (m_readOnly)
	{
		return;
	}
	bool ok = false;
	const QString owner = QInputDialog::getText(this, QStringLiteral("Signal track"), QStringLiteral("ownerId"),
												QLineEdit::Normal, QString(), &ok);
	if (!ok || owner.isEmpty())
	{
		return;
	}
	const QString signal = QInputDialog::getText(this, QStringLiteral("Signal track"), QStringLiteral("signal"),
												 QLineEdit::Normal, QStringLiteral("Start"), &ok);
	if (!ok || signal.isEmpty())
	{
		return;
	}
	TimelineTrack tr;
	tr.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
	tr.type = TimelineTrackType::Signal;
	tr.name = signal;
	tr.ownerId = owner;
	tr.signal = signal;
	tr.signalDirection = QStringLiteral("out");
	m_study.tracks.push_back(tr);
	m_selTrackId = tr.id;
	emitStudy();
}

void SimulationTimelinePageWidget::onAddMaterialTrack()
{
	if (m_readOnly)
	{
		return;
	}
	bool ok = false;
	const QString objectId = QInputDialog::getText(this, QStringLiteral("Material track"), QStringLiteral("objectId"),
												   QLineEdit::Normal, QString(), &ok);
	if (!ok || objectId.isEmpty())
	{
		return;
	}
	TimelineTrack tr;
	tr.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
	tr.type = TimelineTrackType::Material;
	tr.name = objectId.left(12);
	tr.objectId = objectId;
	m_study.tracks.push_back(tr);
	m_selTrackId = tr.id;
	emitStudy();
}

void SimulationTimelinePageWidget::onAddClip()
{
	if (m_readOnly || m_selTrackId.isEmpty())
	{
		setStatusHint(m_useChinese ? QStringLiteral("先选中一条轨") : QStringLiteral("Select a track first"));
		return;
	}
	for (TimelineTrack& tr : m_study.tracks)
	{
		if (tr.id != m_selTrackId)
		{
			continue;
		}
		TimelineClip c;
		c.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
		c.tStart = m_playheadSec;
		if (tr.type == TimelineTrackType::Material)
		{
			c.tEnd = c.tStart;
			c.visible = true;
		}
		else if (tr.type == TimelineTrackType::Signal)
		{
			c.tEnd = c.tStart + 0.5;
			c.signalValue = true;
		}
		else if (tr.type == TimelineTrackType::Device)
		{
			bool ok = false;
			c.poseId = QInputDialog::getText(this, QStringLiteral("Clip"), QStringLiteral("poseId"), QLineEdit::Normal,
											 QStringLiteral("open"), &ok);
			if (!ok)
			{
				return;
			}
			c.tEnd = c.tStart + 1.5;
			c.durationSec = 1.5;
		}
		else
		{
			bool ok = false;
			c.programId = QInputDialog::getText(
				this, QStringLiteral("Clip"),
				QStringLiteral("program id or name (same as Instructions combo)"), QLineEdit::Normal, QString(), &ok);
			if (!ok)
			{
				return;
			}
			double clipDur = 3.0;
			if (m_programDurationEstimator)
			{
				const double est = m_programDurationEstimator(tr.instanceIndex, c.programId);
				if (est > 1e-3)
				{
					clipDur = est;
				}
			}
			c.tEnd = c.tStart + clipDur;
			setStatusHint(m_useChinese
							  ? QStringLiteral("片段时长按规划估算 %.2f s（可拖边修改）").arg(clipDur)
							  : QStringLiteral("Clip length ≈ %.2f s from plan (drag edges to edit)").arg(clipDur));
		}
		tr.clips.push_back(c);
		m_selClipId = c.id;
		emitStudy();
		return;
	}
}

void SimulationTimelinePageWidget::onRemoveSelected()
{
	if (m_readOnly)
	{
		return;
	}
	if (!m_selClipId.isEmpty())
	{
		for (TimelineTrack& tr : m_study.tracks)
		{
			if (tr.id != m_selTrackId)
			{
				continue;
			}
			for (int i = 0; i < tr.clips.size(); ++i)
			{
				if (tr.clips[i].id == m_selClipId)
				{
					tr.clips.removeAt(i);
					m_selClipId.clear();
					emitStudy();
					return;
				}
			}
		}
	}
	if (!m_selTrackId.isEmpty())
	{
		for (int i = 0; i < m_study.tracks.size(); ++i)
		{
			if (m_study.tracks[i].id == m_selTrackId)
			{
				m_study.tracks.removeAt(i);
				m_selTrackId.clear();
				m_selClipId.clear();
				emitStudy();
				return;
			}
		}
	}
}

void SimulationTimelinePageWidget::onDurationChanged(double v)
{
	if (m_readOnly)
	{
		return;
	}
	m_study.durationSec = v;
	emitStudy();
	syncPlayheadUi();
}

void SimulationTimelinePageWidget::onPlayheadSlider(int v)
{
	const double dur = std::max(0.1, m_study.durationSec);
	m_playheadSec = dur * v / 1000.0;
	if (m_canvas)
	{
		m_canvas->setPlayhead(m_playheadSec);
	}
	syncPlayheadUi();
	emit playheadScrubbed(m_playheadSec);
}

void SimulationTimelinePageWidget::onCanvasClipMoved(const QString& trackId, const QString& clipId, double tStart,
													 double tEnd)
{
	if (m_readOnly)
	{
		return;
	}
	for (TimelineTrack& tr : m_study.tracks)
	{
		if (tr.id != trackId)
		{
			continue;
		}
		for (TimelineClip& c : tr.clips)
		{
			if (c.id != clipId)
			{
				continue;
			}
			c.tStart = tStart;
			c.tEnd = (tr.type == TimelineTrackType::Material) ? tStart : tEnd;
			emitStudy();
			return;
		}
	}
}

void SimulationTimelinePageWidget::onCanvasSelectionChanged(const QString& trackId, const QString& clipId)
{
	m_selTrackId = trackId;
	m_selClipId = clipId;
}
