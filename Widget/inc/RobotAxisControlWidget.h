#pragma once

#include <QVector>
#include <QWidget>

#include "widget_global.h"

class QLabel;
class QScrollArea;
class QSlider;
class QVBoxLayout;

/// Per-revolute-joint sliders: range from URDF &lt;limit&gt; (or defaults). Emits radians in FK order.
class WIDGET_EXPORT RobotAxisControlWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RobotAxisControlWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void clearJoints();
	void setJoints(const QStringList& jointNames, const QVector<double>& lowerRad, const QVector<double>& upperRad);
	void setJointAnglesRad(const QVector<double>& rad);
	QVector<double> jointAnglesRad() const;
	int jointCount() const { return m_anglesRad.size(); }
	void setInteractionEnabled(bool enabled);

signals:
	void jointAnglesChanged(const QVector<double>& rad);

private:
	void onSliderValueChanged(int which);
	void emitAnglesFromSliders();
	double sliderToRad(int index, int sliderValue) const;
	int radToSliderValue(int index, double rad) const;

	bool m_useChinese = false;
	QVector<double> m_lowerRad;
	QVector<double> m_upperRad;
	QVector<double> m_anglesRad;
	QVector<QSlider*> m_sliders;
	QVector<QLabel*> m_valueLabels;
	QLabel* m_hintLabel = nullptr;
	QScrollArea* m_scroll = nullptr;
	QVBoxLayout* m_rowsLayout = nullptr;
};
