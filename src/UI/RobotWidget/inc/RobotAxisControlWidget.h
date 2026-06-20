#pragma once

#include <QWidget>
#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QScrollArea>

#include "robotwidget_global.h"
#include <osg/MatrixTransform>

/// 机器人关节轴控制
/// 各轴上下限与滑块/数值输入
class ROBOTWIDGET_EXPORT RobotAxisControlWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RobotAxisControlWidget(QWidget* parent = nullptr);
	~RobotAxisControlWidget();
	void setUseChinese(bool chinese);
	void setInteractionEnabled(bool enabled);
	void setJoints(const QStringList& jointNames, const QVector<double>& lowerLimits, const QVector<double>& upperLimits);
	void clearJoints();
	int jointCount() const;
	QVector<double> jointAnglesRad() const;
	void setJointAnglesRad(const QVector<double>& jointAnglesRad);
	/// 仅刷新 UI，不发 allJointAnglesChanged（末端 IK 回写）
	void setJointAnglesRadSilent(const QVector<double>& jointAnglesRad);

	/// 初始化关节控制界面
	/// @param jointNames 关节名称列表（按顺序）
	/// @param lowerLimits 各关节下限（弧度）
	/// @param upperLimits 各关节上限（弧度）
	/// @param jointTransforms 关节 OSG 变换节点
	void setupJointControls(
		const QStringList& jointNames,
		const QVector<double>& lowerLimits,
		const QVector<double>& upperLimits,
		const QHash<QString, osg::MatrixTransform*>& jointTransforms);

	/// 设置指定关节的角度
	/// @param jointName 关节名称
	/// @param angleRad 角度（弧度）
	void setJointAngle(const QString& jointName, double angleRad);

	/// 获取指定关节的当前角度
	/// @param jointName 关节名称
	/// @return 当前角度（弧度）
	double getJointAngle(const QString& jointName) const;

	/// 重置所有关节到零位
	void resetAllJoints();

signals:
	/// 单关节角度变更
	/// @param jointName 关节名称
	/// @param angleRad 新角度（弧度）
	void jointAngleChanged(const QString& jointName, double angleRad);

	/// 所有单关节角度变更（批量更新）
	/// @param angles 各关节角度（弧度）
	void allJointAnglesChanged(const QVector<double>& angles);

private slots:
	void onSliderValueChanged(int value);
	void onSpinBoxValueChanged(double value);
	void onLineEditReturnPressed();
	void onResetButtonClicked();
	void onResetAllButtonClicked();

private:
	struct JointControl {
		QString name;
		double lowerLimit;
		double upperLimit;
		double currentAngle;
		osg::MatrixTransform* transformNode;
		
		QSlider* slider = nullptr;
		QDoubleSpinBox* spinBox = nullptr;
		QLineEdit* inputEdit = nullptr;
		QPushButton* resetButton = nullptr;
	};

	QHash<QString, JointControl> m_jointControls;
	QVector<QString> m_jointOrder;
	
	QScrollArea* m_scrollArea = nullptr;
	QWidget* m_contentWidget = nullptr;
	QVBoxLayout* m_contentLayout = nullptr;
	QPushButton* m_resetAllButton = nullptr;

/// 滑块精度：弧度×1000 映射整数
	static constexpr double SLIDER_SCALE = 1000.0;

	void createUI();
	void updateJointTransform(const QString& jointName, double angleRad);
	void emitAllJointAnglesNow();
	int angleToSliderValue(double angleRad) const;
	double sliderValueToAngle(int value) const;
};