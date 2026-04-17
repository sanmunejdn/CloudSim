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

#include "robot_urdf_global.h"
#include <osg/MatrixTransform>

namespace UrdfRobotLoader {

/// 【中文】机器人关节轴控制Widget
/// 显示每个轴的上下限，提供滑块和数值输入框来控制轴角度
class ROBOT_URDF_API RobotAxisControlWidget : public QWidget
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

	/// 【中文】初始化关节控制界面
	/// @param jointNames 关节名称列表（按顺序）
	/// @param lowerLimits 各关节下限（弧度）
	/// @param upperLimits 各关节上限（弧度）
	/// @param jointTransforms 关节变换节点（用于实时更新）
	void setupJointControls(
		const QStringList& jointNames,
		const QVector<double>& lowerLimits,
		const QVector<double>& upperLimits,
		const QHash<QString, osg::MatrixTransform*>& jointTransforms);

	/// 【中文】设置指定关节的角度
	/// @param jointName 关节名称
	/// @param angleRad 角度（弧度）
	void setJointAngle(const QString& jointName, double angleRad);

	/// 【中文】获取指定关节的当前角度
	/// @param jointName 关节名称
	/// @return 当前角度（弧度）
	double getJointAngle(const QString& jointName) const;

	/// 【中文】重置所有关节到零位
	void resetAllJoints();

signals:
	/// 【中文】关节角度改变信号
	/// @param jointName 关节名称
	/// @param angleRad 新角度（弧度）
	void jointAngleChanged(const QString& jointName, double angleRad);

	/// 【中文】所有关节角度改变信号（批量更新）
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
		double lowerLimit;  // 弧度
		double upperLimit;  // 弧度
		double currentAngle; // 弧度
		osg::MatrixTransform* transformNode; // OSG节点
		
		// UI控件
		QLabel* nameLabel = nullptr;
		QLabel* limitLabel = nullptr;  // 显示 "下限 ~ 上限"
		QSlider* slider = nullptr;     // 整数滑块（放大1000倍）
		QDoubleSpinBox* spinBox = nullptr; // 数值输入
		QLineEdit* inputEdit = nullptr;    // 直接输入角度
		QPushButton* resetButton = nullptr; // 重置单个轴
	};

	QHash<QString, JointControl> m_jointControls;
	QVector<QString> m_jointOrder; // 保持顺序
	
	QScrollArea* m_scrollArea = nullptr;
	QWidget* m_contentWidget = nullptr;
	QVBoxLayout* m_contentLayout = nullptr;
	QPushButton* m_resetAllButton = nullptr;

	// 滑块精度系数（将弧度转为整数）
	static constexpr double SLIDER_SCALE = 1000.0;

	void createUI();
	void updateLimitLabel(JointControl& jc);
	void updateJointTransform(const QString& jointName, double angleRad);
	void emitAllJointAnglesNow();
	int angleToSliderValue(double angleRad) const;
	double sliderValueToAngle(int value) const;
};

} // namespace UrdfRobotLoader

using RobotAxisControlWidget = UrdfRobotLoader::RobotAxisControlWidget;