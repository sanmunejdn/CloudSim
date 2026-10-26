#ifndef ROBOTWIDGET_ROBOTAXISJOGCONTROLLERDIALOG_H
#define ROBOTWIDGET_ROBOTAXISJOGCONTROLLERDIALOG_H

/// @file RobotAxisJogControllerDialog.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 轴控点动：速率点动（mm/s、°/s），对齐真实示教盒

#include "robotwidget_global.h"

#include <QDialog>
#include <QStringList>
#include <QVector>
#include <functional>

class QComboBox;
class QDoubleSpinBox;
class QFrame;
class QLabel;
class QPushButton;
class QTabWidget;
class QTimer;
class QVBoxLayout;
class QWidget;

/// TCP 点动参考系（增量在该系下解释）
enum class RobotAxisJogTcpFrame : int
{
	Base = 0, ///< 机器人基座
	Tool = 1, ///< TCP / 工具物体系
	World = 2, ///< 场景世界系
	User = 3, ///< 活动用户坐标系
};

class ROBOTWIDGET_EXPORT RobotAxisJogControllerDialog : public QDialog
{
	Q_OBJECT

public:
	explicit RobotAxisJogControllerDialog(QWidget* parent = nullptr);
	~RobotAxisJogControllerDialog() override;

	void setUseChinese(bool chinese);
	void setJointNames(const QStringList& names);
	RobotAxisJogTcpFrame tcpFrame() const;

signals:
	void jogTcpDeltaRequested(double dxMm, double dyMm, double dzMm, double dExDeg, double dEyDeg, double dEzDeg,
							  RobotAxisJogTcpFrame frame);
	void jogJointDeltaRequested(int jointIndex, double deltaRad);

private:
	enum class AxisTone
	{
		X,
		Y,
		Z,
		Neutral,
	};

	static const char* toneKey(AxisTone tone);

	void rebuildAxisButtons();
	void retranslate();
	void applyDialogChrome();
	void emitTcpDelta(double dxMm, double dyMm, double dzMm, double dExDeg, double dEyDeg, double dEzDeg);
	void emitJointDelta(int jointIndex, double signDegPerSec, double dtSec);
	void bindJogHold(QPushButton* btn, std::function<void(double dtSec)> emitDt);
	void stopJogHold();
	double jogTickDtSec() const;
	QPushButton* makeJogButton(const QString& text, QWidget* parent, AxisTone tone = AxisTone::Neutral);
	QFrame* makeSectionCard(QWidget* parent, QLabel** outTitle);
	QWidget* makeAxisJogRow(QWidget* parent, const QString& axisLabel, AxisTone tone, QPushButton** outMinus,
							QPushButton** outPlus);

	bool m_useChinese = true;
	QStringList m_jointNames;
	QTabWidget* m_tabs = nullptr;
	QWidget* m_tcpPage = nullptr;
	QWidget* m_axisPage = nullptr;
	QVBoxLayout* m_axisLayout = nullptr;
	QFrame* m_settingsCard = nullptr;
	QLabel* m_frameLabel = nullptr;
	QComboBox* m_frameCombo = nullptr;
	QLabel* m_transRateLabel = nullptr;
	QLabel* m_rotRateLabel = nullptr;
	QLabel* m_jointRateLabel = nullptr;
	QLabel* m_transSectionTitle = nullptr;
	QLabel* m_rotSectionTitle = nullptr;
	QComboBox* m_transRateCombo = nullptr;
	QDoubleSpinBox* m_rotRateSpin = nullptr;
	QDoubleSpinBox* m_jointRateSpin = nullptr;
	QVector<QPushButton*> m_tcpButtons;
	QTimer* m_jogHoldRepeatTimer = nullptr;
	std::function<void(double)> m_jogHoldEmit;
};

#endif // ROBOTWIDGET_ROBOTAXISJOGCONTROLLERDIALOG_H
