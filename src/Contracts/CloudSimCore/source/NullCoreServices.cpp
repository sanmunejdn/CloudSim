#include "NullCoreServices.h"

#include "CoreTypes.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "IRobotService.h"

#include <QWidget>

namespace cloudsim::core {
namespace {

class NullDataService final : public IDataService
{
public:
	bool isValid(const ObjectId& id) const override { return !id.isEmpty(); }
	void clear() override {}

	ObjectId registerObject(const RegisterObjectDto&, QString* outError) override
	{
		if (outError)
			*outError = QStringLiteral("NullDataService");
		return {};
	}
	bool unregisterSubtree(const ObjectId&, QString* outError) override
	{
		if (outError)
			*outError = QStringLiteral("NullDataService");
		return false;
	}

	ObjectId findByName(const QString&) const override { return {}; }
	QString className(const ObjectId&) const override { return {}; }
	QString displayName(const ObjectId&) const override { return {}; }
	QVector<ObjectId> listChildren(const ObjectId&) const override { return {}; }
	bool attachChild(const ObjectId&, const ObjectId&, QString* outError) override
	{
		if (outError)
			*outError = QStringLiteral("NullDataService");
		return false;
	}

	QVector<PropertyRowDto> propertyRows(const ObjectId&) const override { return {}; }
	bool applyPropertyChange(const ObjectId&, const QString&, const QString&, QString* outError) override
	{
		if (outError)
			*outError = QStringLiteral("NullDataService");
		return false;
	}

	BBoxDto boundingBox(const ObjectId&) const override { return {}; }
	bool hasVisualBranch(const ObjectId&) const override { return false; }

	QJsonObject saveObjectToJson(const ObjectId&) const override { return {}; }
	ObjectId loadObjectFromJson(const QJsonObject&, QString* outError) override
	{
		if (outError)
			*outError = QStringLiteral("NullDataService");
		return {};
	}

	ObjectId importFromFile(const QString&, const ImportOptionsDto&, QString* outError) override
	{
		if (outError)
			*outError = QStringLiteral("NullDataService");
		return {};
	}

	QVector<ObjectId> topoOrder() const override { return {}; }
	QVector<ObjectId> listAll() const override { return {}; }
	QVector<ObjectId> parentsOf(const ObjectId&) const override { return {}; }
};

class NullRobotService final : public IRobotService
{
public:
	RobotRegistrationDto registerUrdfRobot(const QString&, const ImportOptionsDto&) override
	{
		return {false, QStringLiteral("NullRobotService"), {}};
	}

	bool applyJointAnglesRad(const ObjectId&, const QVector<double>&, QVector<double>* /*outAggregated*/,
		QString* outError) override
	{
		if (outError)
			*outError = QStringLiteral("NullRobotService");
		return false;
	}

	bool planInstruction(const MotionInstructionDto&, const PlanContextDto&, PlanResultDto& out,
		QString* outError) override
	{
		out = {};
		if (outError)
			*outError = QStringLiteral("NullRobotService");
		return false;
	}

	QJsonArray robotProgramsJson() const override { return {}; }
	bool setRobotProgramsJson(const QJsonArray&, QString* outError) override
	{
		if (outError)
			*outError = QStringLiteral("NullRobotService");
		return false;
	}
};

class NullRenderView final : public IRenderView
{
public:
	explicit NullRenderView(QWidget* parent) : m_widget(new QWidget(parent)) {}

	QWidget* widget() override { return m_widget.get(); }
	const QWidget* widget() const override { return m_widget.get(); }

	void setWorldMatrix(const ObjectId&, const Mat4&) override {}
	bool getWorldMatrix(const ObjectId&, Mat4&) const override { return false; }
	void setVisible(const ObjectId&, bool) override {}
	void removeVisual(const ObjectId&) override {}
	bool hasVisualBranch(const ObjectId&) const override { return false; }
	bool tryGetModelCenterMm(const ObjectId&, double&, double&, double&) const override { return false; }
	void setPickHandler(PickHandler) override {}
	void clearPickHandler() override {}
	void requestRedraw() override {}
	void focusCameraOnBackend(const ObjectId&) override {}
	void setBackendLogicalParent(const ObjectId&, const ObjectId&) override {}

	SceneNodeInfo sceneGraphSnapshot(int /*maxDepth*/) const override { return {}; }
	bool selectedPosition(float&, float&, float&) const override { return false; }
	bool selectedRotationEulerDeg(float&, float&, float&) const override { return false; }

private:
	std::unique_ptr<QWidget> m_widget;
};

class NullRenderViewFactory final : public IRenderViewFactory
{
public:
	std::unique_ptr<IRenderView> createView(QWidget* parent) override
	{
		return std::make_unique<NullRenderView>(parent);
	}
};

} // namespace

std::unique_ptr<IDataService> makeNullDataService()
{
	return std::make_unique<NullDataService>();
}

std::unique_ptr<IRobotService> makeNullRobotService()
{
	return std::make_unique<NullRobotService>();
}

std::unique_ptr<IRenderViewFactory> makeNullRenderViewFactory()
{
	return std::make_unique<NullRenderViewFactory>();
}

} // namespace cloudsim::core
