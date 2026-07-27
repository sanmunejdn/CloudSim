#include "SketchConstraint.h"
#include "constraints/Constraints.h"

#include <QDebug>
#include <QUuid>
#include <QString>

#include <algorithm>
#include <mutex>
#include <utility>

namespace onecad::core::sketch {

namespace {

void registerBuiltins() {
    static std::once_flag registerFlag;
    std::call_once(registerFlag, []() {
        ConstraintFactory::registerType<constraints::CoincidentConstraint>("Coincident");
        ConstraintFactory::registerType<constraints::HorizontalConstraint>("Horizontal");
        ConstraintFactory::registerType<constraints::VerticalConstraint>("Vertical");
        ConstraintFactory::registerType<constraints::ParallelConstraint>("Parallel");
        ConstraintFactory::registerType<constraints::PerpendicularConstraint>("Perpendicular");
        ConstraintFactory::registerType<constraints::TangentConstraint>("Tangent");
        ConstraintFactory::registerType<constraints::EqualConstraint>("Equal");
        ConstraintFactory::registerType<constraints::DistanceConstraint>("Distance");
        ConstraintFactory::registerType<constraints::HorizontalDistanceConstraint>("HorizontalDistance");
        ConstraintFactory::registerType<constraints::VerticalDistanceConstraint>("VerticalDistance");
        ConstraintFactory::registerType<constraints::AngleConstraint>("Angle");
        ConstraintFactory::registerType<constraints::RadiusConstraint>("Radius");
        ConstraintFactory::registerType<constraints::DiameterConstraint>("Diameter");
        ConstraintFactory::registerType<constraints::ConcentricConstraint>("Concentric");
        ConstraintFactory::registerType<constraints::SymmetricConstraint>("Symmetric");
        ConstraintFactory::registerType<constraints::FixedConstraint>("Fixed");
        ConstraintFactory::registerType<constraints::MidpointConstraint>("Midpoint");
        ConstraintFactory::registerType<constraints::PointOnCurveConstraint>("PointOnCurve");
    });
}

} // namespace

SketchConstraint::SketchConstraint()
    : m_id(generateId()) {
}

SketchConstraint::SketchConstraint(const ConstraintID& id)
    : m_id(id.empty() ? generateId() : id) {
}

ConstraintID SketchConstraint::generateId() {
    return QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
}

bool SketchConstraint::references(const EntityID& entityId) const {
    const auto entities = referencedEntities();
    return std::any_of(entities.begin(), entities.end(),
                       [&](const EntityID& id) { return id == entityId; });
}

void SketchConstraint::serializeBase(QJsonObject& json) const {
    json["id"] = QString::fromStdString(m_id);
    json["type"] = QString::fromStdString(typeName());
}

bool SketchConstraint::deserializeBase(const QJsonObject& json, const char* expectedType) {
    if (!json.contains("type") || !json["type"].isString()) {
        return false;
    }

    const QString typeValue = json["type"].toString();
    if (typeValue != QString::fromUtf8(expectedType)) {
        return false;
    }

    if (json.contains("id") && !json["id"].isString()) {
        return false;
    }

    ConstraintID newId = json.contains("id")
                             ? json["id"].toString().toStdString()
                             : generateId();
    m_id = std::move(newId);
    return true;
}

gp_Pnt2d SketchConstraint::getDimensionTextPosition(const Sketch& sketch) const {
    return getIconPosition(sketch);
}

DimensionalConstraint::DimensionalConstraint(double value)
    : SketchConstraint(),
      m_value(value) {
}

DimensionalConstraint::DimensionalConstraint(const ConstraintID& id, double value)
    : SketchConstraint(id),
      m_value(value) {
}

std::unique_ptr<SketchConstraint> ConstraintFactory::fromJson(const QJsonObject& json) {
    registerBuiltins();

    if (!json.contains("type")) {
        qWarning() << "ConstraintFactory: missing type field";
        return nullptr;
    }

    if (!json["type"].isString()) {
        qWarning() << "ConstraintFactory: type field is not a string";
        return nullptr;
    }

    std::string typeName = json["type"].toString().toStdString();
    auto& registry = detail::constraintRegistry();
    auto it = registry.find(typeName);
    if (it == registry.end()) {
        qWarning() << "ConstraintFactory: unknown type" << QString::fromStdString(typeName);
        return nullptr;
    }

    auto constraint = it->second();
    if (!constraint || !constraint->deserialize(json)) {
        qWarning() << "ConstraintFactory: failed to deserialize type"
                   << QString::fromStdString(typeName);
        return nullptr;
    }

    return constraint;
}

namespace detail {

std::unordered_map<std::string, ConstraintFactoryFn>& constraintRegistry() {
    static std::unordered_map<std::string, ConstraintFactoryFn> registry;
    return registry;
}

} // namespace detail

} // namespace onecad::core::sketch
