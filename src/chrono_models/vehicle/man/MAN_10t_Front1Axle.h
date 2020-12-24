// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// MAN 10t (front1) steered and driven solid three link axle.
//
// =============================================================================

#ifndef MAN_10T_FRONT1_AXLE_H
#define MAN_10T_FRONT1_AXLE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidBellcrankThreeLinkAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

class CH_MODELS_API MAN_10t_Front1Axle : public ChSolidBellcrankThreeLinkAxle {
  public:
    MAN_10t_Front1Axle(const std::string& name);
    ~MAN_10t_Front1Axle();

  protected:
    virtual const ChVector<> getLocation(PointId which) override;

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getKnuckleMass() const override { return m_knuckleMass; }
    virtual double getBellcrankMass() const override { return m_knuckleMass; }
    virtual double getDraglinkMass() const override { return m_draglinkMass; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector<> getAxleTubeCOM() const override { return ChVector<>(0, 0, 0); }

    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getKnuckleInertia() const override { return m_knuckleInertia; }
    virtual const ChVector<>& getBellcrankInertia() const override { return m_bellcrankInertia; }
    virtual const ChVector<>& getDraglinkInertia() const override { return m_draglinkInertia; }

    virtual double getAxleInertia() const override { return m_axleShaftInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

  private:
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_axleShaftInertia;

    static const double m_axleTubeMass;
    static const double m_spindleMass;
    static const double m_knuckleMass;
    static const double m_bellcrankMass;
    static const double m_draglinkMass;

    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const ChVector<> m_axleTubeInertia;
    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_knuckleInertia;
    static const ChVector<> m_bellcrankInertia;
    static const ChVector<> m_draglinkInertia;

    static const double m_springCoefficient1;
    static const double m_springCoefficient2;
    static const double m_springRestLength;
    static const double m_springDesignLength;
    static const double m_springMinLength;
    static const double m_springMaxLength;

    static const double m_damperCoefCompression;
    static const double m_damperDegresCompression;
    static const double m_damperDegresExpansion;
    static const double m_damperCoefExpansion;

    static const double m_twin_tire_dist;
};

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
