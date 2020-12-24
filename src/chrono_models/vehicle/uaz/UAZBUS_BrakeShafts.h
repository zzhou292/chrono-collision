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
// Authors: Radu Serban
// =============================================================================
//
// UAZ shafts-based brake model.
//
// =============================================================================

#ifndef UAZBUS_BRAKE_SHAFTS_H
#define UAZBUS_BRAKE_SHAFTS_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace uaz {

/// @addtogroup vehicle_models_uaz
/// @{

/// Shafts-based UAZ front brake subsystem (uses a clutch between two shafts).
class CH_MODELS_API UAZBUS_BrakeShaftsFront : public ChBrakeShafts {
  public:
    UAZBUS_BrakeShaftsFront(const std::string& name);
    ~UAZBUS_BrakeShaftsFront() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }
    virtual double GetShaftInertia() override { return m_shaft_inertia; }

  private:
    static const double m_maxtorque;
    static const double m_shaft_inertia;
};

/// Shafts-based UAZ rear brake subsystem (uses a clutch between two shafts).
class CH_MODELS_API UAZBUS_BrakeShaftsRear : public ChBrakeShafts {
  public:
    UAZBUS_BrakeShaftsRear(const std::string& name);
    ~UAZBUS_BrakeShaftsRear() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }
    virtual double GetShaftInertia() override { return m_shaft_inertia; }

  private:
    static const double m_maxtorque;
    static const double m_shaft_inertia;
};

/// @} vehicle_models_uaz

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono

#endif
