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
// HMMWV shafts-based brake model.
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/HMMWV_BrakeShafts.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_BrakeShafts::m_maxtorque = 4000;
const double HMMWV_BrakeShafts::m_shaft_inertia = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_BrakeShafts::HMMWV_BrakeShafts(const std::string& name) : ChBrakeShafts(name) {}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
