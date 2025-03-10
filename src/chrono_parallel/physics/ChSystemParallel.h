// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: The definition of a parallel ChSystem, pretty much everything is
// done manually instead of using the functions used in ChSystem. This is to
// handle the different data structures present in the parallel implementation
//
// =============================================================================

#pragma once

#include <cstdlib>
#include <cfloat>
#include <memory>
#include <algorithm>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChContactSMC.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/fea/ChMesh.h"

#include "chrono_parallel/collision/ChCollisionModelParallel.h"
#include "chrono_parallel/physics/Ch3DOFContainer.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/ChSettings.h"
#include "chrono_parallel/ChMeasures.h"

namespace chrono {

class ChParallelDataManager;
class settings_container;

/// @addtogroup parallel_physics
/// @{

/// Base class for parallel systems.
class CH_PARALLEL_API ChSystemParallel : public ChSystem {

  public:
    ChSystemParallel();
    ChSystemParallel(const ChSystemParallel& other);
    virtual ~ChSystemParallel();

    virtual bool Integrate_Y() override;
    virtual void AddBody(std::shared_ptr<ChBody> newbody) override;
    virtual void AddLink(std::shared_ptr<ChLinkBase> link) override;
    virtual void AddMesh(std::shared_ptr<fea::ChMesh> mesh) override;
    virtual void AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> newitem) override;

    void ClearForceVariables();
    virtual void Update();
    virtual void UpdateBilaterals();
    virtual void UpdateLinks();
    virtual void UpdateOtherPhysics();
    virtual void UpdateRigidBodies();
    virtual void UpdateShafts();
    virtual void UpdateMotorLinks();
    virtual void Update3DOFBodies();
    void RecomputeThreads();

    virtual ChBody* NewBody() override;
    virtual ChBodyAuxRef* NewBodyAuxRef() override;

    virtual void AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) = 0;
    virtual void UpdateMaterialSurfaceData(int index, ChBody* body) = 0;
    virtual void Setup() override;
    virtual void ChangeCollisionSystem(CollisionSystemType type);

    /// Change the default composition laws for contact surface materials
    /// (coefficient of friction, cohesion, compliance, etc.).
    virtual void SetMaterialCompositionStrategy(std::unique_ptr<ChMaterialCompositionStrategy>&& strategy) override;

    virtual void PrintStepStats();
    unsigned int GetNumBodies();
    unsigned int GetNumShafts();
    unsigned int GetNumContacts();
    unsigned int GetNumBilaterals();

    /// Return the time (in seconds) spent for computing the time step.
    virtual double GetTimerStep() const override;

    /// Return the time (in seconds) for time integration, within the time step.
    virtual double GetTimerAdvance() const override;

    /// Return the time (in seconds) for the solver, within the time step.
    /// Note that this time excludes any calls to the solver's Setup function.
    virtual double GetTimerLSsolve() const override;

    /// Return the time (in seconds) for the solver Setup phase, within the time step.
    virtual double GetTimerLSsetup() const override;

    /// Return the time (in seconds) for calculating/loading Jacobian information, within the time step.
    virtual double GetTimerJacobian() const override;

    /// Return the time (in seconds) for runnning the collision detection step, within the time step.
    virtual double GetTimerCollision() const override;

    /// Return the time (in seconds) for system setup, within the time step.
    virtual double GetTimerSetup() const override { return 0; }

    /// Return the time (in seconds) for updating auxiliary data, within the time step.
    virtual double GetTimerUpdate() const override;

    /// Calculate current body AABBs.
    void CalculateBodyAABB();

    /// Calculate cummulative contact forces for all bodies in the system.
    /// Note that this function must be explicitly called by the user at each time where
    /// calls to GetContactableForce or ContactableTorque are made.
    virtual void CalculateContactForces() {}

    /// Return the resultant applied force on the specified body.
    /// This resultant force includes all external applied loads acting on the body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact forces are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    virtual ChVector<> GetBodyAppliedForce(ChBody* body) override;

    /// Return the resultant applied torque on the specified body.
    /// This resultant torque includes all external applied loads acting on the body (from gravity, loads, springs,
    /// etc). However, this does *not* include any constraint forces. In particular, contact torques are not included if
    /// using the NSC formulation, but are included when using the SMC formulation.
    virtual ChVector<> GetBodyAppliedTorque(ChBody* body) override;

    /// Get the contact force on the body with specified id.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact forces is desired.
    virtual real3 GetBodyContactForce(uint body_id) const = 0;

    /// Get the contact torque on the body with specified id.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact torques is desired.
    virtual real3 GetBodyContactTorque(uint body_id) const = 0;

    /// Get the contact force on the specified body.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact forces is desired.
    real3 GetBodyContactForce(std::shared_ptr<ChBody> body) const { return GetBodyContactForce(body->GetId()); }

    /// Get the contact torque on the specified body.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact torques is desired.
    real3 GetBodyContactTorque(std::shared_ptr<ChBody> body) const { return GetBodyContactTorque(body->GetId()); }

    settings_container* GetSettings();

    /// Set the number of OpenMP threads used by Chrono itself, Eigen, and the collision detection system.
    /// <pre>
    ///  num_threads_chrono    - used for all OpenMP constructs and thrust algorithms in Chrono::Parallel.
    ///  num_threads_collision - Ignored.  Chrono::Parallel sets num_threads_collision = num_threads_chrono.
    ///  num_threads_eigen     - used in the Eigen sparse direct solvers and a few linear algebra operations.
    ///                          Note that Eigen enables multi-threaded execution only under certain size conditions.
    ///                          See the Eigen documentation.
    ///                          If passing 0, then num_threads_eigen = num_threads_chrono.
    /// </pre>
    /// By default, num_threads_chrono is set to omp_get_num_procs() and num_threads_eigen is set to 1.
    virtual void SetNumThreads(int num_threads_chrono,
                               int num_threads_collision = 0,
                               int num_threads_eigen = 0) override;

    /// Enable dynamic adjustment of number of threads between the specified limits.
    /// The initial number of threads is set to min_threads.
    void EnableThreadTuning(int min_threads, int max_threads);

    // Based on the specified logging level and the state of that level, enable or disable logging level.
    void SetLoggingLevel(LoggingLevel level, bool state = true);

    /// Calculate the (linearized) bilateral constraint violations.
    /// Return the maximum constraint violation.
    double CalculateConstraintViolation(std::vector<double>& cvec);

    ChParallelDataManager* data_manager;

    int current_threads;

  protected:
    double old_timer, old_timer_cd;
    bool detect_optimal_threads;

    int detect_optimal_bins;
    std::vector<double> timer_accumulator, cd_accumulator;
    uint frame_threads, frame_bins, counter;
    std::vector<ChLink*>::iterator it;

    CollisionSystemType collision_system_type;

  private:
    void AddShaft(std::shared_ptr<ChShaft> shaft);

    std::vector<ChShaft*> shaftlist;
    std::vector<ChLinkMotorLinearSpeed*> linmotorlist;
    std::vector<ChLinkMotorRotationSpeed*> rotmotorlist;
};

//====================================================================================================

/// Parallel systems using non-smooth contact (complementarity-based) method.
class CH_PARALLEL_API ChSystemParallelNSC : public ChSystemParallel {

  public:
    ChSystemParallelNSC();
    ChSystemParallelNSC(const ChSystemParallelNSC& other);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSystemParallelNSC* Clone() const override { return new ChSystemParallelNSC(*this); }

    void ChangeSolverType(SolverType type);
    void Initialize();

    virtual ChContactMethod GetContactMethod() const override { return ChContactMethod::NSC; }
    virtual void AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) override;
    virtual void UpdateMaterialSurfaceData(int index, ChBody* body) override;

    void Add3DOFContainer(std::shared_ptr<Ch3DOFContainer> container);

    void CalculateContactForces() override;
    real CalculateKineticEnergy();
    real CalculateDualObjective();

    virtual real3 GetBodyContactForce(uint body_id) const override;
    virtual real3 GetBodyContactTorque(uint body_id) const override;
    using ChSystemParallel::GetBodyContactForce;
    using ChSystemParallel::GetBodyContactTorque;

    virtual void AssembleSystem();
    virtual void SolveSystem();
};

//====================================================================================================

/// Parallel systems using smooth contact (penalty-based) method.
class CH_PARALLEL_API ChSystemParallelSMC : public ChSystemParallel {

  public:
    ChSystemParallelSMC();
    ChSystemParallelSMC(const ChSystemParallelSMC& other);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSystemParallelSMC* Clone() const override { return new ChSystemParallelSMC(*this); }

    virtual ChContactMethod GetContactMethod() const override { return ChContactMethod::SMC; }
    virtual void AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) override;
    virtual void UpdateMaterialSurfaceData(int index, ChBody* body) override;

    virtual void Setup() override;
    virtual void ChangeCollisionSystem(CollisionSystemType type) override;

    virtual real3 GetBodyContactForce(uint body_id) const override;
    virtual real3 GetBodyContactTorque(uint body_id) const override;
    using ChSystemParallel::GetBodyContactForce;
    using ChSystemParallel::GetBodyContactTorque;

    virtual void PrintStepStats() override;

    double GetTimerProcessContact() const {
        return data_manager->system_timer.GetTime("ChIterativeSolverParallelSMC_ProcessContact");
    }
};

/// @} parallel_physics

}  // end namespace chrono
