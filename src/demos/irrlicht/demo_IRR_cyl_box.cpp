// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Collision test
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// ====================================================================================

class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager() {}
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override;
};

// ====================================================================================

class CustomCollision : public ChSystem::CustomCollisionCallback {
  public:
    CustomCollision(std::shared_ptr<ChBody> ground,
                    std::shared_ptr<ChBody> object,
                    std::shared_ptr<ChMaterialSurface> ground_mat,
                    std::shared_ptr<ChMaterialSurface> object_mat,
                    double radius,
                    double hlen);
    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    std::shared_ptr<ChBody> m_ground;
    std::shared_ptr<ChBody> m_object;
    std::shared_ptr<ChMaterialSurface> m_ground_mat;
    std::shared_ptr<ChMaterialSurface> m_object_mat;
    double m_radius;
    double m_hlen;
};

// ====================================================================================

std::shared_ptr<geometry::ChTriangleMeshConnected> GroundMesh(double hx, double hy);

// ====================================================================================

int main(int argc, char* argv[]) {
    // ------------------
    // Collision settings
    // ------------------

    enum class CollisionType { CYLINDER, CYLSHELL, MESH, CUSTOM };
    CollisionType object_model = CollisionType::CYLSHELL;

    std::string tire_mesh_file = GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_fine.obj");
    ////std::string tire_mesh_file = GetChronoDataFile("vehicle/hmmwv/hmmwv_tire_coarse.obj");

    // ---------------------------
    // Contact material properties
    // ---------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    bool use_mat_properties = true;

    float object_friction = 0.8f;
    float object_restitution = 0.0f;
    float object_young_modulus = 2e7f;
    float object_poisson_ratio = 0.3f;
    float object_adhesion = 0.0f;
    float object_kn = 2e5;
    float object_gn = 40;
    float object_kt = 2e5;
    float object_gt = 20;

    float ground_friction = 0.9f;
    float ground_restitution = 0.0f;
    float ground_young_modulus = 2e7f;
    float ground_poisson_ratio = 0.3f;
    float ground_adhesion = 0.0f;
    float ground_kn = 2e5;
    float ground_gn = 40;
    float ground_kt = 2e5;
    float ground_gt = 20;

    // ---------------------------------
    // Parameters for the falling object
    // ---------------------------------

    double init_height = 0.75;
    double init_roll = 0 * CH_C_DEG_TO_RAD;

    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_omg(0, 0, 0);

    double radius = 0.5;  // cylinder radius
    double hlen = 0.5;    // cylinder half-length

    // ---------
    // Step size
    // ---------

    //double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-4;
    double time_step = 5e-4;
    // --------------
    // Print settings
    // --------------

    std::cout << "-----------------------" << std::endl;
    std::cout << "Contact method:        " << (contact_method == ChContactMethod::SMC ? "SMC" : "NSC") << std::endl;
    std::cout << "Step size:             " << time_step << std::endl;
    std::cout << "Object collision type: ";
    switch (object_model) {
        case CollisionType::CYLINDER:
            std::cout << "CYLINDER" << std::endl;
            break;
        case CollisionType::CYLSHELL:
            std::cout << "CYLSHELL" << std::endl;
            break;
        case CollisionType::MESH:
            std::cout << "MESH" << std::endl;
            break;
        case CollisionType::CUSTOM:
            std::cout << "CUSTOM" << std::endl;
            break;
    }
    std::cout << "-----------------------" << std::endl;

    // -----------------
    // Create the system
    // -----------------

    ChSystem* system;

    switch (contact_method) {
        case ChContactMethod::NSC:
            system = new ChSystemNSC();
            break;
        case ChContactMethod::SMC:
            system = new ChSystemSMC(use_mat_properties);
            break;
    }

    system->Set_G_acc(ChVector<>(0, -9.81, 0));

    // Create the Irrlicht visualization
    ChIrrApp application(system, L"Collision test", irr::core::dimension2d<irr::u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0, 1, 1), irr::core::vector3df(-2, 0, 1));

    // Render contact forces or normals
    application.SetSymbolscale(5e-4);
    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);
    ////application.SetSymbolscale(1);
    ////application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_NORMALS);

    // Rotation Z->Y (because meshes used here assume Z up)
    ChQuaternion<> z2y = Q_from_AngX(-CH_C_PI_2);

    // Create the falling object
    auto object = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(object);

    object->SetName("object");
    object->SetMass(400);
    object->SetInertiaXX(40.0 * ChVector<>(1, 1, 0.2));
    object->SetPos(ChVector<>(-1, init_height, 0));
    object->SetRot(z2y * Q_from_AngX(init_roll));
    object->SetPos_dt(init_vel);
    object->SetWvel_par(init_omg);
    object->SetCollide(true);
    object->SetBodyFixed(false);

    auto object_mat = ChMaterialSurface::DefaultMaterial(contact_method);
    object_mat->SetFriction(object_friction);
    object_mat->SetRestitution(object_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChMaterialSurfaceSMC>(object_mat);
        matSMC->SetYoungModulus(object_young_modulus);
        matSMC->SetPoissonRatio(object_poisson_ratio);
        matSMC->SetKn(object_kn);
        matSMC->SetGn(object_gn);
        matSMC->SetKt(object_kt);
        matSMC->SetGt(object_gt);
    }

    switch (object_model) {
        case CollisionType::CYLINDER:
        case CollisionType::CUSTOM: {
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCylinder(object_mat, radius, radius, hlen, ChVector<>(0), ChMatrix33<>(1));
            object->GetCollisionModel()->BuildModel();

            auto cyl = chrono_types::make_shared<ChCylinderShape>();
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, +hlen, 0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, -hlen, 0);
            cyl->GetCylinderGeometry().rad = radius;
            object->AddAsset(cyl);

            break;
        }
        case CollisionType::CYLSHELL: {
            double sphere_r = 0.01;
            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddCylindricalShell(object_mat, radius-sphere_r, hlen-sphere_r, ChVector<>(0), ChMatrix33<>(1));
            object->GetCollisionModel()->BuildModel();

            auto cyl = chrono_types::make_shared<ChCylinderShape>();
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, +hlen, 0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, -hlen, 0);
            cyl->GetCylinderGeometry().rad = radius;
            object->AddAsset(cyl);

            break;
        }
        case CollisionType::MESH: {
            double sphere_r = 0.005;
            auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            if (!trimesh->LoadWavefrontMesh(tire_mesh_file, true, false))
                return 1;

            object->GetCollisionModel()->ClearModel();
            object->GetCollisionModel()->AddTriangleMesh(object_mat, trimesh, false, false, ChVector<>(0),
                                                         ChMatrix33<>(1), sphere_r);
            object->GetCollisionModel()->BuildModel();

            auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetWireframe(true);
            object->AddAsset(trimesh_shape);

            break;
        }
    }

    std::shared_ptr<ChColorAsset> mcol(new ChColorAsset);
    mcol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    object->AddAsset(mcol);

    // Create ground body
    auto ground = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(ground);

    ground->SetName("ground");
    ground->SetMass(1);
    ground->SetPos(ChVector<>(0, 0, 0));
    ground->SetRot(Q_from_AngX(CH_C_PI));
    ground->SetCollide(true);
    ground->SetBodyFixed(true);

    auto ground_mat = ChMaterialSurface::DefaultMaterial(contact_method);
    ground_mat->SetFriction(ground_friction);
    ground_mat->SetRestitution(ground_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChMaterialSurfaceSMC>(ground_mat);
        matSMC->SetYoungModulus(ground_young_modulus);
        matSMC->SetPoissonRatio(ground_poisson_ratio);
        matSMC->SetKn(ground_kn);
        matSMC->SetGn(ground_gn);
        matSMC->SetKt(ground_kt);
        matSMC->SetGt(ground_gt);
    }

    double hx = 2;
    double hy = 0.5;
    double hz = 4;

    ground->GetCollisionModel()->ClearModel();
    ground->GetCollisionModel()->AddBox(ground_mat, hx, hy, hz, ChVector<>(0, +hy, 0));
    ground->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(hx, hy, hz);
    box->GetBoxGeometry().Pos = ChVector<>(0, +hy, 0);
    ground->AddAsset(box);

    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("textures/checker1.png"));
    texture->SetTextureScale(4, 2);
    ground->AddAsset(texture);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    auto cmanager = chrono_types::make_shared<ContactManager>();

    if (object_model == CollisionType::CUSTOM) {
        auto ccollision =
            chrono_types::make_shared<CustomCollision>(ground, object, ground_mat, object_mat, radius, hlen);
        system->RegisterCustomCollisionCallback(ccollision);
    }


    // Create ground body 2
    auto ground_2 = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(ground_2);

    ground_2->SetName("ground_2");
    ground_2->SetMass(1);
    ground_2->SetPos(ChVector<>(4.1,0,0));
    ground_2->SetRot(Q_from_AngX(CH_C_PI));
    ground_2->SetCollide(true);
    ground_2->SetBodyFixed(true);

    auto ground_mat_2 = ChMaterialSurface::DefaultMaterial(contact_method);
    ground_mat_2->SetFriction(ground_friction);
    ground_mat_2->SetRestitution(ground_restitution);
    if (contact_method == ChContactMethod::SMC) {
        auto matSMC = std::static_pointer_cast<ChMaterialSurfaceSMC>(ground_mat_2);
        matSMC->SetYoungModulus(ground_young_modulus);
        matSMC->SetPoissonRatio(ground_poisson_ratio);
        matSMC->SetKn(ground_kn);
        matSMC->SetGn(ground_gn);
        matSMC->SetKt(ground_kt);
        matSMC->SetGt(ground_gt);
    }

    double hx_2 = 2;
    double hy_2 = 0.5;
    double hz_2 = 4;

    ground_2->GetCollisionModel()->ClearModel();
    ground_2->GetCollisionModel()->AddBox(ground_mat, hx_2, hy_2, hz_2, ChVector<>(0, +hy_2, 0));
    ground_2->GetCollisionModel()->BuildModel();

    auto box_2 = chrono_types::make_shared<ChBoxShape>();
    box_2->GetBoxGeometry().Size = ChVector<>(hx_2, hy_2, hz_2);
    box_2->GetBoxGeometry().Pos = ChVector<>(0, +hy, 0);
    ground_2->AddAsset(box_2);

    auto texture_2 = chrono_types::make_shared<ChTexture>();
    texture_2->SetTextureFilename(GetChronoDataFile("textures/checker1.png"));
    texture_2->SetTextureScale(4, 2);
    ground_2->AddAsset(texture_2);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    if (object_model == CollisionType::CUSTOM) {
        auto ccollision =
            chrono_types::make_shared<CustomCollision>(ground_2, object, ground_mat_2, object_mat, radius, hlen);
        system->RegisterCustomCollisionCallback(ccollision);
    }


    // ---------------
    // Simulation loop
    // ---------------
    int count = 0;
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.EndScene();

        system->DoStepDynamics(time_step);

        float time = time_step*count;
        count++;
        //std::cout<<"time: "<<time<<std::endl;
        if(time == 3){
            ChVector<> change_vel(5, 0, 0);
            object->SetPos_dt(change_vel);
        }
        ChVector<> force = object->GetContactForce();
        //std::cout<<"x:"<<force.x()<<" y:"<<force.y()<<" z:"<<force.z()<<std::endl;
    }

    return 0;
}

// ====================================================================================

std::shared_ptr<geometry::ChTriangleMeshConnected> GroundMesh(double hx, double hy) {
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    std::vector<ChVector<>>& v = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& n = trimesh->getCoordsNormals();
    std::vector<ChVector<>>& uv = trimesh->getCoordsUV();
    std::vector<ChVector<int>>& iv = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& in = trimesh->getIndicesNormals();

    v.resize(4);
    n.resize(4);
    uv.resize(4);

    iv.resize(2);
    in.resize(2);

    v[0] = ChVector<>(+hx, +hy, 0);
    v[1] = ChVector<>(-hx, +hy, 0);
    v[2] = ChVector<>(-hx, -hy, 0);
    v[3] = ChVector<>(+hx, -hy, 0);

    n[0] = ChVector<>(0, 0, 1);
    n[1] = ChVector<>(0, 0, 1);
    n[2] = ChVector<>(0, 0, 1);
    n[3] = ChVector<>(0, 0, 1);

    uv[0] = ChVector<>(1, 1, 0);
    uv[1] = ChVector<>(0, 1, 0);
    uv[2] = ChVector<>(0, 0, 0);
    uv[3] = ChVector<>(1, 0, 0);

    iv[0] = ChVector<int>(0, 1, 2);
    iv[1] = ChVector<int>(0, 2, 3);

    in[0] = ChVector<int>(0, 1, 2);
    in[1] = ChVector<int>(0, 2, 3);

    return trimesh;
}

// ====================================================================================

bool ContactManager::OnReportContact(const ChVector<>& pA,
                                     const ChVector<>& pB,
                                     const ChMatrix33<>& plane_coord,
                                     const double& distance,
                                     const double& eff_radius,
                                     const ChVector<>& cforce,
                                     const ChVector<>& ctorque,
                                     ChContactable* modA,
                                     ChContactable* modB) {
    auto bodyA = static_cast<ChBody*>(modA);
    auto bodyB = static_cast<ChBody*>(modB);

    std::cout << "  " << bodyA->GetName() << "  " << bodyB->GetName() << std::endl;
    std::cout << "  " << distance << std::endl;
    std::cout << "  " << pA << "    " << pB << std::endl;
    std::cout << "  " << plane_coord.Get_A_Xaxis() << std::endl;
    std::cout << std::endl;

    return true;
}

CustomCollision::CustomCollision(std::shared_ptr<ChBody> ground,
                                 std::shared_ptr<ChBody> object,
                                 std::shared_ptr<ChMaterialSurface> ground_mat,
                                 std::shared_ptr<ChMaterialSurface> object_mat,
                                 double radius,
                                 double hlen)
    : m_ground(ground),
      m_object(object),
      m_ground_mat(ground_mat),
      m_object_mat(object_mat),
      m_radius(radius),
      m_hlen(hlen) {
    // Disable Chrono collision detection for ground and object
    m_ground->SetCollide(false);
    m_object->SetCollide(false);
}

// Assumptions:
// - object with cylindrical shape
// - ground height at 0
// - collision with cylinder circumference
void CustomCollision::OnCustomCollision(ChSystem* system) {
    // Lowest point on central cylinder plane (in absolute frame)
    ChVector<> A = m_object->GetPos() - m_radius * ChVector<>(0, 1, 0);
    // Express in local frame
    ChVector<> A_loc = m_object->TransformPointParentToLocal(A);
    // Points on cylinder edges (local frame)
    ChVector<> B1_loc = A_loc + ChVector<>(0, -m_hlen, 0);
    ChVector<> B2_loc = A_loc + ChVector<>(0, +m_hlen, 0);
    // Express in absolute frame
    ChVector<> B1 = m_object->TransformPointLocalToParent(B1_loc);
    ChVector<> B2 = m_object->TransformPointLocalToParent(B2_loc);

    // If B1 below ground surface (assumed at 0), add contact
    if (B1.y() < 0) {
        collision::ChCollisionInfo contact;
        contact.modelA = m_ground->GetCollisionModel().get();
        contact.modelB = m_object->GetCollisionModel().get();
        contact.shapeA = nullptr;
        contact.shapeB = nullptr;
        contact.vN = ChVector<>(0, 1, 0);
        contact.vpB = B1;
        contact.vpA = ChVector<>(B1.x(), 0, B1.z());
        contact.distance = B1.y();
        system->GetContactContainer()->AddContact(contact, m_ground_mat, m_object_mat);
    }

    // If B2 below ground surface (assumed at 0), add contact
    if (B2.y() < 0) {
        collision::ChCollisionInfo contact;
        contact.modelA = m_ground->GetCollisionModel().get();
        contact.modelB = m_object->GetCollisionModel().get();
        contact.shapeA = nullptr;
        contact.shapeB = nullptr;
        contact.vN = ChVector<>(0, 1, 0);
        contact.vpB = B2;
        contact.vpA = ChVector<>(B2.x(), 0, B2.z());
        contact.distance = B2.y();
        system->GetContactContainer()->AddContact(contact, m_ground_mat, m_object_mat);
    }
}
