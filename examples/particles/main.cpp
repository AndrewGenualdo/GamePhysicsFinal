#include <chrono>
#include <functional>
#include <iostream>

#include <raylib.h>
#include <raymath.h> //Needed for most Raylib math functions
#include <cyclone/cyclone.h>
#include <imgui.h>
#include <queue>
#include <rlImGui.h>
#include <set>
#include <map>

#include <ew/camera.h>

#include "rlgl.h"
#include "cyclone/collider.h"
#include "cyclone/octree.h"
#include "cyclone/rigidbody.h"
#include "ew/conversions.h"



int main() {
    // Initialization
   //--------------------------------------------------------------------------------------
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "Bouncy balls!!!");

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    camera.position = {0.0f, 10.0f, 15.0f };   // Camera position
    camera.target = { 0.0f, 0.0f, 0.0f };       // Camera looking at point
    camera.up = { 0.0f, 1.0f, 0.0f };           // Camera up vecto
    camera.fovy = 45.0f;                        // Camera vertical field-of-view in degrees
    camera.projection = CAMERA_PERSPECTIVE;     // Camera projection type - perspective vs orthographic

    //default settings
    /*bool useOctree = true;
    bool showOctree = false;
    bool autoSpawn = false;
    float sphereRadius = 0.125f;
    float sphereSpeed = 5.0f;
    float roomSize = 10;
    float octreeMinSize = 0.5f * roomSize * 0.2f;*/

    float restitution = 0.99f;

    Model sphereModel = LoadModelFromMesh(GenMeshSphere(1, 100, 100));
    Model cubeModel = LoadModelFromMesh(GenMeshCube(1, 1, 1));
    Model planeModel = LoadModelFromMesh(GenMeshPlane(10, 10, 10, 10));

    std::vector<cyclone::Collider*> colliders;
    auto sphere1 = new cyclone::SphereCollider();
    sphere1->getRigidbody()->setPosition(cyclone::Vector3(0, 10, 0));
    sphere1->getRigidbody()->setInverseMass(1);
    sphere1->setRadius(0.3541666666666667f);
    sphere1->getRigidbody()->setAcceleration(cyclone::Vector3(0, -10, 0));
    colliders.push_back(sphere1);

#define addBoxCollider(pos, scale, mass) {auto newBox = new cyclone::BoxCollider(); newBox->getRigidbody()->setPosition(pos); newBox->setHalfSize(scale);newBox->getRigidbody()->setInverseMass(mass); if(mass != 0) newBox->getRigidbody()->setAcceleration(cyclone::Vector3(0, 0, 0)); colliders.push_back(newBox);}

    addBoxCollider(cyclone::Vector3(0, 0, -29), cyclone::Vector3(3.5, 1, 60), 0);
    addBoxCollider(cyclone::Vector3(0, -0.55, -29), cyclone::Vector3(5.6, 0.1, 60), 0);
    addBoxCollider(cyclone::Vector3(2.75, 0, -29), cyclone::Vector3(0.1, 1, 60), 0);
    addBoxCollider(cyclone::Vector3(-2.75, 0, -29), cyclone::Vector3(0.1, 1, 60), 0);
    addBoxCollider(cyclone::Vector3(-1.5, 1.125, -58.7292), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(-0.5, 1.125, -58.7292), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(0.5, 1.125, -58.7292), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(1.5, 1.125, -58.7292), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(-1, 1.125, -57.8646), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(0, 1.125, -57.8646), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(1, 1.125, -57.8646), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(-0.5, 1.125, -57), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(0.5, 1.125, -57), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);
    addBoxCollider(cyclone::Vector3(0, 1.125, -56.1354), cyclone::Vector3(0.395833, 1.25, 0.395833), 1);

    auto *data = new cyclone::CollisionData();
    data->reset();

    typedef bool (*intersectionTest)(const cyclone::Collider &, const cyclone::Collider &);
    typedef int (*collisionTest)(const cyclone::Collider &, const cyclone::Collider &, const cyclone::CollisionData &);
    std::map<int, std::pair<intersectionTest, collisionTest>> testTypes = std::map<int, std::pair<intersectionTest, collisionTest>>();
    #define addTestType(a, b, c, d) testTypes[static_cast<int>(a) & static_cast<int>(b)] = std::make_pair(reinterpret_cast<intersectionTest>(c), reinterpret_cast<collisionTest>(d));
    addTestType(cyclone::ColliderType::Sphere, cyclone::ColliderType::Sphere, cyclone::IntersectionTests::SphereSphere, cyclone::CollisionTests::SphereSphere);
    addTestType(cyclone::ColliderType::Sphere, cyclone::ColliderType::Plane, cyclone::IntersectionTests::SpherePlane, cyclone::CollisionTests::SphereTruePlane);
    addTestType(cyclone::ColliderType::Sphere, cyclone::ColliderType::Box, cyclone::IntersectionTests::SphereBox, cyclone::CollisionTests::SphereBox);

    while (!WindowShouldClose()) {


        float deltaTime = GetFrameTime();
        //Input
        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            DisableCursor();
        }
        if (IsMouseButtonReleased(MOUSE_RIGHT_BUTTON)) {
            EnableCursor();
        }
        //Only allow movement if the cursor is hidden
        if (IsCursorHidden()) {
            ew::UpdateFlyCamera(&camera, deltaTime);
        }

        if (IsKeyPressed(KEY_P)) {
            std::cout << std::endl;

            for (int i = 0; i < colliders.size(); i++) {
                if (colliders[i]->getType() == cyclone::ColliderType::Box) {
                    std::cout << "addBoxCollider(cyclone::Vector3(" << colliders[i]->getRigidbody()->getPosition()->x << ", " << colliders[i]->getRigidbody()->getPosition()->y << ", " << colliders[i]->getRigidbody()->getPosition()->z << "), ";
                    cyclone::BoxCollider *box = reinterpret_cast<cyclone::BoxCollider *>(colliders[i]);
                    std::cout << "cyclone::Vector3(" << box->getHalfSize().x << ", " << box->getHalfSize().y << ", " << box->getHalfSize().z << "), " << std::to_string(*colliders[i]->getRigidbody()->getInverseMass()) << ");" << std::endl;
                }
            }

            std::cout << std::endl;
        }

        for (auto & collider : colliders) collider->getRigidbody()->integrate(deltaTime);

        for (int i = 0; i < colliders.size(); i++) {
            for (int j = i + 1; j < colliders.size(); j++) {
                bool swap = static_cast<int>(colliders[i]->getType()) > static_cast<int>(colliders[j]->getType());
                cyclone::Collider *a;
                cyclone::Collider *b;
                if (swap) {
                    a = colliders[j];
                    b = colliders[i];
                } else {
                    a = colliders[i];
                    b = colliders[j];
                }
                int typeKey = static_cast<int>(a->getType()) & static_cast<int>(b->getType());
                if (testTypes.find(typeKey) != testTypes.end()) {
                    auto tests = testTypes[typeKey];
                    if ((*tests.first)(*a, *b)) {
                        (*tests.second)(*a, *b, *data);
                    }
                }
            }
        }

        data->resolveAllContacts(restitution);
        data->reset();

        BeginDrawing();
        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(BLACK);
        BeginBlendMode(BLEND_ALPHA);

        for (auto collider : colliders) collider->updateInternals();

        for(int i = 0; i < colliders.size(); i++) {

                cyclone::Collider* collider = colliders[i];

                cyclone::Vector3 randColorVec = cyclone::Vector3::random(i);
                Color randColor;
                randColor.r = randColorVec.x * 256;
                randColor.g = randColorVec.y * 256;
                randColor.b = randColorVec.z * 256;
                randColor.a = 255;

                Color planeColor = WHITE;
                planeColor.a = 50;

                Color color = collider->getType() == cyclone::ColliderType::Plane ? planeColor : randColor;
                switch(collider->getType()) {

                    case cyclone::ColliderType::Sphere: {
                        const cyclone::SphereCollider &sphere = reinterpret_cast<const cyclone::SphereCollider &>(*collider);
                        sphereModel.transform = MatrixScale(sphere.getRadius(), sphere.getRadius(), sphere.getRadius()) * ew::CTR(*collider->getRigidbody()->getTransformMatrix());
                        DrawModel(sphereModel, {0,0,0}, 1, color);
                        //DrawModelWires(sphereModel, {0,0,0}, 1.0f, BLACK);
                        break;
                    }
                    case cyclone::ColliderType::Box: {
                        const cyclone::BoxCollider &box = reinterpret_cast<const cyclone::BoxCollider &>(*collider);
                        cubeModel.transform = MatrixScale(box.getHalfSize().x, box.getHalfSize().y, box.getHalfSize().z) * ew::CTR(*collider->getRigidbody()->getTransformMatrix());
                        DrawModel(cubeModel, {0,0,0}, 1, color);
                        DrawModelWires(cubeModel, {0,0,0}, 1.0f, BLACK);
                        break;
                    }
                    case cyclone::ColliderType::Plane: {
                        const cyclone::PlaneCollider &plane = reinterpret_cast<const cyclone::PlaneCollider &>(*collider);
                        Vector3 normal = {plane.getNormal().x, plane.getNormal().y, plane.getNormal().z};
                        Vector3 up = {0, 1, 0};
                        Vector3 axis = Vector3CrossProduct(up, normal); //axis to rotate around
                        Matrix rotation = MatrixRotate(axis, acosf(Vector3DotProduct(up, normal)));

                        Vector3 pos = {plane.getPosition().x, plane.getPosition().y, plane.getPosition().z};

                        DrawLine3D(pos, pos + normal, RED); //to show normals, was very helpful for debugging
                        DrawModel(planeModel, {0, 0, 0}, 1.0f, color);
                        DrawModelWires(planeModel, {0, 0, 0}, 1.0f, BLACK);
                        break;
                    }
                    default: std::cout << "????" << std::endl; break;
                }
            }

        EndBlendMode();
        EndMode3D();

        DrawFPS(GetScreenWidth() - 128, 16);

        rlImGuiBegin();
        ImGui::Begin("Boxes");
        ImGui::DragFloat("Restitution", &restitution,0.01f);
        if (ImGui::Button("Create Box")) {
            addBoxCollider(cyclone::Vector3(0,0,0), cyclone::Vector3(1,1,1), 0);
        }
        for (int i = 0; i < colliders.size(); i++) {
            if (colliders[i]->getType() == cyclone::ColliderType::Box) {
                //ImGui::DragFloat3("Position [" + std::to_string(i) + "]"), colliders[i]->getPosition().x, 0.01f);
                float pos[] = {colliders[i]->getPosition().x, colliders[i]->getPosition().y, colliders[i]->getPosition().z};
                if (ImGui::DragFloat3(("Pos ["+std::to_string(i)+"]").c_str(), pos, 0.01f)) colliders[i]->getRigidbody()->setPosition(cyclone::Vector3(pos[0], pos[1], pos[2]));
                cyclone::BoxCollider *box = reinterpret_cast<cyclone::BoxCollider *>(colliders[i]);
                float scale[] = {box->getHalfSize().x, box->getHalfSize().y, box->getHalfSize().z};
                if (ImGui::DragFloat3(("Scale ["+std::to_string(i)+"]").c_str(), scale, 0.01f)) box->setHalfSize(cyclone::Vector3(scale[0], scale[1], scale[2]));
            }
        }
        ImGui::End();
        rlImGuiEnd();

        EndDrawing();
    }
    delete data;
    for (auto & collider : colliders) delete collider;
    UnloadModel(sphereModel);
    UnloadModel(cubeModel);
    UnloadModel(planeModel);


    CloseWindow();
}