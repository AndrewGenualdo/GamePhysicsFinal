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
    camera.position = {-10.0f, 7.0f, 15.0f };   // Camera position
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

    float restitution = 0.75f;

    Model sphereModel = LoadModelFromMesh(GenMeshSphere(1, 10, 10));
    Model cubeModel = LoadModelFromMesh(GenMeshCube(1, 1, 1));
    Model planeModel = LoadModelFromMesh(GenMeshPlane(10, 10, 10, 10));

    std::vector<cyclone::Collider*> colliders;

    bool reset = true;
    bool debug = false;




    auto *data = new cyclone::CollisionData();
    data->reset();

    typedef bool (*intersectionTest)(const cyclone::Collider &, const cyclone::Collider &);
    typedef int (*collisionTest)(const cyclone::Collider &, const cyclone::Collider &, const cyclone::CollisionData &);
    std::map<int, std::pair<intersectionTest, collisionTest>> testTypes = std::map<int, std::pair<intersectionTest, collisionTest>>();
    #define addTestType(a, b, c, d) testTypes[static_cast<int>(a) & static_cast<int>(b)] = std::make_pair(reinterpret_cast<intersectionTest>(c), reinterpret_cast<collisionTest>(d));
    addTestType(cyclone::ColliderType::Sphere, cyclone::ColliderType::Sphere, cyclone::IntersectionTests::SphereSphere, cyclone::CollisionTests::SphereSphere);
    addTestType(cyclone::ColliderType::Sphere, cyclone::ColliderType::Plane, cyclone::IntersectionTests::SpherePlane, cyclone::CollisionTests::SphereTruePlane);
    addTestType(cyclone::ColliderType::Sphere, cyclone::ColliderType::Box, cyclone::IntersectionTests::SphereBox, cyclone::CollisionTests::SphereBox);


    Vector2 mouseStart = {-1,-1};
    float rotation = 0;
    bool isDragging = false;
    while (!WindowShouldClose()) {

        //reset the scene
        if (reset) {
            for (auto & collider : colliders) delete collider;
            colliders.clear();
            //bowling ball
            auto ball = new cyclone::SphereCollider();
            ball->setRadius(0.3541666666666667f);
            ball->getRigidbody()->setPosition(cyclone::Vector3(0, 0.5f + ball->getRadius(), 0));
            ball->getRigidbody()->setInverseMass(1.0f / 7.26f);
            ball->getRigidbody()->setAcceleration(cyclone::Vector3(0, -10, 0));
            ball->getRigidbody()->setLinearDamping(0.95f);
            ball->getRigidbody()->setAngularDamping(0.95f);
            colliders.push_back(ball);

            #define addBoxCollider(pos, scale, invMass) {auto newBox = new cyclone::BoxCollider(); newBox->getRigidbody()->setPosition(pos); newBox->setHalfSize(scale); newBox->getRigidbody()->setInverseMass(invMass); if(invMass != 0) newBox->getRigidbody()->setAcceleration(cyclone::Vector3(0, 0, 0)); colliders.push_back(newBox);}

            //all the box colliders for the world
            addBoxCollider(cyclone::Vector3(0, 0, -29), cyclone::Vector3(3.5, 1, 60), 0);
            addBoxCollider(cyclone::Vector3(0, -0.55, -29), cyclone::Vector3(5.6, 0.1, 60), 0);
            addBoxCollider(cyclone::Vector3(2.75, 0, -29), cyclone::Vector3(0.1, 1, 60), 0);
            addBoxCollider(cyclone::Vector3(-2.75, 0, -29), cyclone::Vector3(0.1, 1, 60), 0);
            addBoxCollider(cyclone::Vector3(-1.5, 1.125, -58.7292), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(-0.5, 1.125, -58.7292), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(0.5, 1.125, -58.7292), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(1.5, 1.125, -58.7292), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(-1, 1.125, -57.8646), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(0, 1.125, -57.8646), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(1, 1.125, -57.8646), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(-0.5, 1.125, -57), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(0.5, 1.125, -57), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);
            addBoxCollider(cyclone::Vector3(0, 1.125, -56.1354), cyclone::Vector3(0.395833, 1.25, 0.395833), 1.0f / 1.64f);

            camera.target = {colliders[colliders.size() - 1]->getPosition().x, colliders[colliders.size() - 1]->getPosition().y, colliders[colliders.size() - 1]->getPosition().z};
            reset = false;
        }

        cyclone::SphereCollider *ball = reinterpret_cast<cyclone::SphereCollider *>(colliders[0]);


        float deltaTime = GetFrameTime();

        //toggle debug mode
        if (IsKeyPressed(KEY_F3)) debug = !debug;

        //camera zoom in for dramatic effect
        if (ball->getPosition().z < -30) {
            camera.position = {-10.0f, 7.0f, -35.0f };
        } else {
            camera.position = {-10.0f, 7.0f, 15.0f };
        }

        //handling bowling the ball
        float mouseMult = 0.1f;
        if (ball->getRigidbody()->getVelocity()->squareMagnitude() < 0.01f) {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                mouseStart = GetMousePosition();
                isDragging = true;
                rotation = 0;
            }
            if (isDragging) {
                if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                    Vector2 mouseEnd = GetMousePosition();
                    Vector2 mouseDelta = mouseEnd - mouseStart;
                    ball->getRigidbody()->addImpulse(cyclone::Vector3(-mouseDelta.x * mouseMult, 0, -mouseDelta.y * mouseMult));
                    //ball->getRigidbody()->addTorque(cyclone::Vector3(rotation * 100 / deltaTime, rotation * 100 / deltaTime, rotation * 100 / deltaTime));
                    ball->getRigidbody()->addTorque({0,0,rotation * 1000});
                    isDragging = false;
                }
                else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                    isDragging = false;
                } else rotation += GetMouseWheelMove(); //scroll wheel to adjust how much the ball should spin
            }
        }




        //reset game
        if (IsKeyPressed(KEY_R)) {
            reset = true;
        }
        //reset just the ball
        else if (ball->getPosition().y < -1.1f || ball->getRigidbody()->getVelocity()->squareMagnitude() < 0.01f) {
            ball->getRigidbody()->setPosition(cyclone::Vector3(0, 0.5f + ball->getRadius(), 0));
            ball->getRigidbody()->setVelocity(cyclone::Vector3(0, 0, 0));
            ball->getRigidbody()->setAngularVelocity(cyclone::Vector3(0, 0, 0));
        }

        for (auto & collider : colliders) collider->getRigidbody()->integrate(deltaTime);

        //collisions
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

        //draw colliders
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
                        DrawModelWires(sphereModel, {0,0,0}, 1.0f, BLACK);
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

        //line-up visual (3D)
        if (isDragging) {
            Vector2 mouseEnd = GetMousePosition();
            Vector2 mouseDelta = mouseEnd - mouseStart;
            cyclone::Vector3 ballPos = colliders[0]->getPosition();
            DrawLine3D({ballPos.x, ballPos.y, ballPos.z}, {ballPos.x + -mouseDelta.x * mouseMult, ballPos.y, ballPos.z + -mouseDelta.y * mouseMult}, YELLOW);
        }

        EndBlendMode();
        EndMode3D();

        DrawFPS(GetScreenWidth() - 128, 16);

        //line-up visual (2D)
        if (isDragging) {
            Vector2 midPoint = (GetMousePosition() + mouseStart) * 0.5f;
            midPoint.x += rotation * 10;
            Vector2 *points = new Vector2[3];//{mouseStart, midPoint, GetMousePosition()};
            points[0] = mouseStart;
            points[1] = midPoint;
            points[2] = GetMousePosition();
            DrawSplineBezierQuadratic(points, 3, 5, GREEN); //fill this in with points?
            delete [] points;
        }

        //debug functions for creating the game
        if (debug) {
            //ImGui for creating boxes (to then print)
            rlImGuiBegin();
            ImGui::Begin("Boxes");
            ImGui::DragFloat("Restitution", &restitution,0.01f);
            if (ImGui::Button("Create Box")) {
                addBoxCollider(cyclone::Vector3(0,0,0), cyclone::Vector3(1,1,1), 0);
            }
            for (int i = 0; i < colliders.size(); i++) {
                if (colliders[i]->getType() == cyclone::ColliderType::Box) {
                    float pos[] = {colliders[i]->getPosition().x, colliders[i]->getPosition().y, colliders[i]->getPosition().z};
                    if (ImGui::DragFloat3(("Pos ["+std::to_string(i)+"]").c_str(), pos, 0.01f)) colliders[i]->getRigidbody()->setPosition(cyclone::Vector3(pos[0], pos[1], pos[2]));
                    cyclone::BoxCollider *box = reinterpret_cast<cyclone::BoxCollider *>(colliders[i]);
                    float scale[] = {box->getHalfSize().x, box->getHalfSize().y, box->getHalfSize().z};
                    if (ImGui::DragFloat3(("Scale ["+std::to_string(i)+"]").c_str(), scale, 0.01f)) box->setHalfSize(cyclone::Vector3(scale[0], scale[1], scale[2]));
                }
            }
            ImGui::End();
            rlImGuiEnd();

            //code creation for making box colliders
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
        }
        EndDrawing();
    }
    delete data;
    for (auto & collider : colliders) delete collider;
    UnloadModel(sphereModel);
    UnloadModel(cubeModel);
    UnloadModel(planeModel);


    CloseWindow();
}