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
    bool useOctree = true;
    bool showOctree = false;
    bool autoSpawn = false;
    float sphereRadius = 0.125f;
    float sphereSpeed = 5.0f;
    float restitution = 1;
    float roomSize = 10;
    float octreeMinSize = 0.5f * roomSize * 0.2f;

    while (!WindowShouldClose()) {
        cyclone::real SPHERE_RADIUS = sphereRadius;//0.125 * 0.75f;
        const int RESTITUTION = restitution;

        Model sphereModel = LoadModelFromMesh(GenMeshSphere(SPHERE_RADIUS, 10, 10));
        Model cubeModel = LoadModelFromMesh(GenMeshCube(1, 1, 1));
        Model planeModel = LoadModelFromMesh(GenMeshPlane(10, 10, 10, 10));

        cyclone::CollisionData *data = new cyclone::CollisionData();
        data->reset();

        std::vector<cyclone::Collider*> colliders;

        const int ROOM_SIZE = roomSize;

        colliders.push_back(new cyclone::PlaneCollider(cyclone::Vector3(0, 0, 0), cyclone::Vector3(0, 1, 0))); //floor
        colliders.push_back(new cyclone::PlaneCollider(cyclone::Vector3(0, ROOM_SIZE, 0), cyclone::Vector3(0, -1, 0))); //ceiling
        colliders.push_back(new cyclone::PlaneCollider(cyclone::Vector3(-ROOM_SIZE * 0.5, ROOM_SIZE * 0.5, 0), cyclone::Vector3(1, 0, 0))); //left wall
        colliders.push_back(new cyclone::PlaneCollider(cyclone::Vector3(ROOM_SIZE * 0.5, ROOM_SIZE * 0.5, 0), cyclone::Vector3(-1, 0, 0))); //right wall
        colliders.push_back(new cyclone::PlaneCollider(cyclone::Vector3(0, ROOM_SIZE * 0.5, -ROOM_SIZE * 0.5), cyclone::Vector3(0, 0, 1))); //back wall
        colliders.push_back(new cyclone::PlaneCollider(cyclone::Vector3(0, ROOM_SIZE * 0.5, ROOM_SIZE * 0.5), cyclone::Vector3(0, 0, -1))); //front wall

        const cyclone::real OCTREE_MAX_SIZE = ROOM_SIZE;
        const cyclone::real OCTREE_MIN_SIZE = octreeMinSize;
        cyclone::real OCTREE_LEAF_SIZE = OCTREE_MAX_SIZE;
        cyclone::real OCTREE_LAYERS = 0;
        while (OCTREE_LEAF_SIZE * 0.5f > OCTREE_MIN_SIZE) {
            OCTREE_LEAF_SIZE *= 0.5f;
            OCTREE_LAYERS++;
        }


        cyclone::OctreeNode<cyclone::Collider*> octree = cyclone::OctreeNode<cyclone::Collider*>(OCTREE_MAX_SIZE, OCTREE_MIN_SIZE);

        bool isSettingChanged = false;
        while (!isSettingChanged && !WindowShouldClose()) {

            auto startTotal = std::chrono::steady_clock::now();

            //Seconds between previous frame and this one
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


            if (autoSpawn && deltaTime < 0.01f) { //100 fps
                cyclone::SphereCollider *newSphere = new cyclone::SphereCollider();
                newSphere->setRadius(SPHERE_RADIUS);
                newSphere->getRigidbody()->setPosition(cyclone::Vector3(0, ROOM_SIZE * 0.5, 0));
                newSphere->getRigidbody()->setVelocity(cyclone::Vector3::random() * sphereSpeed);
                newSphere->getRigidbody()->setMass(1);
                colliders.push_back(newSphere);
            }
            if (IsKeyDown(KEY_B)) {
                cyclone::SphereCollider *newSphere = new cyclone::SphereCollider();
                newSphere->setRadius(SPHERE_RADIUS);
                newSphere->getRigidbody()->setPosition(cyclone::Vector3(0, ROOM_SIZE * 0.5, 0));
                newSphere->getRigidbody()->setVelocity(cyclone::Vector3::random() * sphereSpeed);
                newSphere->getRigidbody()->setMass(1);
                colliders.push_back(newSphere);
            }
            if (IsKeyDown(KEY_R)) {
                for (int i = colliders.size() - 1; i >= 0; i--) {
                    if (colliders[i]->getType() == cyclone::ColliderType::Sphere) {
                        delete colliders[i];
                        colliders[i] = nullptr;
                        colliders.erase(colliders.begin() + i);
                    }
                }
            }

            //Drawing
            BeginDrawing();
            //3D mode draws objects in right handed vector space
            BeginMode3D(camera);
            ClearBackground(BLACK);

            BeginBlendMode(BLEND_ALPHA);
            std::chrono::milliseconds elapsedCollision;
            std::chrono::milliseconds elapsedOctree;
            if (useOctree) {
                for (auto collider : colliders) collider->getRigidbody()->integrate(deltaTime);

                auto startOctree = std::chrono::steady_clock::now();
                std::queue<cyclone::OctreeNode<cyclone::Collider*>*> collisionQueue;
                std::vector<cyclone::Collider*> constants;


                std::set<cyclone::OctreeNode<cyclone::Collider*>*> visited;
                for (int i = 0; i < colliders.size(); i++) {
                    if (colliders[i]->getType() == cyclone::ColliderType::Sphere) {
                        std::vector<std::pair<cyclone::OctreeNode<cyclone::Collider*>*, cyclone::Vector3>> nodes = octree.getNodes(colliders[i]->getPosition(), static_cast<const cyclone::SphereCollider &>(*colliders[i]).getRadius() , cyclone::Vector3(0, ROOM_SIZE * 0.5, 0), OCTREE_MAX_SIZE, OCTREE_MIN_SIZE);
                        for (int j = 0; j < nodes.size(); j++) {
                            nodes[j].first->obj->push_back(&colliders[i]);

                            if (visited.find(nodes[j].first) == visited.end()) {
                                visited.insert(nodes[j].first);
                                collisionQueue.push(nodes[j].first);

                                if (showOctree) {
                                    cubeModel.transform = MatrixScale(OCTREE_LEAF_SIZE, OCTREE_LEAF_SIZE, OCTREE_LEAF_SIZE) * MatrixTranslate(nodes[j].second.x, nodes[j].second.y, nodes[j].second.z);
                                    DrawModelWires(cubeModel, {0, 0, 0}, 1.0f, GREEN);
                                }
                            }
                        }

                    } else {
                        constants.push_back(colliders[i]);
                    }
                }



                auto endOctree = std::chrono::steady_clock::now();
                elapsedOctree = std::chrono::duration_cast<std::chrono::milliseconds>(endOctree - startOctree);

                for (auto collider : colliders) collider->updateInternals();
                auto startCollision = std::chrono::steady_clock::now();
                std::set<std::pair<const cyclone::Collider*, const cyclone::Collider*>> collisions;
                while (!collisionQueue.empty()) {
                    if (collisionQueue.front()->children == nullptr) {
                        int size = collisionQueue.front()->obj->size();
                        for (int i = 0; i < size; i++) {

                            for (int j = 0; j < constants.size(); j++) {
                                cyclone::Collider *a = *collisionQueue.front()->obj->at(i);
                                cyclone::Collider *b = constants[j];

                                if (cyclone::IntersectionTests::SpherePlane(static_cast<const cyclone::SphereCollider &>(*a), static_cast<const cyclone::PlaneCollider &>(*b))) {
                                    cyclone::CollisionTests::SphereTruePlane(static_cast<const cyclone::SphereCollider &>(*a), static_cast<const cyclone::PlaneCollider &>(*b), data);
                                }
                            }

                            for (int j = i + 1; j < size; j++) {
                                cyclone::Collider *a = *collisionQueue.front()->obj->at(i);
                                cyclone::Collider *b = *collisionQueue.front()->obj->at(j);

                                std::pair<const cyclone::Collider*, const cyclone::Collider*> pair(a, b);
                                std::pair<const cyclone::Collider*, const cyclone::Collider*> pair2(b, a);

                                if (collisions.find(pair) != collisions.end() || collisions.find(pair2) != collisions.end()) continue;
                                collisions.insert(pair);

                                if (cyclone::IntersectionTests::SphereSphere(static_cast<const cyclone::SphereCollider &>(*a), static_cast<const cyclone::SphereCollider &>(*b))) {
                                    cyclone::CollisionTests::SphereSphere(static_cast<const cyclone::SphereCollider &>(*a), static_cast<const cyclone::SphereCollider &>(*b), data);
                                }

                                /*if (a->getType() == cyclone::ColliderType::Plane && b->getType() == cyclone::ColliderType::Sphere) {
                                    if (cyclone::IntersectionTests::SpherePlane(static_cast<const cyclone::SphereCollider &>(*b), static_cast<const cyclone::PlaneCollider &>(*a))) {
                                        cyclone::CollisionTests::SphereTruePlane(static_cast<const cyclone::SphereCollider &>(*b), static_cast<const cyclone::PlaneCollider &>(*a), data);
                                    }
                                } else if (a->getType() == cyclone::ColliderType::Sphere && b->getType() == cyclone::ColliderType::Plane) {
                                    if (cyclone::IntersectionTests::SpherePlane(static_cast<const cyclone::SphereCollider &>(*a), static_cast<const cyclone::PlaneCollider &>(*b))) {
                                        cyclone::CollisionTests::SphereTruePlane(static_cast<const cyclone::SphereCollider &>(*a), static_cast<const cyclone::PlaneCollider &>(*b), data);
                                    }
                                } else if (a->getType() == cyclone::ColliderType::Sphere && b->getType() == cyclone::ColliderType::Sphere) {
                                    if (cyclone::IntersectionTests::SphereSphere(static_cast<const cyclone::SphereCollider &>(*a), static_cast<const cyclone::SphereCollider &>(*b))) {
                                        cyclone::CollisionTests::SphereSphere(static_cast<const cyclone::SphereCollider &>(*a), static_cast<const cyclone::SphereCollider &>(*b), data);
                                    }
                                }*/
                            }
                        }
                    }
                    collisionQueue.pop();
                }

                data->resolveAllContacts(RESTITUTION);
                data->reset();
                octree.clearObjects();

                auto endCollision = std::chrono::steady_clock::now();
                elapsedCollision = std::chrono::duration_cast<std::chrono::milliseconds>(endCollision - startCollision);

            } else {

                for (auto collider : colliders) collider->getRigidbody()->integrate(deltaTime);

                auto startCollision = std::chrono::steady_clock::now();

                for(int i = 0; i < colliders.size(); i++) {
                    for (int j = i+1; j < colliders.size(); j++) {
                        if (colliders[i]->getType() == cyclone::ColliderType::Plane && colliders[j]->getType() == cyclone::ColliderType::Sphere) {
                            if (cyclone::IntersectionTests::SpherePlane(static_cast<const cyclone::SphereCollider &>(*colliders[j]), static_cast<const cyclone::PlaneCollider &>(*colliders[i]))) cyclone::CollisionTests::SphereTruePlane(static_cast<const cyclone::SphereCollider &>(*colliders[j]), static_cast<const cyclone::PlaneCollider &>(*colliders[i]), data);
                        } else if (colliders[i]->getType() == cyclone::ColliderType::Sphere && colliders[j]->getType() == cyclone::ColliderType::Plane) {
                            if (cyclone::IntersectionTests::SpherePlane(static_cast<const cyclone::SphereCollider &>(*colliders[i]), static_cast<const cyclone::PlaneCollider &>(*colliders[j]))) cyclone::CollisionTests::SphereTruePlane(static_cast<const cyclone::SphereCollider &>(*colliders[i]), static_cast<const cyclone::PlaneCollider &>(*colliders[j]), data);
                        } else if (colliders[i]->getType() == cyclone::ColliderType::Sphere && colliders[j]->getType() == cyclone::ColliderType::Sphere) {
                            if (cyclone::IntersectionTests::SphereSphere(static_cast<const cyclone::SphereCollider &>(*colliders[i]), static_cast<const cyclone::SphereCollider &>(*colliders[j]))) cyclone::CollisionTests::SphereSphere(static_cast<const cyclone::SphereCollider &>(*colliders[i]), static_cast<const cyclone::SphereCollider &>(*colliders[j]), data);
                        }
                    }
                }
                data->resolveAllContacts(RESTITUTION);
                data->reset();

                auto endCollision = std::chrono::steady_clock::now();
                elapsedCollision = std::chrono::duration_cast<std::chrono::milliseconds>(endCollision - startCollision);

                for (auto collider : colliders) collider->updateInternals();
            }



            auto startRender = std::chrono::steady_clock::now();
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
                        sphereModel.transform = ew::CTR(*collider->getRigidbody()->getTransformMatrix());
                        DrawModel(sphereModel, {0,0,0}, 1, color);
                        //DrawModelWires(sphereModel, {0,0,0}, 1.0f, BLACK);
                        break;
                    }
                    case cyclone::ColliderType::Plane: {
                        const cyclone::PlaneCollider &plane = static_cast<const cyclone::PlaneCollider &>(*collider);
                        Vector3 normal = {plane.getNormal().x, plane.getNormal().y, plane.getNormal().z};
                        Vector3 up = {0, 1, 0};
                        Vector3 axis = Vector3CrossProduct(up, normal); //axis to rotate around
                        Matrix rotation = MatrixRotate(axis, acosf(Vector3DotProduct(up, normal)));

                        Vector3 pos = {plane.getPosition().x, plane.getPosition().y, plane.getPosition().z};
                        planeModel.transform = MatrixScale(ROOM_SIZE * 0.1, ROOM_SIZE * 0.1, ROOM_SIZE * 0.1) * rotation * MatrixTranslate(plane.getPosition().x, plane.getPosition().y, plane.getPosition().z);

                        DrawLine3D(pos, pos + normal, RED); //to show normals, was very helpful for debugging
                        DrawModel(planeModel, {0, 0, 0}, 1.0f, color);
                        DrawModelWires(planeModel, {0, 0, 0}, 1.0f, BLACK);
                        break;
                    }
                    default: std::cout << "????" << std::endl; break;
                }
            }
            auto endRender = std::chrono::steady_clock::now();
            auto elapsedRender = std::chrono::duration_cast<std::chrono::milliseconds>(endRender - startRender);






            EndBlendMode();
            EndMode3D();

            auto endTotal = std::chrono::steady_clock::now();
            auto elapsedTotal = std::chrono::duration_cast<std::chrono::milliseconds>(endTotal - startTotal);

            DrawFPS(GetScreenWidth() - 128, 16);
            rlImGuiBegin();

            ImGui::Begin("Spawner");
            if (ImGui::Button("Sphere [B]")) {
                cyclone::SphereCollider *newSphere = new cyclone::SphereCollider();
                newSphere->setRadius(SPHERE_RADIUS);
                newSphere->getRigidbody()->setPosition(cyclone::Vector3(0, ROOM_SIZE * 0.5f, 0));
                newSphere->getRigidbody()->setVelocity(cyclone::Vector3::random() * 5);
                newSphere->getRigidbody()->setMass(1);
                colliders.push_back(newSphere);
            }
            ImGui::Checkbox("Auto-Spawn", &autoSpawn);
            if (ImGui::Checkbox("Use Octree", &useOctree)) {
                isSettingChanged = true;
            }
            if (useOctree) ImGui::Checkbox("Show Octree", &showOctree);
            if (useOctree) ImGui::Text(("Octree Setup Time: " + std::to_string(elapsedOctree.count()) + "ms").c_str());
            if (useOctree) ImGui::Text(("Octree Collision Time: " + std::to_string(elapsedCollision.count()) + "ms").c_str());
            else ImGui::Text(("Collision Time: " + std::to_string(elapsedCollision.count()) + "ms").c_str());
            ImGui::Text(("Render Time: " + std::to_string(elapsedRender.count()) + "ms").c_str());
            ImGui::Text(("Total Time: " + std::to_string(elapsedTotal.count()) + "ms").c_str());
            ImGui::Text(("Sphere Count: " + std::to_string(colliders.size() - 6)).c_str());
            if (ImGui::DragFloat("Room Size", &roomSize, 1, 10, 100)) {
                isSettingChanged = true;
            }
            if (ImGui::DragFloat("Sphere Radius", &sphereRadius, 0.01f, 0.01f, 100)) {
                isSettingChanged = true;
            }
            if (ImGui::DragFloat("Sphere Speed", &sphereSpeed, 0.1f, 0.01f, 100)) {
                isSettingChanged = true;
            }

            if (useOctree) {
                if (octreeMinSize < roomSize / 40) {
                    octreeMinSize = roomSize / 40;
                }
                if (ImGui::DragFloat("Octree Min Size ", &octreeMinSize, 0.01f, roomSize / 40, roomSize / 4)) {
                    isSettingChanged = true;
                }
                if (ImGui::DragFloat("Restitution", &restitution, 0.01f)) {
                    isSettingChanged = true;
                }
                ImGui::Text(("Octree Leaf Nodes: " + std::to_string(pow(8, OCTREE_LAYERS)) + "").c_str());
                ImGui::Text(("Octree Leaf Size: " + std::to_string(OCTREE_LEAF_SIZE) + "").c_str());
            }

            ImGui::End();
            rlImGuiEnd();

            EndDrawing();
        }
        UnloadModel(sphereModel);
        UnloadModel(cubeModel);
        UnloadModel(planeModel);

        for (auto & collider : colliders) delete collider;
        delete data;
    }



    CloseWindow();
}