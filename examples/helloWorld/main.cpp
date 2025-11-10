#include <iostream>

#include <raylib.h>
#include <raymath.h> //Needed for most Raylib math functions
#include <cyclone/cyclone.h>
#include <imgui.h>
#include <rlImGui.h>

#include <ew/camera.h>

struct AppSettings {
    float spinSpeed = 2.0f; //Radians per second
    Vector3 position; //Position of the cube
}_appSettings;

int main() {
    // Initialization
   //--------------------------------------------------------------------------------------
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 720, "Hello World");

    rlImGuiSetup(true);

    Camera3D camera = { 0 };
    camera.position = {0.0f, 10.0f, 15.0f };   // Camera position
    camera.target = { 0.0f, 0.0f, 0.0f };       // Camera looking at point
    camera.up = { 0.0f, 1.0f, 0.0f };           // Camera up vecto
    camera.fovy = 45.0f;                        // Camera vertical field-of-view in degrees
    camera.projection = CAMERA_PERSPECTIVE;     // Camera projection type - perspective vs orthographic

    //Vector used by the Cyclone physics engine
    cyclone::Vector3 cubePosition(0.0f, 0.0f, 0.0f);

    //Vector used by Raylib
    //Raylib is a C library! There are no constructors or namespaces
    Vector3 rayVector{ 0.0f, 0.0f, 0.0f };

    float cubeYaw = 0.0f;
    Model cubeModel = LoadModelFromMesh(GenMeshCube(2, 2, 2));

    while (!WindowShouldClose()) {
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

        //Spin the cube
        cubeYaw += _appSettings.spinSpeed * deltaTime;

        //Model.transform allows us to modify the model (local->world) matrix directly
        //Concatenation order is left to right
        //Note that Raylib does come with optional operator overloads for C++!
        cubeModel.transform = MatrixRotateY(cubeYaw) * MatrixTranslate(cubePosition.x, cubePosition.y, cubePosition.z);

        //Drawing
        BeginDrawing();

        //3D mode draws objects in right handed vector space
        BeginMode3D(camera);
        ClearBackground(SKYBLUE);

        //Position is zeroed because Model.transform is handling the position for us
        DrawModel(cubeModel, { 0,0,0 }, 1.0f, RED);
        DrawModelWires(cubeModel, { 0,0,0 }, 1.0f, BLACK);

        DrawGrid(100, 1.0f);

        EndMode3D();

        //Outside of 3D mode, we can draw 2D things directly to the screen in pixel coordinates
        DrawCircle(32, 32, 16.f, BLUE);
        DrawText("RayLib Text!", 64, 16, 32, WHITE);
        DrawFPS(GetScreenWidth() - 128, 16);

        //ImGUI
        //All ImGui calls must be between rlGuiBegin() and rlImGuiEnd()
        rlImGuiBegin();
        ImGui::Begin("SETTINGS");
        ImGui::DragFloat("SpinSpeed", &_appSettings.spinSpeed);
        ImGui::End();

        bool open = true;
        ImGui::ShowDemoWindow(&open);
        rlImGuiEnd();

        EndDrawing();
    }
    UnloadModel(cubeModel);
    CloseWindow();
}