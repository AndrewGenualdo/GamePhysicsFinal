#pragma once

#include <raylib.h>
namespace ew {
	/// <summary>
	/// Fly camera movement with controls:
	/// W/S: forward/back
	/// A/D: left/right
	/// Q/E: down/up
	/// Left shift: Sprint
	/// </summary>
	/// <param name="camera"></param>
	/// <param name="deltaTime"></param>
	/// <param name="moveSpeed"></param>
	/// <param name="lookSensitivity"></param>
	void UpdateFlyCamera(Camera* camera, float deltaTime, float moveSpeed = 10.0f, float lookSensitivity = 0.2f) {
		Vector3 movement = { 0,0,0 };
		Vector3 rotation = { 0,0,0 };
		//Moving forward/back
		if (IsKeyDown(KEY_W)) {
			movement.x ++;
		}
		if (IsKeyDown(KEY_S)) {
			movement.x--;
		}
		//Moving right/left
		if (IsKeyDown(KEY_A)) {
			movement.y--;
		}
		if (IsKeyDown(KEY_D)) {
			movement.y++;
		}
		//Moving up and down
		if (IsKeyDown(KEY_Q)) {
			movement.z--;
		}
		if (IsKeyDown(KEY_E)) {
			movement.z++;
		}
		float moveStep = moveSpeed * deltaTime;
		//Sprinting
		if (IsKeyDown(KEY_LEFT_SHIFT)) {
			moveStep *= 2.0f;
		}
		movement = Vector3Scale(movement, moveStep);
		rotation.x = GetMouseDelta().x * lookSensitivity;
		rotation.y = GetMouseDelta().y * lookSensitivity;

		//Raylib UpdateCameraPro function expects the values in this order, for some reason.
		// Required values
		// movement.x - Move forward/backward
		// movement.y - Move right/left
		// movement.z - Move up/down
		// rotation.x - yaw
		// rotation.y - pitch
		// rotation.z - roll
		// zoom - Move towards target
		UpdateCameraPro(camera, movement, rotation, -GetMouseWheelMove());
	}
}