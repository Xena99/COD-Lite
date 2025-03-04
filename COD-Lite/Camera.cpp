#include "Camera.h"

GameCamera::GameCamera()
{
    camera.position = { 1.0f, 4.0f, 15.0f };
    camera.target = { 0.0f, 2.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

void GameCamera::Update()
{
    UpdateCamera(&camera, CAMERA_FREE);
}