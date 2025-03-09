#include "Model.h"
#include "Camera.h"
#include "Animation.h"
#include "NavMeshUtils.h"
#include "AIEngine.h"
#include "Pathfinding.h"

#include "PCH.h"
#include "Triangle.h"
#include "QuadTree.h"

void DrawDefaultAxes()
{
    // X-Axis (Red)
    DrawLine3D({ 0, 2, 0 }, { 5, 0, 0 }, RED);
    // Y-Axis (Green)
    DrawLine3D({ 0, 2, 0 }, { 0, 5, 0 }, GREEN);
    // Z-Axis (Blue)
    DrawLine3D({ 0, 2, 0 }, { 0, 0, 5 }, BLUE);
}

int main() {
    // Initialize Window
    InitWindow(1000, 800, "Raylib 3D NPC");
    SetTargetFPS(60);
    rlDisableBackfaceCulling();

    // Create Game Camera and Model
    GameCamera gameCamera;
    GameModel npc("resources/NPC/NPC.glb", "resources/NPC/NpcTexture.png");
    GameModel ground("resources/NPC/Scene.glb", "resources/NPC/ground.png");
    GameModel obstacle("resources/NPC/Obstacle.glb", "resources/NPC/rock.png");

    // Load Animations
    AnimationHandler animation;
    animation.LoadAnimations("resources/NPC/NPC.glb", npc.model);

    float radius = 8.0f;
    Vector2 aiPosition = { 0,0 };
    NavMeshUtils::InitializeNavMesh(ground, obstacle);

    // Initialize QuadTree with bounds
    QuadTree* quadTree = new QuadTree({ { -radius, -radius }, { radius, radius } });

    // Detect Walkable Triangles
    vector<Triangle*> walkableTriangles = NavMeshUtils::DetectWalkableTriangles(aiPosition, radius);

    // Build QuadTree using detected walkable triangles
    quadTree->BuildQuadTree(quadTree->GetRoot(), walkableTriangles, 0);
    quadTree->ComputeNeighbours();
    //quadTree->PrintQuadTree(quadTree->GetRoot(), 0);
    
    AIEngine aiEngine(quadTree, aiPosition, animation);

    aiEngine.initialTriangles = walkableTriangles;
    aiEngine.SetTarget({ 5.0f, 5.0f });

    aiEngine.StartEngine();
    Vector2 newTarget({ 5.0f, 5.0f });

    // Create a 90-degree rotation matrix around the X-axis
    Matrix xRotation = MatrixRotateX(DEG2RAD * 90.0f);

    // Apply rotation to all meshes in the model
    for (int i = 0; i < npc.model.meshCount; i++) {
        npc.model.transform = xRotation;
    }

    while (!WindowShouldClose()) 
    {
        // Update animation
        animation.Update(npc.model);
        aiEngine.UpdateAIMovement();

        if (aiEngine.targetReached) {
            aiEngine.targetReached = false; // Reset flag

            if (!walkableTriangles.empty()) {
                // Pick a random walkable triangle
                int randomIndex = GetRandomValue(0, walkableTriangles.size() - 1);
                Triangle* randomTriangle = walkableTriangles[randomIndex];

                // Set the target at the center of the selected triangle
                newTarget = randomTriangle->center;
                aiEngine.SetTarget(newTarget);

                debugPortals.clear();
                apexHistory.clear();
                debugTrianglePath.clear();

                TraceLog(LOG_INFO, "New Target Set: (%.2f, %.2f)", newTarget.x, newTarget.y);
            }
            else {
                TraceLog(LOG_WARNING, "No walkable triangles found! AI target not set.");
            }
        }

        // Draw
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(gameCamera.camera);

        // Draw Model
        DrawModel(obstacle.model, { 0,0,0 }, 1.0f, WHITE);
        DrawModel(ground.model, { 0,0,0 }, 1.0f, DARKGRAY);
        
        
        DrawModelEx(npc.model,
            { aiEngine.aiPosition.x, 0.1f, aiEngine.aiPosition.y },  // Keep AI above the ground
            { 0, 1, 0 },  // Rotate AI around the Y-axis (turns towards movement)
            -aiEngine.rotationAngle + 90.0f,  // Apply computed rotation
            { 0.01f, 0.01f, 0.01f },  // Scale
            WHITE);



        DrawSphere({ newTarget.x, 0.4f, newTarget.y }, 0.2f, WHITE);

        aiEngine.DrawNavMesh();
        
        EndMode3D();

        // UI Debug Text
        DrawFPS(10, 30);

        EndDrawing();
        
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup
    aiEngine.StopEngine();
    npc.Unload();
    obstacle.Unload();
    ground.Unload();
    animation.Unload();
    CloseWindow();

    return 0;
}
