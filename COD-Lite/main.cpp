#include "Model.h"
#include "Camera.h"
#include "Animation.h"
#include "NavMeshUtils.h"
#include "AIEngine.h"

#include "PCH.h"
#include "Triangle.h"
#include "QuadTree.h"

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
    
    AIEngine aiEngine (quadTree, aiPosition);

    aiEngine.initialTriangles = walkableTriangles;
    aiEngine.SetTarget({ -5.0f, 5.0f });

    aiEngine.StartEngine();

    while (!WindowShouldClose()) 
    {
        // Update animation
        animation.Update(npc.model);

        // Draw
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(gameCamera.camera);

        // Draw Model
        DrawModel(obstacle.model, { 0,0,0 }, 2.0f, WHITE);
        DrawModel(ground.model, { 0,0,0 }, 3.0f, DARKGRAY);
        DrawModelEx(npc.model, { aiEngine.GetAiPosition().x, 0.1f, aiEngine.GetAiPosition().y}, {1.0f, 0.0f, 0.0f}, 90.0f, {0.01f, 0.01f, 0.01f}, WHITE);

        aiEngine.DrawNavMesh();

        DrawSphere({ aiEngine.aiPosition.x, 0.2f, aiEngine.aiPosition.y }, 0.1f, BLACK);

        EndMode3D();

        // UI Debug Text
        DrawText("Default Animation: Idle", 10, 10, 20, BLACK);
        DrawFPS(10, 30);

        EndDrawing();
        //std::this_thread::sleep_for(std::chrono::seconds(1));
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
