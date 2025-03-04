#include "AIEngine.h"
#include "Pathfinding.h"

AIEngine::AIEngine(QuadTree* quadTree, Vector2 startPosition)
    : quadTree(quadTree), aiPosition(startPosition), running(false) {
}

AIEngine::~AIEngine() {
    StopEngine();
}

void AIEngine::StartEngine() {
    running = true;
    aiThread = std::thread(&AIEngine::AIThreadLoop, this);
}

void AIEngine::StopEngine() {
    running = false;
    if (aiThread.joinable()) {
        aiThread.join();
    }
}

void AIEngine::SetTarget(Vector2 newTarget) {
    std::lock_guard<std::mutex> lock(moveMutex);

    if (target.x != newTarget.x || target.y != newTarget.y) {
        target = newTarget;
        targetUpdated = true;
        stepReady = true;
    }
}

void AIEngine::AIThreadLoop() {
    while (running) {
        while (!stepReady) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Poll efficiently
            if (!running) return;  // Exit if the AI thread is stopping
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::lock_guard<std::mutex> aiLock(aiStateMutex);

        if (!target.x && !target.y) continue;

        // Stop when AI reaches target
        if (Vector2Distance(aiPosition, target) < stoppingThreshold) {
            nextMove.reset();
            targetUpdated = false;
            stepReady = false;
            continue;
        }

        if (targetUpdated || !nextMove) {
            std::lock_guard<std::mutex> pathLock(moveMutex);
            nextMove = PathFinding::FindNextMove(*quadTree, aiPosition, target);
            targetUpdated = false;  // Reset update flag
        }

        UpdateAIMovement();

        stepReady = (Vector2Distance(aiPosition, target) >= stoppingThreshold);
    }
}

Vector2 AIEngine::GetAiPosition()
{
    return aiPosition;
}

void AIEngine::DrawNavMesh() {
    std::lock_guard<std::mutex> lock(moveMutex);

    // Find the current triangle AI is in
    Triangle* currentTriangle = quadTree->FindTriangleAtPosition(aiPosition);
    if (!currentTriangle) {
        TraceLog(LOG_ERROR, "No triangle found at AI position (%.2f, %.2f)", aiPosition.x, aiPosition.y);
        return;
    }

    // Draw AI’s current triangle in RED
    Vector3 v0 = { currentTriangle->vertex[0].x, 0.15f, currentTriangle->vertex[0].y };
    Vector3 v1 = { currentTriangle->vertex[1].x, 0.15f, currentTriangle->vertex[1].y };
    Vector3 v2 = { currentTriangle->vertex[2].x, 0.15f, currentTriangle->vertex[2].y };

    DrawTriangle3D(v0, v1, v2, RED);
    DrawLine3D(v0, v1, BLACK);
    DrawLine3D(v1, v2, BLACK);
    DrawLine3D(v2, v0, BLACK);

    // Draw neighboring triangles in YELLOW
    for (Triangle* neighbor : currentTriangle->neighbors) {
        Vector3 nv0 = { neighbor->vertex[0].x, 0.17f, neighbor->vertex[0].y };
        Vector3 nv1 = { neighbor->vertex[1].x, 0.17f, neighbor->vertex[1].y };
        Vector3 nv2 = { neighbor->vertex[2].x, 0.17f, neighbor->vertex[2].y };

        DrawTriangle3D(nv0, nv1, nv2, YELLOW);
        DrawLine3D(nv0, nv1, BLACK);
        DrawLine3D(nv1, nv2, BLACK);
        DrawLine3D(nv2, nv0, BLACK);
    } 

    NavMeshUtils::DrawWalkableTriangles(initialTriangles, DARKGREEN, 0.1f); // Green for initial
}

void AIEngine::UpdateAIMovement() {
    if (!nextMove) return;

    aiPosition = Vector2Lerp(aiPosition, nextMove.value(), movementSpeed);

    if (Vector2Distance(aiPosition, nextMove.value()) < 0.01f) {
        aiPosition = nextMove.value();
        nextMove.reset();
    }
}