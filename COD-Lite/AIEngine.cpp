#include "AIEngine.h"
#include "Pathfinding.h"

AIEngine::AIEngine(QuadTree* quadTree, Vector2 startPosition, AnimationHandler& animHandler)
    : quadTree(quadTree), aiPosition(startPosition), running(false), animationHandler(animHandler) {
}


AIEngine::~AIEngine() {
    StopEngine();
}

void AIEngine::StartEngine() {
    running = true;
    aiThread = thread(&AIEngine::AIThreadLoop, this);
}

void AIEngine::StopEngine() {
    running = false;
    if (aiThread.joinable()) {
        aiThread.join();
    }
}

//Target updates while NPC patrolling or chasing/sensing 
void AIEngine::SetTarget(Vector2 newTarget) {
    lock_guard<mutex> lock(pathMutex); 

    if (target.x != newTarget.x || target.y != newTarget.y) {
        target = newTarget;
        pathNeedsUpdate = true;
        pathQueue = {};
        pathCondition.notify_one();

        Vector2 direction = Vector2Normalize(Vector2Subtract(target, aiPosition));
        rotationAngle = atan2f(direction.y, direction.x) * RAD2DEG;
    }
}

void AIEngine::AIThreadLoop() {
    while (running) {
        
        unique_lock<mutex> lock(pathMutex);
        //wait until path needs to be updated or running is false
        pathCondition.wait(lock, [this]() {return pathNeedsUpdate.load() || !running; });

        if (!running) return;

        if (!target.x && !target.y) continue;

        lock.unlock();
        queue<Vector2> newPathQueue = PathFinding::FindPath(*quadTree, aiPosition, target);

        lock.lock();
        pathNeedsUpdate = false;

        if (pathNeedsUpdate) continue;

        pathQueue = move(newPathQueue);
    }
}

void AIEngine::DrawNavMesh() {

    // Find the current triangle AI is in
    Triangle* currentTriangle = quadTree->FindTriangleAtPosition(aiPosition);
    if (!currentTriangle) {
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

    if (!pathQueue.empty()) {
        Vector2 prevPos = aiPosition;  // Start from AI's current position
        queue<Vector2> tempPathQueue = pathQueue;  // Copy queue (to avoid modifying original)

        while (!tempPathQueue.empty()) {
            Vector2 nextPos = tempPathQueue.front();
            tempPathQueue.pop();

            Vector3 from = { prevPos.x, 0.2f, prevPos.y };
            Vector3 to = { nextPos.x, 0.2f, nextPos.y };

            DrawLine3D(from, to, BLUE); // Draw path in BLUE
            prevPos = nextPos;
        }
    }
    NavMeshUtils::DrawWalkableTriangles(initialTriangles, DARKGREEN, 0.1f); // Green for initial
}

void AIEngine::UpdateAIMovement()
{
    lock_guard<mutex> lock(pathMutex);

    if (pathQueue.empty())
    {
        animationHandler.PlayAnimation("Idle");
        isWalking = false;
        return;
    }

    Vector2 nextTarget = pathQueue.front();
    float distanceToTarget = Vector2Distance(aiPosition, nextTarget);

    // If AI is very close, snap to position and proceed to the next target
    if (distanceToTarget < 0.1)
    {
        aiPosition = nextTarget;
        pathQueue.pop();

        if (pathQueue.empty()) {
            animationHandler.PlayAnimation("Idle");
            isWalking = false;
            targetReached = true; // Notify main that the AI reached the last point
        }
        return;
    }

    // Move towards the next target with a fixed step
    float step = movementSpeed * GetFrameTime(); // Frame-rate independent movement

    // Compute direction vector and normalize
    Vector2 direction = Vector2Normalize(Vector2Subtract(nextTarget, aiPosition));

    // Calculate new potential position
    Vector2 newPosition = { aiPosition.x + direction.x * step, aiPosition.y + direction.y * step };

    // Check if the new position is within a valid triangle
    Triangle* triangle = quadTree->FindTriangleAtPosition(newPosition);
    if (!triangle)
    {
        pathNeedsUpdate.store(true);

        if (!pathQueue.empty())
            pathQueue = {};

        pathCondition.notify_one();
        return;
    }

    // Only update aiPosition if it's inside a valid triangle
    aiPosition = newPosition;
    if (!isWalking) {
        animationHandler.PlayAnimation("Walk");
        isWalking = true;  // Ensure it only plays once
    }
}
