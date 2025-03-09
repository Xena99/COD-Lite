#pragma once

#include "PCH.h"
#include "QuadTree.h"
#include "Pathfinding.h"
#include "NavMeshUtils.h"
#include "Utils.h"
#include "Animation.h"

class AIEngine {
public:

    AIEngine(QuadTree* quadTree, Vector2 startPosition, AnimationHandler& animHandler);
    ~AIEngine();

    void StartEngine();
    void StopEngine();
    void SetTarget(Vector2 newTarget);
    void DrawNavMesh();

    void UpdateAIMovement();

    Vector2 Seek(Vector2 current, Vector2 target, float speed);

    Vector2 ObstacleAvoidance(Vector2 position, Vector2 velocity);

    Vector2 Limit(Vector2 v, float max);

    std::atomic<bool> targetReached = false; // Add this flag

    Vector2 aiPosition;
    vector<Triangle*> initialTriangles;
    float rotationAngle = 0.0f;

private:
    void AIThreadLoop();

    thread aiThread;
    atomic<bool> running;
    mutex pathMutex;
    condition_variable pathCondition;
    atomic<bool> pathNeedsUpdate = false;

    queue<Vector2> pathQueue;
    QuadTree* quadTree;
    Vector2 target;
    AnimationHandler& animationHandler;

    float movementSpeed = 1.0f;
    bool isWalking = false;


};
