#pragma once

#include "PCH.h"
#include "QuadTree.h"
#include "Pathfinding.h"
#include "NavMeshUtils.h"

class AIEngine {
public:

    AIEngine(QuadTree* quadTree, Vector2 startPosition);
    ~AIEngine();

    void StartEngine();
    void StopEngine();
    void SetTarget(Vector2 newTarget);
    Vector2 GetAiPosition();
    void DrawNavMesh();

    void UpdateAIMovement();

    //to be removed just for debugging
    std::atomic<bool> stepReady = false;
    vector<Triangle*> initialTriangles;

    Vector2 aiPosition;

private:
    void AIThreadLoop();

    thread aiThread;
    atomic<bool> running;
    mutex moveMutex;
    mutex aiStateMutex;
    optional<Vector2> nextMove;

    QuadTree* quadTree;
    Vector2 target;
    std::atomic<bool> targetUpdated = false;
    std::atomic<bool> aiReachedTarget = true;
    float movementSpeed = 0.1f;
    float stoppingThreshold = 0.5f;

};
