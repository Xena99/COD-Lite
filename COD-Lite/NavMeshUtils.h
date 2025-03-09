#pragma once

#include "Triangle.h"
#include "PCH.h"
#include "Model.h"
#include "QuadTree.h"

namespace NavMeshUtils {
    //Declare variables as extern (Defined in `NavMeshUtils.cpp`)
    extern GameModel* groundModel;
    extern GameModel* obstacleModel;
    extern float gridSize;
    extern float startHeight;

    //Function declarations (only declarations, no definitions)
    void InitializeNavMesh(GameModel& ground, GameModel& obstacles);
    std::vector<Triangle*> DetectWalkableTriangles(Vector2 aiPosition, float updateRadius);
    void DrawWalkableTriangles(std::vector<Triangle*> walkableTriangles, Color color, float heightOffset = 0.01f);
   // bool CheckBoxCollisions(BoundingBox box1, BoundingBox box2);
}
