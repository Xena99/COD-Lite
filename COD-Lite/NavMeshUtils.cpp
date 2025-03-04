#include "NavMeshUtils.h"

//Store NavMesh parameters globally
GameModel* NavMeshUtils::groundModel = nullptr;
GameModel* NavMeshUtils::obstacleModel = nullptr;
float NavMeshUtils::gridSize = 0.5f;
float NavMeshUtils::startHeight = 10.0f;

//Initialize NavMesh parameters
void NavMeshUtils::InitializeNavMesh(GameModel& ground, GameModel& obstacles) {
    groundModel = &ground;
    obstacleModel = &obstacles;
}

vector<Triangle*> NavMeshUtils::DetectWalkableTriangles(Vector2 aiPosition, float updateRadius) {
    vector<Triangle*> walkableTriangles;

    if (!groundModel || !obstacleModel) {
        TraceLog(LOG_WARNING, "NavMeshUtils: Ground or Obstacle model is NULL!");
        return walkableTriangles;
    }

    float startX = aiPosition.x - updateRadius;
    float startZ = aiPosition.y - updateRadius;
    float endX = aiPosition.x + updateRadius;
    float endZ = aiPosition.y + updateRadius;

    for (float x = startX; x < endX; x += gridSize) {
        for (float z = startZ; z < endZ; z += gridSize) {
            Vector3 rayStart = { x + gridSize * 0.5f, startHeight, z + gridSize * 0.5f };
            Vector3 rayDir = { 0, -1, 0 };
            Ray ray = { rayStart, rayDir };

            RayCollision groundHit = GetRayCollisionMesh(ray, groundModel->model.meshes[0], MatrixIdentity());
            RayCollision obstacleHit = GetRayCollisionMesh(ray, obstacleModel->model.meshes[0], MatrixIdentity());

            if (groundHit.hit && (!obstacleHit.hit || groundHit.distance < obstacleHit.distance)) {
                Vector2 v0 = { x, z };
                Vector2 v1 = { x + gridSize, z };
                Vector2 v2 = { x, z + gridSize };
                Vector2 v3 = { x + gridSize, z + gridSize };

                Triangle* t1 = new Triangle(v0, v1, v3, true);
                Triangle* t2 = new Triangle(v0, v3, v2, true);

                walkableTriangles.push_back(t1);
                walkableTriangles.push_back(t2);


            }
        }
    }

    TraceLog(LOG_INFO, "INFO: Detected %d walkable triangles", walkableTriangles.size());

    return walkableTriangles;
}

void NavMeshUtils::DrawWalkableTriangles(vector<Triangle*> walkableTriangles, Color color, float heightOffset) {
    for (auto& tri : walkableTriangles) {
        //Convert 2D (XZ) triangle to 3D (XYZ)
        Vector3 v0 = { tri->vertex[0].x, heightOffset, tri->vertex[0].y };
        Vector3 v1 = { tri->vertex[1].x, heightOffset, tri->vertex[1].y };
        Vector3 v2 = { tri->vertex[2].x, heightOffset, tri->vertex[2].y };

        //Draw filled triangle in 3D
        DrawTriangle3D(v0, v1, v2, color);

        //Draw triangle outline for visibility
        DrawLine3D(v0, v1, BLACK);
        DrawLine3D(v1, v2, BLACK);
        DrawLine3D(v2, v0, BLACK);
    }
}

void NavMeshUtils::DrawNavMesh(QuadTree* quadTree, float heightOffset) {
    if (!quadTree) return;

    function<void(QuadTreeNode*)> DrawNode = [&](QuadTreeNode* node) {
        if (!node) return;

        // Draw all triangles inside this node
        for (Triangle* tri : node->walkableTriangles) {
            Vector3 v0 = { tri->vertex[0].x, heightOffset, tri->vertex[0].y };
            Vector3 v1 = { tri->vertex[1].x, heightOffset, tri->vertex[1].y };
            Vector3 v2 = { tri->vertex[2].x, heightOffset, tri->vertex[2].y };

            // Color based on validity
            Color triangleColor = tri->isWalkable ? DARKGREEN : RED;

            // Draw filled triangle
            DrawTriangle3D(v0, v1, v2, triangleColor);

            // Draw outline for visibility
            DrawLine3D(v0, v1, BLACK);
            DrawLine3D(v1, v2, BLACK);
            DrawLine3D(v2, v0, BLACK);
        }

        // Recursively draw child nodes
        for (int i = 0; i < 4; i++) {
            if (node->children[i]) {
                DrawNode(node->children[i]);
            }
        }
        };

    DrawNode(quadTree->GetRoot());
}


