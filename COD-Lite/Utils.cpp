#include "Utils.h"


float Utils::EuclideanDistance(Vector2 a, Vector2 b)
{
    return sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

Vector2 Utils::GetBestEntryPoint(Triangle* tri, Vector2 target) {
    Vector2 bestPoint;
    float minDist = FLT_MAX;

    for (Vector2 edgePoint : tri->GetEdgeMidpoints()) { // Get midpoints of all edges
        float dist = EuclideanDistance(edgePoint, target);
        if (dist < minDist) {
            minDist = dist;
            bestPoint = edgePoint;
        }
    }

    // Fallback: If no better edge found, use center
    if (minDist == FLT_MAX) {
        bestPoint = tri->center;
    }

    return bestPoint;
}


float Utils::TriangleArea(Vector2 a, Vector2 b, Vector2 c) {
    return 0.5f * fabsf((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
}

float Utils::CrossProduct(const Vector2& a, const Vector2& b, const Vector2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

float Utils::CrossProduct(const Vector2& a, const Vector2& b) {
    return (a.x * b.y) - (a.y * b.x);
}

bool Utils::TrianglesAreAdjacent(const Triangle& t1, const Triangle& t2) {
    return GetSharedEdge(&t1, &t2).has_value();
}

optional<Edge> Utils::GetSharedEdge(const Triangle* t1, const Triangle* t2) {
    for (const Edge& edge1 : t1->edges) {
        for (const Edge& edge2 : t2->edges) {
            if (edge1 == edge2) {
                return edge1; // Found the shared edge
            }
        }
    }
    return std::nullopt; // No shared edge
}

bool Utils::VectorsEqual(const Vector2& v1, const Vector2& v2, float epsilon) {
    return (fabs(v1.x - v2.x) < epsilon) && (fabs(v1.y - v2.y) < epsilon);
}
