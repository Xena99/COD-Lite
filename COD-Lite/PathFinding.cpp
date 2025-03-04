#include "Pathfinding.h"

float EuclideanDistance(Vector2 a, Vector2 b)
{
	return sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

struct Vector2Hash {
	size_t operator()(const Vector2& v) const {
		return std::hash<float>()(v.x) ^ (std::hash<float>()(v.y) << 1);
	}
};

optional<Vector2> PathFinding::FindNextMove(QuadTree& quadTree, Vector2 start, Vector2 target)
{
    TraceLog(LOG_INFO, "FindNextMove() called - Start: (%.2f, %.2f) | Target: (%.2f, %.2f)",
        start.x, start.y, target.x, target.y);

    // Min-heap priority queue (processes lowest fScore first)
    priority_queue<Node, vector<Node>, greater<Node>> openSet;
    unordered_map<Vector2, Vector2, Vector2Hash> cameFrom;
    unordered_map<Vector2, float, Vector2Hash> gScore;

    gScore[start] = 0;
    openSet.push({ start, 0, EuclideanDistance(start, target) });

    Vector2 bestReachable = start;
    float bestDistance = EuclideanDistance(start, target);

    unordered_set<Vector2, Vector2Hash> visitedNodes; 

    while (!openSet.empty())
    {
        Node current = openSet.top();
        openSet.pop();

        float distToTarget = EuclideanDistance(current.position, target);

        if (distToTarget < bestDistance)
        {
            bestDistance = distToTarget;
            bestReachable = current.position;
        }

        Triangle* currentTriangle = quadTree.FindTriangleAtPosition(current.position);
        if (!currentTriangle)
        {
            //TraceLog(LOG_WARNING, "No triangle found at (%.2f, %.2f). Stopping search.", current.position.x, current.position.y);
            break;
        }

        // Process neighbors **in order of closest to target**
        vector<Triangle*> sortedNeighbors = currentTriangle->neighbors;
        sort(sortedNeighbors.begin(), sortedNeighbors.end(),
            [&](Triangle* a, Triangle* b) {
                return EuclideanDistance(a->center, target) < EuclideanDistance(b->center, target);
            });

        for (Triangle* tri : sortedNeighbors)
        {
            Vector2 nextPos = tri->center;

            if (visitedNodes.find(nextPos) != visitedNodes.end())
                continue; // Skip visited nodes

            visitedNodes.insert(nextPos);

            float newGScore = gScore[current.position] + Vector2Distance(current.position, nextPos);
            float fScore = newGScore + EuclideanDistance(nextPos, target);

            if (gScore.find(nextPos) == gScore.end() || newGScore < gScore[nextPos])
            {
                gScore[nextPos] = newGScore;
                openSet.push({ nextPos, newGScore, fScore });
                cameFrom[nextPos] = current.position;
            }
        }
    }

    if (bestReachable != start)
    {
        return bestReachable;  // Return next best move
    }

    TraceLog(LOG_ERROR, "Pathfinding failed! No valid path from (%.2f, %.2f)", start.x, start.y);
    return nullopt;  // No valid move
}
