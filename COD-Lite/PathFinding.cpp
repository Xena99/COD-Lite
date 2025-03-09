#include "Pathfinding.h"

vector<Edge> debugPortals;
vector<Triangle*> debugTrianglePath;
vector<Vector2> apexHistory;

queue<Vector2> PathFinding::FindPath(QuadTree& quadTree, Vector2 start, Vector2 target)
{
    TraceLog(LOG_INFO, "FindPath() called - Start: (%.2f, %.2f) | Target: (%.2f, %.2f)",
        start.x, start.y, target.x, target.y);

    Triangle* startTriangle = quadTree.FindTriangleAtPosition(start);
    if (!startTriangle) {
        return {};
    }

    // Min-heap priority queue (processes lowest fScore first)
    priority_queue<Node, vector<Node>, greater<Node>> openSet;

    //to backtrack the path
    unordered_map<Vector2, pair<Vector2, Triangle*>, Vector2Hash> cameFrom;

    unordered_map<Vector2, float, Vector2Hash> gScore;

    unordered_set<Vector2, Vector2Hash> visitedTriangles;

    gScore[start] = 0;
    //cameFrom[start] = start;
    openSet.push({ start, 0, Utils::EuclideanDistance(start, target) });

    while (!openSet.empty())
    {
        Node current = openSet.top();
        openSet.pop();

        if (Utils::EuclideanDistance(current.position, target) < 0.5f) 
        {
            vector<Vector2> rawPath;
            vector<Triangle*> trianglePath;

            rawPath.push_back(target);
            Triangle* targetTriangle = quadTree.FindTriangleAtPosition(target);
            if (targetTriangle) {
                trianglePath.push_back(targetTriangle);
            }

            // Start backtracking
            for (auto step = current.position; cameFrom.find(step) != cameFrom.end(); step = cameFrom[step].first) {
                rawPath.push_back(step);

                // Also store the corresponding triangle
                Triangle* tri = cameFrom[step].second;
                if (!trianglePath.empty() && trianglePath.back() == tri) continue; // Avoid duplicates
                trianglePath.push_back(tri);
            }

            // Add the start position and its triangle
            rawPath.push_back(start);
            Triangle* startTriangle = quadTree.FindTriangleAtPosition(start);
            if (startTriangle && (trianglePath.empty() || trianglePath.back() != startTriangle)) {
                trianglePath.push_back(startTriangle);
            }

            // Reverse the path
            reverse(rawPath.begin(), rawPath.end());
            reverse(trianglePath.begin(), trianglePath.end()); // Ensure order matches rawPath
            
            debugTrianglePath = trianglePath;

            // Extract Portals
            vector<PathSegment> portals = FindPortals(trianglePath);

            // Apply Funnel Algorithm
            vector<Vector2> smoothPath = FunnelAlgorithm(start, portals, quadTree);

            queue<Vector2> finalPath;
            for (const auto& p : smoothPath) {
                finalPath.push(p);
            }
            return finalPath;
        }


        Triangle* currentTriangle = quadTree.FindTriangleAtPosition(current.position);
        if (!currentTriangle)
        {
            TraceLog(LOG_WARNING, "No triangle found at (%.2f, %.2f). Stopping search.", current.position.x, current.position.y);
            break;
        }

        // Process neighbors **in order of closest to target**
        vector<Triangle*> sortedNeighbors = currentTriangle->neighbors;
        sort(sortedNeighbors.begin(), sortedNeighbors.end(),
            [&](Triangle* a, Triangle* b) {
                float fA = (gScore.count(a->center) ? gScore[a->center] : FLT_MAX) +
                    Utils::EuclideanDistance(a->center, target);
                float fB = (gScore.count(b->center) ? gScore[b->center] : FLT_MAX) +
                    Utils::EuclideanDistance(b->center, target);
                return fA < fB;
            });

        for (Triangle* tri : currentTriangle->neighbors)
        {
            Vector2 nextPos = tri->center;

            if (visitedTriangles.find(nextPos) != visitedTriangles.end())
                continue;

            visitedTriangles.insert(nextPos);

            float newGScore = gScore[current.position] + Vector2Distance(currentTriangle->center, nextPos);

            float heuristic = Utils::EuclideanDistance(nextPos, target);

            float fScore = newGScore + heuristic;

            if (gScore.find(nextPos) == gScore.end() || newGScore < gScore[nextPos])
            {
                gScore[nextPos] = newGScore;
                openSet.push({ nextPos, newGScore, fScore });
                cameFrom[nextPos] = { current.position, currentTriangle };
            }
        }
    }

    return {};  // No valid move
}

std::vector<Vector2> PathFinding::FunnelAlgorithm(Vector2& start,
    std::vector<PathSegment>& portals, QuadTree& quadTree, float agentRadius) {

    std::vector<Vector2> path;
    path.push_back(start);
    apexHistory.push_back(start);

    if (portals.empty())
        return path;

    Vector2 apex = start;
    Vector2 left = portals[0].left;
    Vector2 right = portals[0].right;
    int left_index = 0;
    int right_index = 0;

    for (size_t i = 1; i < portals.size(); ++i) {

        const Vector2& new_left = portals[i].left;
        const Vector2& new_right = portals[i].right;
        Vector2 portalCenter = (new_left + new_right) * 0.5f;

        // Ensure the new left and right points are valid
        if (!quadTree.FindTriangleAtPosition(new_left) || !quadTree.FindTriangleAtPosition(new_right)) {
            TraceLog(LOG_WARNING, "Skipping invalid portal at index %d", (int)i);
            continue;  // Skip bad portal
        }

        bool apexUpdated = false;

        // Check if new left is to the right of the current right
        if (Utils::CrossProduct(new_left - apex, right - apex) < -EPSILON)
        {
            if (Utils::CrossProduct(left - apex, new_left - apex) >= 0.0f) {
                path.push_back(right);
                apexHistory.push_back(right);
                apex = right;
                left = new_left;
                right = new_right;
                left_index = i;
                right_index = i;
                apexUpdated = true;
            }
            else {
                right = new_left;
            }
        }

        // Check if new right is to the left of the current left
        if (Utils::CrossProduct(new_right - apex, left - apex) > EPSILON)
        {
            if (Utils::CrossProduct(right - apex, new_right - apex) <= 0.0f) {
                path.push_back(left);
                apexHistory.push_back(left);
                apex = left;
                left = new_right;
                left_index = i;
                right_index = i;
                apexUpdated = true;
            }
            else {
                left = new_right;
            }
        }

        // Correct Apex using Perpendicular Offset
        if (apexUpdated) {

            if (apexUpdated && quadTree.FindTriangleAtPosition(portalCenter)) {
                path.push_back(portalCenter);
                apexHistory.push_back(portalCenter);
            }

            int next = right_index;
            int prev = right_index;

            // Find next vertex
            for (int j = next + 1; j < portals.size(); j++) {
                if (!Utils::VectorsEqual(portals[j].right, portals[next].right, EPSILON) ||
                    !Utils::VectorsEqual(portals[j].left, portals[next].left, EPSILON)) {
                    next = j;
                    break;
                }
            }

            // Find previous vertex
            for (int j = prev - 1; j >= 0; j--) {
                if (!Utils::VectorsEqual(portals[j].right, portals[prev].right, EPSILON) ||
                    !Utils::VectorsEqual(portals[j].left, portals[prev].left, EPSILON)) {
                    prev = j;
                    break;
                }
            }

            // Calculate angles
            float nextAngle = atan2(portals[next].right.y - portals[right_index].right.y,
                portals[next].right.x - portals[right_index].right.x);
            float prevAngle = atan2(portals[right_index].right.y - portals[prev].right.y,
                portals[right_index].right.x - portals[prev].right.x);

            // Compute the minimum angle distance
            float distance = fmod(nextAngle - prevAngle + M_PI, 2 * M_PI) - M_PI;

            // Compute the perpendicular direction
            float angle = std::round((prevAngle + (distance / 2) + M_PI_2) * 1000.0f) / 1000.0f;
            Vector2 normal = { std::round(cos(angle) * 1000.0f) / 1000.0f, std::round(sin(angle) * 1000.0f) / 1000.0f };
            normal.x *= agentRadius;
            normal.y *= agentRadius;

            Vector2 offsetApex = portals[right_index].right + normal;

            if (quadTree.FindTriangleAtPosition(offsetApex)) {
                if (Vector2Distance(apex, offsetApex) > EPSILON) {
                    path.push_back(offsetApex);
                    apexHistory.push_back(offsetApex);
                }
            }

            apex = portals[right_index].right;
            right_index = next;
        }
    }

    // **Final Position Offset Handling**
    Vector2 finalPosition = portals.back().right;
    if (quadTree.FindTriangleAtPosition(finalPosition)) {
        path.push_back(finalPosition);
        apexHistory.push_back(finalPosition);
    }

    return path;
}

vector<PathFinding::PathSegment> PathFinding::FindPortals(vector<Triangle*>& trianglePath) {
    vector<PathFinding::PathSegment> portals;
    if (trianglePath.empty()) return portals;

    for (size_t i = 0; i < trianglePath.size() - 1; i++) {
        Triangle* current = trianglePath[i];
        Triangle* next = trianglePath[i + 1];

        // Find the shared edge
        optional<Edge> sharedEdgeOpt = Utils::GetSharedEdge(current, next);
        if (!sharedEdgeOpt.has_value()) {
            TraceLog(LOG_WARNING, "No shared edge found between triangles at index %zu and %zu", i, i + 1);
            continue;
        }

        Edge sharedEdge = sharedEdgeOpt.value();

        // Compute midpoint between triangle centers (used for smoothing)
        Vector2 portalMidpoint = (current->center + next->center) * 0.5f;

        Vector2 left, right;
        if (Utils::CrossProduct(current->center, sharedEdge.v1, sharedEdge.v2) < 0) {
            left = sharedEdge.v1;
            right = sharedEdge.v2;
        }
        else {
            left = sharedEdge.v2;
            right = sharedEdge.v1;
        }

        // Adjust left and right toward the portal midpoint (smooth transitions)
        left = (left + portalMidpoint) * 0.5f;
        right = (right + portalMidpoint) * 0.5f;

        // Ensure no duplicate portals
        if (!portals.empty() && portals.back().left == left && portals.back().right == right) {
            continue;
        }

        portals.push_back({ left, right });
        debugPortals.push_back({ left, right });
    }

    return portals;
}
