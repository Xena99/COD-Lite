#include "QuadTree.h"

QuadTree::QuadTree(Bounds boundingBox) {
    root = CreateQuadTreeNode(boundingBox);
}

QuadTreeNode* QuadTree::GetRoot()
{
    return root;
}


QuadTreeNode* QuadTree::CreateQuadTreeNode(Bounds boundingBox)
{
    QuadTreeNode* node = new QuadTreeNode();
    node->boundingBox = boundingBox;
    for (int i = 0; i < 4; i++)
        node->children[i] = nullptr;
    return node;
}

void QuadTree::BuildQuadTree(QuadTreeNode* node, vector<Triangle*> walkableTriangles, int depth)
{
    if (walkableTriangles.size() < MAX_TRIANGLE_PER_NODE || depth > MAX_DEPTH)
    {
        node->walkableTriangles = walkableTriangles;
        return;
    }

    SubdivideQuadrants(node);
    
    RedistributeTriangles(node, walkableTriangles);

    for (int i = 0; i < 4; i++)
    {
        if (node->children[i])
        {
            BuildQuadTree(node->children[i], node->children[i]->walkableTriangles, depth + 1);
        }
    }
}

void QuadTree::InsertTriangle(Triangle* triangle, QuadTreeNode* node)
{
    if (!CheckTriangleInsideBoundingBox(triangle, node->boundingBox))
        return;

        if (node->isLeafNode())
        {
            node->walkableTriangles.push_back(triangle);

            if (node->walkableTriangles.size() > MAX_TRIANGLE_PER_NODE)
            {
                SubdivideQuadrants(node);
                vector<Triangle*> triangles = move(node->walkableTriangles);
                RedistributeTriangles(node, triangles);
            }
            return;
        }
        int firstFit = -1;
        for (int i = 0; i < 4; i++)
        {
            if (node->children[i] && CheckTriangleInsideBoundingBox(triangle, node->children[i]->boundingBox))
            {
                if (firstFit == -1)
                    firstFit = i;
                else
                {
                    firstFit = -1; // Triangle overlaps multiple children, so store in parent
                    break;
                }
            }
        }

        if (firstFit != -1)
        {
            InsertTriangle(triangle, node->children[firstFit]);
        }
        else
            node->walkableTriangles.push_back(triangle); // Store at parent level if overlapping
}

void QuadTree::RedistributeTriangles(QuadTreeNode* node, vector<Triangle*> walkableTriangles)
{
    vector<Triangle*> overlappingTriangles;
    unordered_set<Triangle*> assignedTriangles;  //Track which triangles were moved.

    for (Triangle* triangle : walkableTriangles)
    {
        int firstFit = -1;
        for (int i = 0; i < 4; i++)
        {
            if (node->children[i] && CheckTriangleInsideBoundingBox(triangle, node->children[i]->boundingBox))
            {
                if (firstFit == -1)
                    firstFit = i;
                else
                {
                    firstFit = -1; // Overlaps multiple children, so keep in parent.
                    break;
                }
            }
        }

        if (firstFit != -1)
        {
            //Ensure removal from parent before inserting into child
            node->children[firstFit]->walkableTriangles.push_back(triangle);
            assignedTriangles.insert(triangle);
        }
        else
            overlappingTriangles.push_back(triangle);
    }

    //Remove assigned triangles from the parent’s list
    node->walkableTriangles.erase(std::remove_if(node->walkableTriangles.begin(), node->walkableTriangles.end(),
        [&](Triangle* tri) { return assignedTriangles.find(tri) != assignedTriangles.end(); }), node->walkableTriangles.end());

    node->walkableTriangles = overlappingTriangles;  //Keep only the remaining ones.
}

void QuadTree::SubdivideQuadrants(QuadTreeNode* node) {
    if (!node || !node->isLeafNode()) return;

    Vector2 mid = {
        (node->boundingBox.min.x + node->boundingBox.max.x) / 2,
        (node->boundingBox.min.y + node->boundingBox.max.y) / 2
    };

    //Create child nodes
    node->children[0] = new QuadTreeNode({ node->boundingBox.min, mid });
    node->children[1] = new QuadTreeNode({ { mid.x, node->boundingBox.min.y }, { node->boundingBox.max.x, mid.y } });
    node->children[2] = new QuadTreeNode({ { node->boundingBox.min.x, mid.y }, { mid.x, node->boundingBox.max.y } });
    node->children[3] = new QuadTreeNode({ mid, node->boundingBox.max });

    for (int i = 0; i < 4; i++) {
        if (node->children[i]) {
            node->children[i]->parent = node;
        }
    }

    //Set adjacency between children (siblings)
    node->children[0]->adjacentNodes = { node->children[1], node->children[2] };
    node->children[1]->adjacentNodes = { node->children[0], node->children[3] };
    node->children[2]->adjacentNodes = { node->children[0], node->children[3] };
    node->children[3]->adjacentNodes = { node->children[1], node->children[2] };

    //Set adjacency to parent's adjacent nodes
    for (QuadTreeNode* adj : node->adjacentNodes) {
        for (int i = 0; i < 4; i++) {
            if (adj) {
                node->children[i]->adjacentNodes.push_back(adj);
            }
        }
    }
}


bool QuadTree::CheckTriangleInsideBoundingBox(Triangle* triangle, Bounds boundingBox) {
    // 1. Point Check: Check if any vertex is inside the bounding box.
    for (int i = 0; i < 3; i++) {
        if (!CheckPointInBoundingBox(triangle->vertex[i], boundingBox)) {
            return false;
        }
    }

    // 2. SAT Check: Check if the triangle and bounding box intersect using SAT.
    return SATCollision(triangle, boundingBox);
}

bool QuadTree::SATCollision(Triangle* triangle, Bounds boundingBox) {
    // Convert bounding box to a vector of Vector2 vertices.
    Vector2 boundingBoxVertices[] = {
        {boundingBox.min.x, boundingBox.min.y},
        {boundingBox.max.x, boundingBox.min.y},
        {boundingBox.max.x, boundingBox.max.y},
        {boundingBox.min.x, boundingBox.max.y}
    };

    // Get the edges of the triangle and bounding box.
    std::vector<Vector2> edges;
    for (int i = 0; i < 3; i++) {
        edges.push_back(Vector2Subtract(triangle->vertex[(i + 1) % 3], triangle->vertex[i]));
    }
    for (int i = 0; i < 4; i++) {
        edges.push_back(Vector2Subtract(boundingBoxVertices[(i + 1) % 4], boundingBoxVertices[i]));
    }

    // Get the normals of the edges.
    std::vector<Vector2> normals;
    for (const Vector2& edge : edges) {
        normals.push_back({ -edge.y, edge.x }); // Perpendicular vector
    }

    // Project the triangle and bounding box onto each normal.
    for (const Vector2& normal : normals) {
        float triangleMin, triangleMax;
        ProjectPolygon(triangle->vertex, 3, normal, triangleMin, triangleMax);

        float boxMin, boxMax;
        ProjectPolygon(boundingBoxVertices, 4, normal, boxMin, boxMax);

        // Check for separation.
        if (triangleMax < boxMin || boxMax < triangleMin) {
            return false; // Separating axis found, no collision.
        }
    }

    return true; // No separating axis found, collision.
}

void QuadTree::ProjectPolygon(const Vector2* vertices, int vertexCount, const Vector2& axis, float& min, float& max) {
    min = Vector2DotProduct(vertices[0], axis);
    max = min;

    for (int i = 1; i < vertexCount; i++) {
        float projection = Vector2DotProduct(vertices[i], axis);
        min = std::min(min, projection);
        max = std::max(max, projection);
    }
}

bool QuadTree::CheckPointInBoundingBox(Vector2 point, Bounds boundingBox) {
    return (point.x >= boundingBox.min.x && point.x <= boundingBox.max.x) &&
        (point.y >= boundingBox.min.y && point.y <= boundingBox.max.y);
}

QuadTree::~QuadTree()
{
    DestroyQuadTree(root);
}

void QuadTree::DestroyQuadTree(QuadTreeNode* node)
{
    if (!node) return;

    for (int i = 0; i < 4; i++)
        DestroyQuadTree(node->children[i]);  // Recursively delete children

    delete node;
    node = nullptr;
}

Triangle* QuadTree::FindTriangleAtPosition(Vector2 position) {
    // Check if the position is within the root's bounding box.
    if (!CheckPointInBoundingBox(position, root->boundingBox)) 
    {
        return nullptr; // Point is outside the navmesh.
    }
    return SearchNode(root, position);
}

Triangle* QuadTree::SearchNode(QuadTreeNode* node, Vector2 position) {
    if (!node) return nullptr;

    // If it's a leaf node, check its triangles
    if (node->isLeafNode()) {
        Triangle* bestTriangle = nullptr;
        float bestDistance = FLT_MAX; // Store the closest triangle

        for (Triangle* tri : node->walkableTriangles) {
            if (PointInTriangle(position, tri)) {
                float dist = Vector2Distance(position, tri->center);
                if (dist < bestDistance) { // Select the closest triangle
                    bestDistance = dist;
                    bestTriangle = tri;
                }
            }
        }
        return bestTriangle; // Return the best (closest) triangle found
    }

    // Recursively search child nodes
    for (int i = 0; i < 4; i++) {
        if (node->children[i]) {
            Triangle* foundTriangle = SearchNode(node->children[i], position);
            if (foundTriangle) return foundTriangle;
        }
    }

    return nullptr; // No valid triangle found
}

void QuadTree::ComputeNeighbours() {
    unordered_map<Triangle*, vector<Triangle*>> newNeighbors;
   // TraceLog(LOG_INFO, "Starting ComputeNeighbours()...");

    function<void(QuadTreeNode*)> FindNeighbours = [&](QuadTreeNode* node) {
        if (!node) return; // Only return if node is null

        // Process child nodes first (recursive call)
        for (int i = 0; i < 4; i++) {
            if (node->children[i]) {
                FindNeighbours(node->children[i]);
            }
        }

        for (Triangle* triangle : node->walkableTriangles) {
            unordered_set<Triangle*> uniqueNeighbors;

            //TraceLog(LOG_INFO, "Processing Triangle at (%.2f, %.2f)",triangle->center.x, triangle->center.y);

            // Check neighbors within the same leaf node
            for (Triangle* other : node->walkableTriangles) {
                if (triangle != other && Utils::TrianglesAreAdjacent(*triangle, *other)) {
                    //TraceLog(LOG_INFO, "  Found adjacent triangle in same node: (%.2f, %.2f)", other->center.x, other->center.y);
                    uniqueNeighbors.insert(other);
                }
            }

            // Check neighbors in adjacent leaf nodes (TOP, BOTTOM, LEFT, RIGHT)
            for (Direction dir : {TOP, BOTTOM, LEFT, RIGHT}) {
                vector<QuadTreeNode*> adjacentNodes = FindAdjacentLeafNodes(node, dir);
                for (QuadTreeNode* adjacentNode : adjacentNodes) {
                    if (!adjacentNode || !adjacentNode->isLeafNode()) continue;

                   // TraceLog(LOG_INFO, "  Checking adjacent node in direction %d with %d triangles", dir, (int)adjacentNode->walkableTriangles.size());

                    for (Triangle* other : adjacentNode->walkableTriangles) {
                        if (Utils::TrianglesAreAdjacent(*triangle, *other)) 
                        {
                            uniqueNeighbors.insert(other);
                            //TraceLog(LOG_INFO, "    Found adjacent triangle in neighbor node: (%.2f, %.2f)", other->center.x, other->center.y);
                        }
                    }
                }
            }

            newNeighbors[triangle] = vector<Triangle*>(uniqueNeighbors.begin(), uniqueNeighbors.end());
        }
        };

    FindNeighbours(root);

    // Update actual triangle neighbors
    for (auto& entry : newNeighbors) {
        entry.first->neighbors = entry.second;
    }
}

vector<QuadTreeNode*> QuadTree::FindAdjacentLeafNodes(QuadTreeNode* leafNode, Direction dir) {
    vector<QuadTreeNode*> adjacentLeafNodes;

    QuadTreeNode* parent = leafNode->parent;
    if (!parent) return adjacentLeafNodes;

    while (parent) {  // Traverse upwards to find correct neighbor branches
        for (QuadTreeNode* sibling : parent->children) {
            if (!sibling || sibling == leafNode) continue;

            if (IsAdjacentInDirection(leafNode, sibling, dir)) {
                if (sibling->isLeafNode()) {
                    adjacentLeafNodes.push_back(sibling);
                }
                else {
                    CollectLeafNodes(sibling, adjacentLeafNodes);
                }
            }
        }
        parent = parent->parent; // Move up the tree
    }

    return adjacentLeafNodes;
}

bool QuadTree::IsAdjacentInDirection(QuadTreeNode* node1, QuadTreeNode* node2, Direction dir) {

    switch (dir) {
    case TOP:
        return fabs(node1->boundingBox.max.y - node2->boundingBox.min.y) < EPSILON &&
            node1->boundingBox.min.x < node2->boundingBox.max.x &&
            node1->boundingBox.max.x > node2->boundingBox.min.x;
    case BOTTOM:
        return fabs(node1->boundingBox.min.y - node2->boundingBox.max.y) < EPSILON &&
            node1->boundingBox.min.x < node2->boundingBox.max.x &&
            node1->boundingBox.max.x > node2->boundingBox.min.x;
    case LEFT:
        return fabs(node1->boundingBox.min.x - node2->boundingBox.max.x) < EPSILON &&
            node1->boundingBox.min.y < node2->boundingBox.max.y &&
            node1->boundingBox.max.y > node2->boundingBox.min.y;
    case RIGHT:
        return fabs(node1->boundingBox.max.x - node2->boundingBox.min.x) < EPSILON &&
            node1->boundingBox.min.y < node2->boundingBox.max.y &&
            node1->boundingBox.max.y > node2->boundingBox.min.y;
    default:
        return false;
    }
}

QuadTreeNode* QuadTree::FindLeafNode(QuadTreeNode* node, Vector2 point) {
    if (!node) return nullptr;

    // If the point is outside the current node's bounding box, return nullptr.
    if (point.x < node->boundingBox.min.x || point.x > node->boundingBox.max.x ||
        point.y < node->boundingBox.min.y || point.y > node->boundingBox.max.y) {
        return nullptr;
    }

    // If this node is a leaf node, return it.
    if (node->isLeafNode()) {
        return node;
    }

    // Recursively search the appropriate child node.
    for (int i = 0; i < 4; i++) {
        if (node->children[i]) {
            QuadTreeNode* found = FindLeafNode(node->children[i], point);
            if (found) return found; // Return the found node immediately.
        }
    }

    return nullptr; // Point not found in any leaf node (shouldn't happen if point is within the root's bounds).
}

void QuadTree::CollectLeafNodes(QuadTreeNode* node, vector<QuadTreeNode*>& leafNodes) {
    if (!node) return;

    if (node->isLeafNode()) {
        leafNodes.push_back(node);
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (node->children[i]) {
            CollectLeafNodes(node->children[i], leafNodes);
        }
    }
}


void QuadTree::PrintQuadTree(QuadTreeNode* node, int depth) {
    if (!node) return;

    // Indentation for depth visualization
    string indent(depth * 2, ' ');

    TraceLog(LOG_INFO, "%sDepth %d | Min(%.2f, %.2f) Max(%.2f, %.2f) | Triangles: %d",
        indent.c_str(), depth,
        node->boundingBox.min.x, node->boundingBox.min.y,
        node->boundingBox.max.x, node->boundingBox.max.y,
        (int)node->walkableTriangles.size());

    // Print assigned triangles in this node
    for (Triangle* triangle : node->walkableTriangles) {
        TraceLog(LOG_INFO, "%s Triangle at Center: (%.2f, %.2f) | V0(%.2f, %.2f) V1(%.2f, %.2f) V2(%.2f, %.2f)",
            indent.c_str(),
            triangle->center.x, triangle->center.y,
            triangle->vertex[0].x, triangle->vertex[0].y,
            triangle->vertex[1].x, triangle->vertex[1].y,
            triangle->vertex[2].x, triangle->vertex[2].y);
    }

    // Recursively print children
    for (int i = 0; i < 4; i++) {
        if (node->children[i]) {
            PrintQuadTree(node->children[i], depth + 1);
        }
    }
}