#pragma once

#include "PCH.h"

struct Edge {
    Vector2 v1, v2;

    // Edge equality check (order-independent)
    bool operator==(const Edge& other) const {
        return (v1 == other.v1 && v2 == other.v2) || (v1 == other.v2 && v2 == other.v1);
    }

};

// Custom hash function for Edge
namespace std {
    template <>
    struct hash<Edge> {
        size_t operator()(const Edge& edge) const {
            return std::hash<float>()(edge.v1.x) ^ std::hash<float>()(edge.v1.y) ^
                std::hash<float>()(edge.v2.x) ^ std::hash<float>()(edge.v2.y);
        }
    };
}

struct Triangle {
    Vector2 vertex[3];
    bool isWalkable;
    Vector2 center;
    vector<Triangle*> neighbors; // Cached adjacent triangles
    unordered_set<Edge> edges;          // Edges of the triangle
    

    Triangle(Vector2 v0, Vector2 v1, Vector2 v2, bool walkable = false)
        : isWalkable(walkable)
    {
        vertex[0] = v0;
        vertex[1] = v1;
        vertex[2] = v2;
        ComputeCenter();
        GenerateEdges();
    }

    void ComputeCenter() {
        center = (vertex[0] + vertex[1] + vertex[2]) / 3.0f;
    }

    void GenerateEdges() {
        edges = {
            {vertex[0], vertex[1]},
            {vertex[1], vertex[2]},
            {vertex[2], vertex[0]}
        };
    }

    bool HasEdge(const Edge& edge) const {
        return edges.find(edge) != edges.end();
    }

    vector<Vector2> GetEdgeMidpoints() const {
        return {
            { (vertex[0].x + vertex[1].x) / 2.0f, (vertex[0].y + vertex[1].y) / 2.0f },
            { (vertex[1].x + vertex[2].x) / 2.0f, (vertex[1].y + vertex[2].y) / 2.0f },
            { (vertex[2].x + vertex[0].x) / 2.0f, (vertex[2].y + vertex[0].y) / 2.0f }
        };
    }
};

// Check if a point is inside a triangle using the Barycentric method
bool PointInTriangle(Vector2 P, Triangle* triangle);
