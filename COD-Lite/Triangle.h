#pragma once

#include "PCH.h"

struct Triangle
{
	Vector2 vertex[3];
	bool isWalkable;
	Vector2 center;
    vector<Triangle*> neighbors; // Cached adjacent triangles

    Triangle(Vector2 v0, Vector2 v1, Vector2 v2, bool walkable = false)
        : isWalkable(walkable)
    {
        vertex[0] = v0;
        vertex[1] = v1;
        vertex[2] = v2;
        ComputeCenter();
    }

    void ComputeCenter()
    {
        Vector2 A = vertex[0], B = vertex[1], C = vertex[2];

        center = (A + B + C) / 3.0f;
    }

};

bool PointInTriangle(Vector2 P, Triangle* triangle); //Use Barycentric
