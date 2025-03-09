#pragma once

#include "PCH.h"
#include "QuadTree.h"
#include "Utils.h"

extern vector<Edge> debugPortals;
extern vector<Triangle*> debugTrianglePath;
extern vector<Vector2> apexHistory;

namespace PathFinding
{
	struct Node
	{
		Vector2 position;
		float gCost, HCost;
		float GetFCost() const { return gCost + HCost; }
		bool operator>(const Node& other) const {
			return GetFCost() > other.GetFCost();
		}
	};

	struct PathSegment {
		Vector2 left;
		Vector2 right;
		PathSegment() {}
		PathSegment(Vector2 left, Vector2 right) : left(left), right(right) {}

	};

	queue<Vector2> FindPath(QuadTree& quadTree, Vector2 start, Vector2 target);

	vector<Vector2> FunnelAlgorithm(Vector2& start,
		std::vector<PathSegment>& portals, QuadTree& quadTree, float agentRadius = 0.5f);

	vector<PathSegment> FindPortals(vector<Triangle*>& trianglePath);

}

