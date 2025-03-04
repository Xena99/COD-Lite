#pragma once

#include "PCH.h"
#include "QuadTree.h"

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

	optional<Vector2> FindNextMove(QuadTree& quadTree, Vector2 start, Vector2 target);
}

