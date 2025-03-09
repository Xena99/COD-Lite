#pragma once

#include "Triangle.h"
#include "Utils.h"
#include "PCH.h"

struct Bounds
{
	Vector2 min; //bottom-left
	Vector2 max; //top-right
};

struct QuadTreeNode
{
	vector<Triangle*> walkableTriangles;
	Bounds boundingBox;
	QuadTreeNode* children[4];
	QuadTreeNode* parent;
	vector<QuadTreeNode*> adjacentNodes;

	bool isLeafNode()
	{
		return (children[0] == nullptr) ;
	}

	QuadTreeNode()
	{
		parent = nullptr;
		for (int i = 0; i < 4; i++) children[i] = nullptr;
	}

	QuadTreeNode(Bounds bounds) 
	{
		boundingBox = bounds;
		parent = nullptr;
		for (int i = 0; i < 4; i++) children[i] = nullptr;
	}
};

enum Direction {
	TOP,
	BOTTOM,
	LEFT,
	RIGHT
};

class QuadTree
{
	private:
		QuadTreeNode* root;

		bool CheckTriangleInsideBoundingBox(Triangle* triangle, Bounds boundingBox);

		bool SATCollision(Triangle* triangle, Bounds boundingBox);

		void ProjectPolygon(const Vector2* vertices, int vertexCount, const Vector2& axis, float& min, float& max);

		bool CheckPointInBoundingBox(Vector2 point, Bounds boundingBox);

		Triangle* SearchNode(QuadTreeNode* node, Vector2 position);

		void DestroyQuadTree(QuadTreeNode* node);
		
	public:
		QuadTree(Bounds boundingBox);

		QuadTreeNode* GetRoot();

		QuadTreeNode* CreateQuadTreeNode(Bounds boundingBox);

		void BuildQuadTree(QuadTreeNode* node, vector<Triangle*> walkableTriangles, int depth);

		void InsertTriangle(Triangle* triangle, QuadTreeNode* node);

		void RedistributeTriangles(QuadTreeNode* node, vector<Triangle*> walkableTriangles);

		void SubdivideQuadrants(QuadTreeNode* node);

		Triangle* FindTriangleAtPosition(Vector2 position);

		bool IsAdjacentInDirection(QuadTreeNode* node1, QuadTreeNode* node2, Direction dir);

		QuadTreeNode* FindLeafNode(QuadTreeNode* node, Vector2 point);

		void CollectLeafNodes(QuadTreeNode* node, vector<QuadTreeNode*>& leafNodes);

		void PrintQuadTree(QuadTreeNode* node, int depth);

		void ComputeNeighbours();

		vector<QuadTreeNode*> FindAdjacentLeafNodes(QuadTreeNode* leafNode, Direction dir);

		~QuadTree();

};

