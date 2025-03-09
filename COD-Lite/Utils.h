#pragma once
#include "Triangle.h"
#include "PCH.h"


struct Vector2Hash {
	size_t operator()(const Vector2& v) const {
		return std::hash<float>()(v.x) ^ (std::hash<float>()(v.y) << 1);
	}
};

class Utils
{
	public:

		static float EuclideanDistance(Vector2 a, Vector2 b);

		static Vector2 GetBestEntryPoint(Triangle* tri, Vector2 target);

		static float TriangleArea(Vector2 a, Vector2 b, Vector2 c);

		static float CrossProduct(const Vector2& a, const Vector2& b, const Vector2& c);

		static float CrossProduct(const Vector2& a, const Vector2& b);

		static bool TrianglesAreAdjacent(const Triangle& t1, const Triangle& t2);

		static optional<Edge> GetSharedEdge(const Triangle* t1, const Triangle* t2);

		static bool VectorsEqual(const Vector2& v1, const Vector2& v2, float epsilon);

};

