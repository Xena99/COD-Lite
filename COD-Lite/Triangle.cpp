#include "Triangle.h"

bool PointInTriangle(Vector2 P, Triangle* triangle)
{
    Vector2 A = triangle->vertex[0], B = triangle->vertex[1], C = triangle->vertex[2];

    // Relative to A which is at {0,0}
    Vector2 v0 = Vector2Subtract(B, A);
    Vector2 v1 = Vector2Subtract(C, A);
    Vector2 v2 = Vector2Subtract(P, A);

    float d00 = Vector2DotProduct(v0, v0);
    float d01 = Vector2DotProduct(v1, v0);
    float d11 = Vector2DotProduct(v1, v1);
    float d02 = Vector2DotProduct(v0, v2);
    float d21 = Vector2DotProduct(v1, v2);

    // Area of ABC
    float denom = d00 * d11 - d01 * d01;

    // Lambda calculations
    float lambda2 = (d02 * d11 - d21 * d01) / denom;
    float lambda3 = (d00 * d21 - d02 * d01) / denom;
    float lambda1 = 1 - (lambda2 + lambda3);

    return (lambda1 >= -EPSILON && lambda1 <= 1.0f + EPSILON) &&
           (lambda2 >= -EPSILON && lambda2 <= 1.0f + EPSILON) &&
           (lambda3 >= -EPSILON && lambda3 <= 1.0f + EPSILON);
}

