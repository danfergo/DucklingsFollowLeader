#include <ducklings_follower/Geometry.h>

#include <math.h>

float Geometry::angleBetween(const Point2f &v1, const Point2f &v2)
{
    float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

    float dot = v1.x * v2.x + v1.y * v2.y;

    float a = dot / (len1 * len2);

    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return M_PI;
    else
        return acos(a); // 0..PI
}


float Geometry::norm(const Point2f &p){
    return sqrt(pow(p.x, 2) + pow(p.y, 2));
}

float Geometry::distance(const Point2f &v1, const Point2f &v2){
    return sqrt(pow(v2.x - v1.x, 2) + pow(v2.y - v1.y, 2));
}

Point2f Geometry::normalized(const Point2f &p){
    float nn = norm(p);
    return Point2f(p.x/nn, p.y/nn);
}
