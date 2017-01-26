#ifndef __GEOMETRY_CPP__
#define __GEOMETRY_CPP__

#include <opencv2/core/core.hpp>

using namespace cv;

class Geometry{

public:
    static double angleBetween(const Point2d &v1, const Point2d &v2);
    static double angleBetween2(const Point2d &v1, const Point2d &v2);
    static double norm(const Point2d &p);
    static double distance(const Point2d &v1, const Point2d &v2);
    static Point2d normalized(const Point2d &p);
    static Point2d rotate(const Point2d & v, double angle);
};


#endif
