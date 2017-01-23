#ifndef __GEOMETRY_CPP__
#define __GEOMETRY_CPP__

#include <opencv2/core/core.hpp>

using namespace cv;

class Geometry{

public:
    static float angleBetween(const Point2f &v1, const Point2f &v2);
    static float norm(const Point2f &p);
    static float distance(const Point2f &v1, const Point2f &v2);
    static Point2f normalized(const Point2f &p);


};


#endif