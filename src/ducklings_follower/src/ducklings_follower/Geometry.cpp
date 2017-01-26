#include <ducklings_follower/Geometry.h>

#include <math.h>

double Geometry::angleBetween(const Point2d &v1, const Point2d &v2)
{
    double len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    double len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

    double dot = v1.x * v2.x + v1.y * v2.y;

    double a = dot / (len1 * len2);

    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return M_PI;
    else
      return acos(a); // 0..PI
}

double Geometry::angleBetween2(const Point2d &v1, const Point2d &v2)
{
  Point2d nv1 = normalized(v1);
  Point2d nv2 =  normalized(v2);
  double abs = angleBetween(nv1,nv2);
  Point2d perpendicular = Point2d(-1*nv1.y, nv1.x);
  return angleBetween(nv2, perpendicular) < (M_PI_2) ? abs : -1*abs;
}


double Geometry::norm(const Point2d &p){
    return sqrt(pow(p.x, 2) + pow(p.y, 2));
}

double Geometry::distance(const Point2d &v1, const Point2d &v2){
    return sqrt(pow(v2.x - v1.x, 2) + pow(v2.y - v1.y, 2));
}

Point2d Geometry::normalized(const Point2d &p){
    double nn = norm(p);
    return Point2d(p.x/nn, p.y/nn);
}

Point2d Geometry::rotate(const Point2d &v, double angle)
{
  double x = v.x;
  double y = v.y;
  double cosa = cos(angle);
  double sina = sin(angle);
  return Point2d((x*cosa)-(y*sina), (x*sina) + (y*cosa));
}
