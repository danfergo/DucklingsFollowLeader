#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Twist.h>

#define LINEAR_TRAJECTORY  0
#define ELLIPSE_TRAJECTORY 1

using namespace std;
using namespace cv;


class Trajectory {
public:
    static int const LINE = 0;
    static int const ELIPSE = 1;
private:
    int trajectoryType = -1;
    bool active = false;

    double r1, r2;
    int elapsedDuration;
    double currentAngle;

    Point2d startPosition, centerPosition, currentPosition, endPosition;
    double startYaw, currentYaw;
    int dir;

    int distance;
    double duration;

public:
    Trajectory();
    bool updateState(double xx, double yy, double yaw);
    void walk(geometry_msgs::Twist & twist);
    void stop();

    static Trajectory createLineTrajectory(int distance);
    static Trajectory createEllipseTrajectory(double r1, double r2, bool clockWise);
};







#endif // TRAJECTORY_H
