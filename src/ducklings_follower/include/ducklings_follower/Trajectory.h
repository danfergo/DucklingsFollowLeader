#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Twist.h>

using namespace std;
using namespace cv;


class Trajectory {
public:
    static int const LINE = 0;
    static int const ELIPSE = 1;
private:
    int trajectoryType = -1;
    int alpha = 0;
    double r1, r2;

    int elapsedDuration;
    double currentAngle;

    bool active = false;
    Point2d startPosition, centerPosition, currentPosition, endPosition;
    double startYaw, currentYaw;
    int dir;

    int distance;
    double duration;

public:
    Trajectory();
    Trajectory(int distance, int alpha);
    Trajectory(int distance, double r1, double r2, bool clockWise);
    bool updateState(double xx, double yy, double yaw);
    bool walk(geometry_msgs::Twist & twist);
    void stop();
};







#endif // TRAJECTORY_H
