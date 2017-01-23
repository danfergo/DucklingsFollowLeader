#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace cv;


class Trajectory {
public:
    static int const LINE = 0;
    static int const ELIPSE = 1;
private:
    int trajectoryType;
    int duration;
    int alpha = 0;
    int r1, r2;

    int elapsedAlpha;
    int elapsedDuration;
public:
    Trajectory();
    Trajectory(int duration, int alpha);
    Trajectory(int duration, int r1, int r2);
    bool walk(geometry_msgs::Twist & twist);
    void restart();
};







#endif // TRAJECTORY_H
