#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <ducklings_follower/Trajectory.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace cv;


ros::Publisher publisher;


Trajectory trajectories [2];
int currentTrajectory = 0;


void walk(){
    geometry_msgs::Twist tw;

    if(trajectories[currentTrajectory].walk(tw)){
      trajectories[currentTrajectory].restart();
      currentTrajectory = (++currentTrajectory)%2;
    }

    publisher.publish(tw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    publisher = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel_mux/input/teleop", 10);

    trajectories[0] = Trajectory(5, 180);
    trajectories[1] = Trajectory(4, 180);

    ROS_INFO_STREAM("Walking.");


    ros::Rate r(1);
    while (ros::ok()) {
        ros::spinOnce();
        walk();
        r.sleep();
    }

}
