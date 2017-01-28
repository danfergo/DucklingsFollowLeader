#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <ducklings_follower/Trajectory.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/Quaternion.h>
#include <ducklings_follower/Agent.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

using namespace std;
using namespace cv;


ros::Publisher publisher;
Agent agent;

vector<Trajectory> trajectories;
int currentTrajectory = 0;
/*
void onReceivePointCloud2(const geometry_msgs::PointCloud2 pointCloud2){

}*/

void onReceiveOdom(const geometry_msgs::PoseWithCovarianceStamped odometry){
    agent.setOdometry(odometry);
}

void walk(){
    geometry_msgs::Twist tw;
    agent.walk(tw);
    publisher.publish(tw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Wake up.");

    ros::Subscriber sub = nh.subscribe("/robot1/robot_pose_ekf/odom_combined", 1, &onReceiveOdom);
    //ros::Subscriber sub2 = nh.subscribe("/robot1/camera/depth/points", 1, &onReceivePointCloud2);

    publisher = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel_mux/input/teleop", 10);



    trajectories.push_back(Trajectory::createLineTrajectory(4));
    trajectories.push_back(Trajectory::createEllipseTrajectory(2, 2, false));
    //trajectories.push_back(Trajectory(2, 2, 3, true));


    agent.setTrajectories(trajectories);


    ros::Rate r(2);
    while (ros::ok()) {
        ros::spinOnce();
        walk();
        r.sleep();
    }
}
