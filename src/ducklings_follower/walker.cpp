#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <ducklings_follower/Trajectory.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/Quaternion.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

using namespace std;
using namespace cv;


ros::Publisher publisher;


vector<Trajectory> trajectories;
int currentTrajectory = 0;


void onReceiveOdom(const geometry_msgs::PoseWithCovarianceStamped odometry){
  double xx = odometry.pose.pose.position.x;
  double yy = odometry.pose.pose.position.y;

  double yaw = tf::getYaw(odometry.pose.pose.orientation);

  if(trajectories[currentTrajectory].updateState(xx, yy, yaw)){
      currentTrajectory = (++currentTrajectory)%(trajectories.size());
  }
}

void walk(){
    geometry_msgs::Twist tw;

    trajectories[currentTrajectory].walk(tw);
    publisher.publish(tw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/robot1/robot_pose_ekf/odom_combined", 1, &onReceiveOdom);
    publisher = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel_mux/input/teleop", 10);

    trajectories.push_back(Trajectory(4, 0));
    trajectories.push_back(Trajectory(2, 2, 2, false));
    trajectories.push_back(Trajectory(2, 2, 3, true));


    ROS_INFO_STREAM("Walking.");


    ros::Rate r(1);
    while (ros::ok()) {
        ros::spinOnce();
        walk();
        r.sleep();
    }
}
