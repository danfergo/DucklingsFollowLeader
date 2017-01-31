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
#include <sensor_msgs/PointCloud2.h>

#include <vector>

using namespace std;
using namespace cv;


ros::Publisher publisher;
Agent agent;


void onReceivePointCloud2(const sensor_msgs::PointCloud2 pointCloud2){
    agent.setDepthView(pointCloud2);
}

void onReceiveOdom(const geometry_msgs::PoseWithCovarianceStamped odometry){
    agent.setOdometry(odometry);
}

void walk(){
    geometry_msgs::Twist tw;
    if(!agent.watch(tw)){
      agent.walkTrajectory(tw);
    }
    publisher.publish(tw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Wake up.");

    ros::Subscriber sub = nh.subscribe("robot_pose_ekf/odom_combined", 1, &onReceiveOdom);
    ros::Subscriber sub2 = nh.subscribe("camera/depth/points", 1, &onReceivePointCloud2);

    publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10);
    vector<Trajectory> trajectories;

    XmlRpc::XmlRpcValue v;
    nh.param(ros::this_node::getName() + "/trajectories", v, v);
    for(int i =0; i < v.size(); i++)
    {
        if(v[i].getType()==XmlRpc::XmlRpcValue::TypeArray) {
            if(v[i].size()==1 &&
                v[i][0].getType()==XmlRpc::XmlRpcValue::TypeInt) { // linear - distance
                trajectories.push_back(Trajectory::createLineTrajectory(v[i][0]));
            }
            else if(v[i].size()==3 &&
                v[i][0].getType()==XmlRpc::XmlRpcValue::TypeDouble &&
                v[i][1].getType()==XmlRpc::XmlRpcValue::TypeDouble &&
                v[i][2].getType()==XmlRpc::XmlRpcValue::TypeBoolean) { // elipse - distance, r1, r2, clockwise
                trajectories.push_back(Trajectory::createEllipseTrajectory((v[i][0]), v[i][1], v[i][2]));
            }
            else {
             //   cout << "Invalid Trajectory!(size=" << v[i].size() << ";types:";
                for(unsigned int j=0; j < v[i].size(); j++) {
               //     cout << "[" << j << "]=" << v[i][j].getType();
                    if(j!=v[i].size()-1) cout << ",";
                }
             //   cout << ")" << endl;
            }
        }

        //cout << "params: " << v[i] << endl;
    }

    //cout << "trajectories.size(): " << trajectories.size() << endl;

    //trajectories.push_back(Trajectory::createLineTrajectory(4));
    //trajectories.push_back(Trajectory::createEllipseTrajectory(2, 2, false));
    //trajectories.push_back(Trajectory(2, 2, 3, true));


    agent.setTrajectories(trajectories);


    ros::Rate r(2);
    while (ros::ok()) {
        ros::spinOnce();
        walk();
        r.sleep();
    }
}
