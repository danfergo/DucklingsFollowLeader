#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>

#include <boost/thread/thread.hpp>
#include <ducklings_follower/Agent.h>



Agent agent;


void onReceivePointCloud2(const sensor_msgs::PointCloud2 pointCloud2){
    agent.setDepthView(pointCloud2);
}
/*
void internalBehaviourThread(){
    NodeHandle nh;

    ros::Rate r(30);
    while (ros::ok()) {

        r.sleep();
    }
}

void subscriptionsThread(){


}*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    agent.init();

    ROS_INFO_STREAM("Wake up.");


    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/robot2/camera/depth/points", 1, &onReceivePointCloud2);
    ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("/robot2/cmd_vel_mux/input/teleop", 10);

    ros::Rate r(30);
    while (ros::ok()) {
        ros::spinOnce();

        geometry_msgs::Twist tw;
        if(agent.follow(tw)){
          publisher.publish(tw);
        }

        r.sleep();
    }



   // boost::thread thread_s(subscriptionsThread);

    //ros::spinOnce();


  //  boost::thread thread_b(internalBehaviourThread);


   // thread_b.join();
   // thread_s.join();

}
