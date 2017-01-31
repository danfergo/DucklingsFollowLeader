#ifndef __AGENT_CPP__
#define __AGENT_CPP__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/common.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/impl/point_types.hpp>

#include <vector>

#include <ducklings_follower/Trajectory.h>

using namespace std;
using namespace pcl;
using namespace ros;
using namespace cv;


class Agent {
    PointCloud <pcl::PointXYZ> cloud;
    geometry_msgs::PoseWithCovarianceStamped odom;

    geometry_msgs::PoseWithCovarianceStamped odomBeforeWatch;
    int watchingState = 0;
    long lastWatchEnding = 0;


    vector<Trajectory> trajectories;
    int currentTrajectory = 0;

    bool existsPointCloud;
    bool existsOdom;
private:

    void buildViews(const PointCloud <pcl::PointXYZ> & cloud, Mat & frontalDepthView, Mat & topView, PointXYZ & delta);
    void detectTurtlebots(const Mat & topView, vector<Vec3f> & circles);
    bool walkToward(const vector<Vec3f> & circles,  PointXYZ & delta, geometry_msgs::Twist & twist);
    void showViews(const Mat & frontalDepthView, const Mat & topView, const vector<Vec3f> & circles ,  PointXYZ & delta);

public:
    Agent();
    void init();

    bool follow(geometry_msgs::Twist & twist);
    bool walkTrajectory(geometry_msgs::Twist & twist);
    bool watch(geometry_msgs::Twist & twist);

    void setDepthView(const sensor_msgs::PointCloud2 pointCloud2);
    void setOdometry(const geometry_msgs::PoseWithCovarianceStamped odom);
    void setTrajectories(vector<Trajectory> trajectories);
};


#endif
