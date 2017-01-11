#include <iostream>
#include <string>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace std;
using namespace cv;

int i = 0;
void onReceiveData(const sensor_msgs::PointCloud2 pointCloud2) {
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(pointCloud2, pcl_pc);

    pcl::PointCloud <pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
   // cout << ++i << endl;
/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_ptrCloud(&cloud);
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (m_ptrCloud);
    while (!viewer.wasStopped ())
    {
    }*/

/*
    Mat mat = Mat(cloud.height, cloud.width, CV_8UC3, Scalar(255,0,0));

    for (int i = 0; i < cloud.height; i++) {
        for (int j = 0; j < cloud.width; j++) {
           // cout << cloud.at(j,300).x << " ";
            float x = (cloud.at(j,i).x*256);

            if(x > 255){
                mat.at<Vec3b>(Point(j,i)) = Vec3b(0,0,255);
            }else{
                mat.at<Vec3b>(Point(j,i)) = Vec3b(x,0,0);
            }

        }
    }

    imshow("Depth view", mat);
    waitKey(1);
*/
    //  cout << "fields size: " << pointCloud2.fields

    //ROS_INFO_STREAM(out);


    // for (int i = 0; i < imageRaw.height; i++) {
    // for (int j = 0; j < imageRaw.width; j++) {
    //  image.at<int>(i,j) = (int)imageRaw.data[i*imageRaw.width + j];
    // ROS_INFO_STREAM(i << " " << j << " " << (int)imageRaw.data[i*imageRaw.width + j]);

    // std::cout << (int)imageRaw.data[i*imageRaw.width + j] << std::endl;
    //   }
    // }
    //namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
    // imshow("Display Image", image);
    // waitKey(0);



    // ROS_INFO_STREAM(pointCloud);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Listening");

    ros::Subscriber sub0 = nh.subscribe("/robot1/camera/depth/points", 1000, &onReceiveData);

    ros::Rate r(1);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

}