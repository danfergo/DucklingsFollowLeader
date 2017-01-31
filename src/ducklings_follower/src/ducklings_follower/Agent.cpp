#include <ducklings_follower/Agent.h>

#include <ducklings_follower/Geometry.h>
#include <tf/transform_datatypes.h>
#include <ctime>

#define TIME_BETWEEN_WATCH  30

std::time_t INIT_TIME = std::time(nullptr);


Agent::Agent(){
  lastWatchEnding = INIT_TIME;
}

void Agent::init(){

}

int scale = 100.f;

bool Agent::follow(geometry_msgs::Twist & twist)
{
  if(!existsPointCloud) return false;

  Mat frontalDepthView;
  Mat topView;
  PointXYZ delta;
  vector<Vec3f> circles;

  buildViews(cloud, frontalDepthView, topView, delta);
  detectTurtlebots(topView, circles);  
  walkToward(circles, delta, twist);
  showViews(frontalDepthView, topView, circles, delta);
  return true;
}

bool Agent::walkTrajectory(geometry_msgs::Twist &twist)
{
  trajectories[currentTrajectory].walk(twist);
}


void Agent::buildViews(const PointCloud <pcl::PointXYZ> & cloud, Mat & frontalDepthView, Mat & topView, PointXYZ & delta){

  PointXYZ  min, max;
  PointXYZ O = PointXYZ(0,0,0);

  getMinMax3D(cloud, min, max);

  delta = PointXYZ(O.x - min.x , O.y - min.y , O.z - min.z);
  delta.x = delta.x < 0.0f ? 0.0f : delta.x;
  delta.z = delta.z < 0.0f ? 0.0f : delta.z;


  int w = (max.x + delta.x)*scale;
  int h = (max.z + delta.z)*scale;

  Mat mat = Mat(cloud.height, cloud.width, CV_8UC3, Scalar(255,0,0));
  Mat tView = Mat(h, w, CV_8UC1, Scalar(255));

  for (int i = 0; i < cloud.height; i++) {
      for (int j = 0; j < cloud.width; j++) {

          double depth = (cloud.at(j,i).z*50);
          double z = cloud.at(j,i).z;
          double x = cloud.at(j,i).x;
          double y = cloud.at(j,i).y;

          if(y < 0.27 && y > 0.24){
              mat.at<Vec3b>(Point(j,i)) = Vec3b(0,0,255);

              int xx = (delta.x + x)*scale;
              int yy = (delta.z + z)*scale;

              if(xx >= 0 && xx < w && yy >= 0 && yy < h){
                  tView.at<uchar>(Point(xx,yy))  = 0;
              }

          }else{
              mat.at<Vec3b>(Point(j,i)) = Vec3b(depth,depth,depth);
          }
      }
  }

  frontalDepthView = mat;
  topView = tView;
}

void Agent::detectTurtlebots(const Mat & topView, vector<Vec3f> & circles)
{
  double radius = (scale/5.35);
  HoughCircles(topView, circles, CV_HOUGH_GRADIENT, 1, radius-5, 10, 10, radius-3, radius+3);
}

bool Agent::walkToward(const vector<Vec3f> & circles, PointXYZ & delta, geometry_msgs::Twist & twist){
  if(!existsPointCloud) return false;

  geometry_msgs::Twist tw;

  for(size_t i = 0; i < circles.size(); i++ ){
      Vec3f c = circles[i];

      Point2d C = Point2d(c[0], c[1]);
      Point2d O = scale*Point2d(delta.x, delta.z);
      Point2d F = scale*Point2d(delta.x, delta.z+1);

      Point2d nOF = Geometry::normalized(F - O);
      Point2d nOC = Geometry::normalized(C - O);
      double angle = -1*Geometry::angleBetween2(nOC, nOF);
      double dst = Geometry::norm(C - O)/scale;

      double stopLine = 1.30f;
      double margin = 0.05f;

       cout << dst << endl;


      tw.linear.x =  dst > stopLine ? (dst - stopLine) : ( dst < (stopLine - margin) ? (dst - stopLine) : 0.0f);
      tw.angular.z = angle;


      twist = tw;
      return true;
  }

  return false;
}


void Agent::setDepthView(const sensor_msgs::PointCloud2 pointCloud2){ 
  PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(pointCloud2, pcl_pc);
  fromPCLPointCloud2(pcl_pc, cloud);
  existsPointCloud = true;
}

void Agent::setOdometry(const geometry_msgs::PoseWithCovarianceStamped odometry)
{
  double xx = odometry.pose.pose.position.x;
  double yy = odometry.pose.pose.position.y;
  double yaw = tf::getYaw(odometry.pose.pose.orientation);

  if(trajectories[currentTrajectory].updateState(xx, yy, yaw)){
      trajectories[currentTrajectory].stop();
      currentTrajectory = (++currentTrajectory)%(trajectories.size());
  }
  odom = odometry;
  existsOdom = true;
}

void Agent::setTrajectories(vector<Trajectory> trajectories)
{
  this->trajectories = trajectories;
}

void Agent::showViews(const Mat & frontalDepthView, const Mat & topView, const vector<Vec3f> & circles,  PointXYZ & delta){
 if(topView.cols > 0 && topView.rows > 0
     && frontalDepthView.cols > 0 && frontalDepthView.rows > 0) {

    Mat topColorView;
    cvtColor(topView, topColorView, COLOR_GRAY2BGR);


    for(size_t i = 0; i < circles.size(); i++ ){
        Vec3f c = circles[i];

        circle( topColorView, Point(c[0], c[1]), c[2], Scalar(0,0,255), 1, LINE_AA);
        circle( topColorView, Point(c[0], c[1]), 2, Scalar(0,255,0), 1, LINE_AA);

        Point2d C = Point2d(c[0], c[1]);
        Point2d O = scale*Point2d(delta.x, delta.z);
        Point2d F = scale*Point2d(delta.x, delta.z+1);

        arrowedLine(topColorView, O, O + scale*Geometry::normalized(C - O), Scalar(0,255,0), 1);
        arrowedLine(topColorView, O, O + scale*Geometry::normalized(F - O), Scalar(255,0,0), 1);

    }

    imshow("Top view", topColorView);
    imshow("Depth view", frontalDepthView);
    waitKey(1);
  }
}

bool Agent::watch(geometry_msgs::Twist & twist){
    if(!existsPointCloud || !existsOdom) return false;

    std::time_t now = std::time(nullptr);
    if(watchingState == 0 && (now - lastWatchEnding < TIME_BETWEEN_WATCH)) return false;


    vector<Vec3f> circles;
    double yaw = tf::getYaw(odom.pose.pose.orientation);
    double yawBW = tf::getYaw(odomBeforeWatch.pose.pose.orientation);

    if(watchingState != 2){
        Mat topView;
        PointXYZ delta;
        Mat frontalDepthView;

        buildViews(cloud, frontalDepthView, topView, delta);
        detectTurtlebots(topView, circles);
    }



    switch(watchingState){
      case 0:
        if(now - lastWatchEnding >= TIME_BETWEEN_WATCH){
          odomBeforeWatch = odom;
          watchingState = 1; // turn on watch
        }
        break;
      case 1: // watching
        if(circles.size() > 0){
          watchingState = 2; // go back
        }
        break;
      case 2:
        if(abs(yawBW - yaw) < (0.001f*M_PI)){
          watchingState = 0; // stop watch
          lastWatchEnding = now;
        }
    }


    double r = M_PI/(FRAME_RATE*2);

    switch(watchingState){
      case 1:
        twist.angular.z = r;
        break;
      case 2:
        double a = yawBW - yaw;
        twist.angular.z = a > r ? r : (a < -r ? -r : a);
        break;
    }


   // cout << (yawBW - yaw) << " " << circles.size() << endl;

    return watchingState != 0;
}

