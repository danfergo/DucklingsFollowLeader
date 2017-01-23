#include <ducklings_follower/Agent.h>

#include <ducklings_follower/Geometry.h>


Agent::Agent(){

}

void Agent::init(){

}


int scale = 100.f;

bool Agent::body(geometry_msgs::Twist & twist){

  PointCloud <pcl::PointXYZ> cloud;
    if(whatPointCloud(cloud)){
        Mat frontalDepthView;
        Mat topView;
        PointXYZ delta;
        vector<Vec3f> circles;

        buildViews(cloud, frontalDepthView, topView, delta);
        detectTurtlebots(topView, circles);
        walk(circles, delta, twist);
        showViews(frontalDepthView, topView, circles, delta);
        return true;
    }


    return true;
};


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

          float depth = (cloud.at(j,i).z*50);
          float z = cloud.at(j,i).z;
          float x = cloud.at(j,i).x;
          float y = cloud.at(j,i).y;

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


bool Agent::whatPointCloud(PointCloud <pcl::PointXYZ> & cloud){

  sensor_msgs::PointCloud2 ptcl = tmpPointCloud2;

  PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(ptcl, pcl_pc);

  fromPCLPointCloud2(pcl_pc, cloud);

  return cloud.size() > 0;
}

void Agent::detectTurtlebots(const Mat & topView, vector<Vec3f> & circles){

  float radius = (scale/5.35);
  HoughCircles(topView, circles, CV_HOUGH_GRADIENT, 1, radius-5, 10, 10, radius-3, radius+3);


}

bool Agent::walk(const vector<Vec3f> & circles, PointXYZ & delta, geometry_msgs::Twist & twist){

  geometry_msgs::Twist tw;

  for(size_t i = 0; i < circles.size(); i++ ){
      Vec3f c = circles[i];

      Point2f C = Point2f(c[0], c[1]);
      Point2f O = scale*Point2f(delta.x, delta.z);
      Point2f F = scale*Point2f(delta.x, delta.z+1);

      Point2f nOF = Geometry::normalized(F - O);
      Point2f nOC = Geometry::normalized(C - O);
      float angle = Geometry::angleBetween(nOC, nOF);
      float dst = Geometry::norm(C - O)/scale;

       float stopLine = 1.30f;
       float margin = 0.05f;

       tw.linear.x =  dst > stopLine ? 1.5f*(dst - stopLine) : ( dst < (stopLine - margin) ? -2.f*(stopLine - dst) : 0.0f);
       tw.angular.z = 0.7*((nOC.x > 0.0f) ? -1.0f*angle : angle);


      twist = tw;
      return true;
  }

  return false;
};


void Agent::setDepthView(const sensor_msgs::PointCloud2 pointCloud2){
    tmpPointCloud2 = pointCloud2;
};

void Agent::showViews(const Mat & frontalDepthView, const Mat & topView, const vector<Vec3f> & circles,  PointXYZ & delta){
  if(topView.cols > 0 && topView.rows > 0
     && frontalDepthView.cols > 0 && frontalDepthView.rows > 0) {

    Mat topColorView;
    cvtColor(topView, topColorView, COLOR_GRAY2BGR);


    for(size_t i = 0; i < circles.size(); i++ ){
        Vec3f c = circles[i];

        circle( topColorView, Point(c[0], c[1]), c[2], Scalar(0,0,255), 1, LINE_AA);
        circle( topColorView, Point(c[0], c[1]), 2, Scalar(0,255,0), 1, LINE_AA);

        Point2f C = Point2f(c[0], c[1]);
        Point2f O = scale*Point2f(delta.x, delta.z);
        Point2f F = scale*Point2f(delta.x, delta.z+1);

        arrowedLine(topColorView, O, O + scale*Geometry::normalized(C - O), Scalar(0,255,0), 1);
        arrowedLine(topColorView, O, O + scale*Geometry::normalized(F - O), Scalar(255,0,0), 1);

    }

    imshow("Top view", topColorView);
    imshow("Depth view", frontalDepthView);
    waitKey(1);
  }
}
