#include <ducklings_follower/Trajectory.h>
#include <ducklings_follower/Geometry.h>
#include <iostream>

Trajectory::Trajectory(){

}

Trajectory::Trajectory(int distance, int alpha): distance(distance), alpha(alpha)
{
   trajectoryType = 0; // linear
   duration = 5*distance;
   stop();
}

Trajectory::Trajectory(int distance, double r1, double r2, bool clockWise): distance(distance), r1(r1), r2(r2)
{
  trajectoryType = 1; // elipse
  dir = clockWise ? 1 : -1;
  duration = 180;
  stop();
}

Mat mappp(1000, 1000, CV_8UC3, Scalar(255,255,255));

bool Trajectory::updateState(double xx, double yy, double yaw)
{
  //cout << " current pos << " << xx << " " << yy << " " << endl;
  if(!active){
    active = true;

    startPosition = Point2d(xx,yy);
    startYaw = yaw;

    if(trajectoryType == 0){
      endPosition = startPosition + (distance * Point2d(cos(startYaw), sin(startYaw)));
      circle( mappp,Point2d(300,300)+(50*startPosition), 2, Scalar(100,100,100), 1, LINE_AA);
      circle( mappp,Point2d(300,300)+(50*endPosition), 2, Scalar(0,0,0), 1, LINE_AA);
    }else if(trajectoryType == 1){

      Point2d startOrientation = Geometry::normalized(Point2d(cos(startYaw), sin(startYaw)));
      Point2d perpendicular = Point2d(-1*startOrientation.y, startOrientation.x);

      arrowedLine(mappp, Point2d(300,300)+(50*startPosition), Point2d(300,300)+(50*(startPosition+startOrientation)), Scalar(0,0, 255), 1);
      arrowedLine(mappp, Point2d(300,300)+(50*startPosition), Point2d(300,300)+(50*(startPosition+perpendicular)), Scalar(255,0,0), 1);


      centerPosition = startPosition + dir*r1*perpendicular;


      arrowedLine(mappp, Point2d(300,300)+(50*centerPosition), Point2d(300,300)+(50*(centerPosition+startOrientation)), Scalar(0,0, 100), 1);
      arrowedLine(mappp, Point2d(300,300)+(50*centerPosition), Point2d(300,300)+(50*(centerPosition+perpendicular)), Scalar(100,0,0), 1);


      circle(mappp,Point2d(300,300)+(50*startPosition), 2, Scalar(255,0,0), 2, LINE_AA);
      circle(mappp,Point2d(300,300)+(50*centerPosition), 2, Scalar(0,0,0), 1, LINE_AA);

    }

  }

  currentPosition = Point2d(xx,yy);
  currentYaw = yaw;

  return  elapsedDuration > duration;
}

int iii = 0;
bool Trajectory::walk(geometry_msgs::Twist &twist)
{
  if(!active) return false;
  if(elapsedDuration > duration) return false;

  double pElapsedDuration = ((double)(elapsedDuration+1))/duration;

  switch (trajectoryType) {
    case 0:
    {
        Point2d nextPosition = startPosition + (pElapsedDuration * distance * Point2d(cos(startYaw), sin(startYaw)));
      //    cout << "next " << nextPosition << endl;
          circle( mappp,Point2d(300,300)+(50*nextPosition), 1, Scalar(255,0,0), 1, LINE_AA);

          Point2d delta = nextPosition - currentPosition;
          Point2d currentOrientation = Point2d(cos(currentYaw), sin(currentYaw));




          double angle = Geometry::angleBetween2(Geometry::normalized(currentOrientation),
                                                Geometry::normalized(delta));

          Point2d perpendicular = Geometry::normalized(Point2d(-1*currentOrientation.y, currentOrientation.x));

          arrowedLine(mappp, Point2d(300,300)+(50*currentPosition), Point2d(300,300)+(50*nextPosition), Scalar(0,255,0), 1);
          arrowedLine(mappp, Point2d(300,300)+(50*currentPosition), Point2d(300,300)+(50*(currentPosition+perpendicular)), Scalar(255,0,0), 1);



          twist.linear.x = Geometry::norm(delta);
          twist.angular.z = angle; //0.1f*( Geometry::angleBetween(delta, perpendicular) < (M_PI/2) ? angle : -1*angle);

          circle( mappp,Point2d(300,300)+(50*currentPosition), 1, Scalar(0,0,255), 1, LINE_AA);


        //std::cout << "> " << angle << " " << twist.angular.z << std::endl;


        //  std::cout << "delta " << Geometry::norm(delta) << ", yaw " << Geometry::angleBetween(currentOrientation, delta) << endl;
        //  std::cout << "start yaw " << startYaw << std::endl;
        //  std::cout << "elapsed distance " << Geometry::distance(startPosition, currentPosition) << std::endl;
        //  std::cout << "remaining distance " << Geometry::distance(currentPosition, endPosition) << std::endl;
        //  double total = (Geometry::distance(currentPosition, endPosition) + Geometry::distance(startPosition, currentPosition));
        //  std::cout << "total distance " << total << std::endl;

      elapsedDuration++;
      imshow("Trajector 6", mappp);
      waitKey(1);
      break;
    }
    case 1:
    {
      double nextAngle = (dir == 1 ? M_PI : 0) + dir*2*M_PI*pElapsedDuration;
      Point2d nextPosition = centerPosition + Geometry::rotate(Point2d(r1*cos(nextAngle),r2*sin(nextAngle)), -1*(2*M_PI -1*(startYaw + M_PI_2)));


      Point2d delta = nextPosition - currentPosition;
      Point2d currentOrientation = Point2d(cos(currentYaw), sin(currentYaw));

      //double angle = Geometry::angleBetween(Geometry::normalized(currentOrientation),
      //                                      Geometry::normalized(delta));


      Point2d perpendicular = Geometry::normalized(Point2d(-1*currentOrientation.y, currentOrientation.x));
      arrowedLine(mappp, Point2d(300,300)+(50*currentPosition), Point2d(300,300)+(50*nextPosition), Scalar(0,255,0), 1);
      arrowedLine(mappp, Point2d(300,300)+(50*currentPosition), Point2d(300,300)+(50*(currentPosition+perpendicular)), Scalar(255,0,0), 1);

      arrowedLine(mappp, Point2d(300,300), Point2d(300,300)+ (50*(Point2d( cos(startYaw), sin(startYaw) ) )), Scalar(255,0,0), 1);
      arrowedLine(mappp, Point2d(300,300), Point2d(300,300)+ (50*(
                          Geometry::rotate(Point2d( cos(startYaw), sin(startYaw) ), M_PI_2)
                                                                )), Scalar(0,0,255), 1);


      double angle = Geometry::angleBetween2(Geometry::normalized(currentOrientation),
                                             Geometry::normalized(delta));

      if(abs(angle) < M_PI_2){
        twist.linear.x = Geometry::norm(delta);
      }
      twist.angular.z = angle; //( Geometry::angleBetween(delta, perpendicular) < (M_PI/2) ? angle : -1*angle);


      circle( mappp,Point2d(300,300)+(50*currentPosition), 1, Scalar(0,0,255), 1, LINE_AA);


      circle( mappp,Point2d(300,300)+(50*nextPosition), 1, Scalar(255,0,0), 1, LINE_AA);

      elapsedDuration++;
      imshow("Trajector 6", mappp);
      waitKey(1);
      iii++;
      break;
    }
  }
  return true;
}

void Trajectory::stop()
{
  active = false;
}
