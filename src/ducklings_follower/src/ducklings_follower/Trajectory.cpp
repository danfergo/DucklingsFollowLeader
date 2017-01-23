#include <ducklings_follower/Trajectory.h>

#include <iostream>

Trajectory::Trajectory(){

}

Trajectory::Trajectory(int duration, int alpha): duration(duration), alpha(alpha)
{
   trajectoryType = 0; // linear
   restart();
}

Trajectory::Trajectory(int duration, int r1, int r2): duration(duration), r1(r1), r2(r2)
{
  trajectoryType = 1; // elipse
  restart();
}

bool Trajectory::walk(geometry_msgs::Twist &twist)
{
  switch (trajectoryType) {
  case 0:
    if(elapsedAlpha < alpha){
        twist.angular.z = 0.52359877559;
        elapsedAlpha += 30;
    } else {
        twist.linear.x = 0.5f;
        elapsedDuration++;
    }
    std::cout << (elapsedDuration >= duration);
    std::cout << " " << elapsedAlpha << " " << duration << std::endl;
    return elapsedDuration >= duration;
  }

  return true;
}

void Trajectory::restart()
{
 elapsedAlpha = 0;
 elapsedDuration = 0;
}
