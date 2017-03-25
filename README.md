# Ducklings follow leader

A ROS where a leader robot walks in a predefined trajectory and the remaining Turtlebot2's follow the leader. The work as developed and tested using Gazebo simulator.

[![Ducklings Follow Leader Demo](https://img.youtube.com/vi/MRKCEkJYjbg/0.jpg)](https://www.youtube.com/watch?v=MRKCEkJYjbg)

For more information read the attached article, [here](https://github.com/danfergo/DucklingsFollowLeader/blob/master/Article%20-%20Ducklings%20Follow%20Leader.pdf)

The source code consists of 4 packages:

1. **ducklings_follower**, automatic following controllers
2. **ducklings_gazebo**, a set of launch files to launch gazebo with world map and turtlebots
3. **ducklings_rviz**, for debug purposes, integration with RVIZ
4. **ducklings_teleop**, for debug purposes, allows you to drive turtlebots
