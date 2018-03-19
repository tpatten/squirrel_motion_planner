#include <ros/ros.h>
#include <ros/package.h>

#include <squirrel_8dof_visualizer/squirrel_8dof_visualizer.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "squirrel_8dof_visualizer_node");
  ros::NodeHandle nh("~");

  ros::Time startingTime = ros::Time::now();
  ros::Rate rate(1.0);
  while(!nh.hasParam("robot_description") && (ros::Time::now() - startingTime).toSec() < 20.0)
  {
    ros::spinOnce();
    rate.sleep();
  }

  SquirrelMotionPlanning::Visualizer* visualizer = new SquirrelMotionPlanning::Visualizer("robot_description");

  delete visualizer;

  return 0;
}
