#ifndef SQUIRREL_8DOF_VISUALIZER_H_
#define SQUIRREL_8DOF_VISUALIZER_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <rviz_robot_marker/RvizRobotMarkerPublisher.h>
#include <vector>
#include <stdint.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>


namespace SquirrelMotionPlanning
{

typedef int32_t Int;
typedef uint32_t UInt;
typedef double Real;

class Visualizer
{
public:
  Visualizer();

  void run();

private:
  ros::NodeHandle nh, nhPrivate;
  ros::Subscriber subscriberTrajectory, subscriberRate, subscriberVisibility;
  rviz_robot_marker::RvizRobotMarkerPublisher robotMarkerPublisher;
  UInt robotID;

  tf::StampedTransform transformBase;
  tf::Vector3 transformBaseTranslation;
  tf::Matrix3x3 transformBaseRotation;
  sensor_msgs::JointStatePtr jointStatesArm;
  std::vector<std::vector<Real> > poses;
  bool visible;
  UInt poseCurrent;
  bool finalPoseState;
  ros::Rate rate;
  ros::WallTime finalPoseTime;

  void subscriberTrajectoryHandler(const std_msgs::Float64MultiArray &msg);

  void subscriberRateHandler(const std_msgs::Float64 &msg);

  void subscriberVisibilityHandler(const std_msgs::Bool &msg);

}; //Visualizer


} //namespace SquirrelMotionPlanning

#endif //SQUIRREL_8DOF_VISUALIZER_H_

