#ifndef SQUIRREL_8DOF_VISUALIZER_SQUIRREL_8DOF_VISUALIZER_H_
#define SQUIRREL_8DOF_VISUALIZER_SQUIRREL_8DOF_VISUALIZER_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <rviz_robot_marker/RvizRobotMarkerPublisher.h>

#include <vector>
#include <string>
#include <stdint.h>

namespace SquirrelMotionPlanning
{
typedef int32_t Int;     ///< 32-bit signed integer definition.
typedef uint32_t UInt;     ///< 32-bit unsigned integer definition.
typedef double Real;     ///< 64-bit floating point definition.

/**
 * @brief Visualizer class, used for visualizing 8dof trajectories of the squirrel robot.
 * The node subscribes to a multi array topic that contains the full robot trajectory and displays the full trajectory
 * as a ghost model ghost model that performs the trajectory perfectly.
 */
class Visualizer
{
public:
  /**
   * @brief Constructor. Initializes all topics, variables and launches the visualizer.
   */
  Visualizer(const std::string &robotDescriptionTopic);

private:
  ros::NodeHandle nh;     ///< ROS node handle with global namespace.
  ros::NodeHandle nhPrivate;     ///< ROS node handle with namespace relative to current node.
  ros::Subscriber subscriberTrajectoryNormalized;     ///< ROS subscriber. Subscribes to a Float64MultiArray that contains a vector of 8D poses.
  ros::Subscriber subscriberTrajectoryRaw;     ///< ROS subscriber. Subscribes to a Float64MultiArray that contains a vector of 8D poses.
  ros::Subscriber subscriberGoalPose;     ///< ROS subscriber. Subscribes to a Float64MultiArray that contains a 8D pose.
  ros::Subscriber subscriberRate;     ///< ROS subscriber. Subscribes to a Float64 and sets the publishing speed of the trajectory.
  ros::ServiceServer serviceServerSetVisibilityNormalized;     ///< ROS service server. Sets the visibility of the normlized trajectory.
  ros::ServiceServer serviceServerSetVisibilityRaw;     ///< ROS service server. Sets the visibility of the raw trajectory.
  ros::ServiceServer serviceServerSetVisibilityGoal;     ///< ROS service server. Sets the visibility of the goal pose.

  rviz_robot_marker::RvizRobotMarkerPublisher robotMarkerPublisher;     ///< Publisher class, takes care of direct visualization to Rviz.
  UInt robotIDPlanNormalized;     ///< Robot ID needed to communicate with robotMarkerPublisher.
  UInt robotIDPlanRaw;     ///< Robot ID needed to communicate with robotMarkerPublisher.
  UInt robotIDGoal;     ///< Robot ID needed to communicate with robotMarkerPublisher.

  std::vector<std::vector<Real> > posesNormalized;     ///< Vector to which all normalized 8D poses are saved, once a new trajectory is received.
  std::vector<std::vector<Real> > posesRaw;     ///< Vector to which all raw 8D poses are saved, once a new trajectory is received.
  std::vector<Real> poseGoal;     ///< Current 8D goal pose to which a trajectory is planned.
  UInt indexMaxNormalized;     ///< Maximum index for the playback of the normlalized trajectory, =posesNormalized.size() + 30.
  UInt indexMaxRaw;     ///< Maximum index for the playback of the raw trajectory, =posesRaw.size() + 30.
  UInt indexCurrentNormalized;     ///< Index of the current normalized pose to be visualized.
  UInt indexCurrentRaw;     ///< Index of the current raw pose to be visualized.
  bool visibilityNormalized;
  bool visibilityRaw;
  bool visibilityGoal;

  tf::StampedTransform transformBase;     ///< The full base transform of the robot, sent to robotMarkerPublisher.
  tf::Vector3 transformBaseTranslation;     ///< Base translation, used to set translation of transformBase.
  tf::Matrix3x3 transformBaseRotation;     ///< Base rotation, used to set rotation of transformBase.
  sensor_msgs::JointStatePtr jointStatesArm;     ///< The full set of arm joint angles, sent to robotMarkerPublisher.

  ros::Rate rate;     ///< Current publishing rate, set in subscriberRateHandler.

  /**
   * @brief Main loop running at rate. Sets current robot pose and publishes it using robotMarkerPublisher.
   */
  void run();

  /**
   * @brief Handler for the normalized robot trajectory. Checks the dimension of the multi array and saves it to poses.
   * @param msg Contains the full robot trajectory represented as vector of 8D poses.
   */
  void subscriberTrajectoryNormalizedHandler(const std_msgs::Float64MultiArray &msg);

  /**
   * @brief Handler for the raw robot trajectory. Checks the dimension of the multi array and saves it to poses.
   * @param msg Contains the full robot trajectory represented as vector of 8D poses.
   */
  void subscriberTrajectoryRawHandler(const std_msgs::Float64MultiArray &msg);

  /**
   * @brief Handler for the goal pose. Checks the dimension of the multi array and saves it to poseSingle.
   * @param msg Contains the a robot pose represented as an 8D pose.
   */
  void subscriberGoalPoseHandler(const std_msgs::Float64MultiArray &msg);

  /**
    * @brief Handler for the visualization rate. Sets rate and changes the sleep time between visualization of poses in the main loop.
    * @param msg Contains the new rate.
    */
  void subscriberRateHandler(const std_msgs::Float64 &msg);

  /**
   * @brief Callback for setting the visibility of the normalized trajectory.
   * @param req The service request that contains visibility setting.
   * @param res The service response.
   */
  bool serviceCallbackSetVisibilityNormalized(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

  /**
   * @brief Callback for setting the visibility of the raw trajectory.
   * @param req The service request that contains visibility setting.
   * @param res The service response.
   */
  bool serviceCallbackSetVisibilityRaw(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

  /**
   * @brief Callback for setting the visibility of the goal pose.
   * @param req The service request that contains visibility setting.
   * @param res The service response.
   */
  bool serviceCallbackSetVisibilityGoal(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
}; //Visualizer


} //namespace SquirrelMotionPlanning

#endif // SQUIRREL_8DOF_VISUALIZER_SQUIRREL_8DOF_VISUALIZER_H_

