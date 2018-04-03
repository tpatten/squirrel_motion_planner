#include <squirrel_8dof_visualizer/squirrel_8dof_visualizer.h>

namespace SquirrelMotionPlanning
{

Visualizer::Visualizer(const std::string &robotDescriptionTopic) :
    nhPrivate("~"), rate(30), robotMarkerPublisher("robot_trajectory_visualization"), jointStatesArm(new sensor_msgs::JointState), visibilityNormalized(true), visibilityRaw(
        false), visibilityGoal(true)
{
  subscriberTrajectoryNormalized = nh.subscribe("squirrel_8dof_planner_node/robot_trajectory_normalized", 1, &Visualizer::subscriberTrajectoryNormalizedHandler,
                                                this);
  subscriberTrajectoryRaw = nh.subscribe("squirrel_8dof_planner_node/robot_trajectory_raw", 1, &Visualizer::subscriberTrajectoryRawHandler, this);
  subscriberGoalPose = nh.subscribe("squirrel_8dof_planner_node/robot_goal_pose", 1, &Visualizer::subscriberGoalPoseHandler, this);
  subscriberRate = nhPrivate.subscribe("set_rate", 1, &Visualizer::subscriberRateHandler, this);

  serviceServerSetVisibilityNormalized = nh.advertiseService("set_visibility_normalized", &Visualizer::serviceCallbackSetVisibilityNormalized, this);
  serviceServerSetVisibilityRaw = nh.advertiseService("set_visibility_raw", &Visualizer::serviceCallbackSetVisibilityRaw, this);
  serviceServerSetVisibilityGoal = nh.advertiseService("set_visibility_goal", &Visualizer::serviceCallbackSetVisibilityGoal, this);

  robotIDGoal = robotMarkerPublisher.addRobotFromDescription(robotDescriptionTopic);
  robotIDPlanNormalized = robotMarkerPublisher.addRobotFromDescription(robotDescriptionTopic);
  robotIDPlanRaw = robotMarkerPublisher.addRobotFromDescription(robotDescriptionTopic);


  std_msgs::ColorRGBA color;
  color.a = 0.2;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  robotMarkerPublisher.setRobotColor(robotIDPlanNormalized, color, rviz_robot_marker::RvizRobotMarkerPublisher::FORCE_COLOR);

  color.r = 0.0;
  color.g = 0.8;
  color.b = 0.0;
  robotMarkerPublisher.setRobotColor(robotIDGoal, color, rviz_robot_marker::RvizRobotMarkerPublisher::FORCE_COLOR);

  color.r = 0.8;
  color.g = 0.1;
  color.b = 0.1;
  robotMarkerPublisher.setRobotColor(robotIDPlanRaw, color, rviz_robot_marker::RvizRobotMarkerPublisher::FORCE_COLOR);
  robotMarkerPublisher.hideRobot(robotIDPlanRaw);

  transformBase.frame_id_ = "map";
  transformBase.child_frame_id_ = "base_link";
  transformBase.setIdentity();

  transformBaseTranslation.setZero();
  transformBaseRotation.setIdentity();

  jointStatesArm->position.resize(5, 0.0);
  jointStatesArm->name.resize(5);
  jointStatesArm->name[0] = "arm_joint1";
  jointStatesArm->name[1] = "arm_joint2";
  jointStatesArm->name[2] = "arm_joint3";
  jointStatesArm->name[3] = "arm_joint4";
  jointStatesArm->name[4] = "arm_joint5";

  run();
}

void Visualizer::run()
{
  while (ros::ok())
  {
    if (posesNormalized.size() > 1)
    {
      UInt indexAdjusted = indexCurrentNormalized < posesNormalized.size() ? indexCurrentNormalized : posesNormalized.size() - 1;

      transformBaseTranslation[0] = posesNormalized[indexAdjusted][0];
      transformBaseTranslation[1] = posesNormalized[indexAdjusted][1];
      transformBaseRotation.setRPY(0.0, 0.0, posesNormalized[indexAdjusted][2]);

      transformBase.setOrigin(transformBaseTranslation);
      transformBase.setBasis(transformBaseRotation);

      jointStatesArm->position[0] = posesNormalized[indexAdjusted][3];
      jointStatesArm->position[1] = posesNormalized[indexAdjusted][4];
      jointStatesArm->position[2] = posesNormalized[indexAdjusted][5];
      jointStatesArm->position[3] = posesNormalized[indexAdjusted][6];
      jointStatesArm->position[4] = posesNormalized[indexAdjusted][7];

      robotMarkerPublisher.setRobotPose(robotIDPlanNormalized, transformBase, jointStatesArm);
    }

    if (posesRaw.size() > 1)
    {
      UInt indexAdjusted = indexCurrentRaw < posesRaw.size() ? indexCurrentRaw : posesRaw.size() - 1;

      transformBaseTranslation[0] = posesRaw[indexAdjusted][0];
      transformBaseTranslation[1] = posesRaw[indexAdjusted][1];
      transformBaseRotation.setRPY(0.0, 0.0, posesRaw[indexAdjusted][2]);

      transformBase.setOrigin(transformBaseTranslation);
      transformBase.setBasis(transformBaseRotation);

      jointStatesArm->position[0] = posesRaw[indexAdjusted][3];
      jointStatesArm->position[1] = posesRaw[indexAdjusted][4];
      jointStatesArm->position[2] = posesRaw[indexAdjusted][5];
      jointStatesArm->position[3] = posesRaw[indexAdjusted][6];
      jointStatesArm->position[4] = posesRaw[indexAdjusted][7];

      robotMarkerPublisher.setRobotPose(robotIDPlanRaw, transformBase, jointStatesArm);
    }

    if (poseGoal.size() == 8)
    {
      transformBaseTranslation[0] = poseGoal[0];
      transformBaseTranslation[1] = poseGoal[1];
      transformBaseRotation.setRPY(0.0, 0.0, poseGoal[2]);

      transformBase.setOrigin(transformBaseTranslation);
      transformBase.setBasis(transformBaseRotation);

      jointStatesArm->position[0] = poseGoal[3];
      jointStatesArm->position[1] = poseGoal[4];
      jointStatesArm->position[2] = poseGoal[5];
      jointStatesArm->position[3] = poseGoal[6];
      jointStatesArm->position[4] = poseGoal[7];

      robotMarkerPublisher.setRobotPose(robotIDGoal, transformBase, jointStatesArm);
    }

    ++indexCurrentNormalized;
    ++indexCurrentRaw;

    if (indexCurrentNormalized > indexMaxNormalized)
      indexCurrentNormalized = 0;

    if (indexCurrentRaw > indexMaxRaw)
      indexCurrentRaw = 0;

    ros::spinOnce();
    rate.sleep();
  }
}

void Visualizer::subscriberTrajectoryNormalizedHandler(const std_msgs::Float64MultiArray &msg)
{
  if (msg.layout.dim[1].size != 8)
  {
    ROS_WARN("Wrong robot trajectory dimension. Expected '8', but received '%d'.", msg.layout.dim[1].size);
    return;
  }
  else if (msg.layout.dim[0].size * msg.layout.dim[1].size != msg.data.size())
  {
    ROS_WARN("Specified layout does not match dimension of data.");
    return;
  }

  posesNormalized.resize(msg.layout.dim[0].size, std::vector<Real>(8));
  indexMaxNormalized = posesNormalized.size() + 40;

  UInt counter = 0;
  for (UInt i = 0; i < msg.layout.dim[0].size; ++i)
    for (UInt j = 0; j < 8; ++j)
    {
      posesNormalized[i][j] = msg.data[counter];
      ++counter;
    }

  indexCurrentNormalized = 0;
}

void Visualizer::subscriberTrajectoryRawHandler(const std_msgs::Float64MultiArray &msg)
{
  if (msg.layout.dim[1].size != 8)
  {
    ROS_WARN("Wrong robot trajectory dimension. Expected '8', but received '%d'.", msg.layout.dim[1].size);
    return;
  }
  else if (msg.layout.dim[0].size * msg.layout.dim[1].size != msg.data.size())
  {
    ROS_WARN("Specified layout does not match dimension of data.");
    return;
  }

  posesRaw.resize(msg.layout.dim[0].size, std::vector<Real>(8));
  indexMaxRaw = posesRaw.size() + 40;

  UInt counter = 0;
  for (UInt i = 0; i < msg.layout.dim[0].size; ++i)
    for (UInt j = 0; j < 8; ++j)
    {
      posesRaw[i][j] = msg.data[counter];
      ++counter;
    }

  indexCurrentRaw = 0;
}

void Visualizer::subscriberGoalPoseHandler(const std_msgs::Float64MultiArray &msg)
{
  if (msg.layout.dim[0].size != 8)
  {
    ROS_WARN("Wrong robot pose dimension. Expected '8', but received '%d'.", msg.layout.dim[1].size);
    return;
  }
  else if (msg.layout.dim[0].size != msg.data.size())
  {
    ROS_WARN("Specified layout does not match dimension of data.");
    return;
  }

  poseGoal.resize(8);
  for (UInt j = 0; j < 8; ++j)
    poseGoal[j] = msg.data[j];
}

void Visualizer::subscriberRateHandler(const std_msgs::Float64 &msg)
{
  rate = ros::Rate(msg.data);
}

bool Visualizer::serviceCallbackSetVisibilityNormalized(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
  visibilityNormalized = req.data;
  res.success = true;

  if (visibilityNormalized && posesNormalized.size() > 1)
    robotMarkerPublisher.showRobot(robotIDPlanNormalized);
  else
    robotMarkerPublisher.hideRobot(robotIDPlanNormalized);

  return true;
}

bool Visualizer::serviceCallbackSetVisibilityRaw(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
  visibilityRaw = req.data;
  res.success = true;

  if (visibilityRaw && posesRaw.size() > 1)
    robotMarkerPublisher.showRobot(robotIDPlanRaw);
  else
    robotMarkerPublisher.hideRobot(robotIDPlanRaw);

  return true;
}

bool Visualizer::serviceCallbackSetVisibilityGoal(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
  visibilityGoal = req.data;
  res.success = true;

  if (visibilityGoal && poseGoal.size() == 8)
    robotMarkerPublisher.showRobot(robotIDGoal);
  else
    robotMarkerPublisher.hideRobot(robotIDGoal);

  return true;
}

} //namespace SquirrelMotionPlanning
