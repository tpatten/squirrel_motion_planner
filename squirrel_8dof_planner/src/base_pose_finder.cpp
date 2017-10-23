#include <squirrel_8dof_planner/base_pose_finder.h>

namespace SquirrelMotionPlanner
{

BasePoseFinder::BasePoseFinder()
{
  if (!initialize())
  {
    ros::shutdown();
    return;
  }
}

bool BasePoseFinder::initialize()
{
  initializeMessageHandling();

  if (!initializeKDL())
    return false;

  return true;
}

void BasePoseFinder::initializeMessageHandling()
{

}

bool BasePoseFinder::initializeKDL()
{
  bool readFine = kdl_parser::treeFromParam("/robot_description", kdlTree);
  if (!readFine)
  {
    ROS_ERROR("Could not parse urdf.");
    return false;
  }

  if (!kdlTree.getChain("hand_wrist_link", "base_link", chain))
  {
    ROS_ERROR("Could not find chain between base_link and hand_wrist_link");
    return false;
  }

  jointArrayMin.resize(8);
  jointArrayMax.resize(8);

  jointArrayMin.data[0] = -0.523598775598;
  jointArrayMax.data[0] = 0.261799387799;
  jointArrayMin.data[1] = -1.308996939;
  jointArrayMax.data[1] = 0.785398163397;
  jointArrayMin.data[2] = 0.0;
  jointArrayMax.data[2] = 2.61799387799;
  jointArrayMin.data[3] = -1.74532925199;
  jointArrayMax.data[3] = 0.785398163397;
  jointArrayMin.data[4] = -0.261799387799;
  jointArrayMax.data[4] = 0.523598775598;
  jointArrayMin.data[5] = -0.785398163397;
  jointArrayMax.data[5] = 0.523598775598;
  jointArrayMin.data[6] = -0.785398163397;
  jointArrayMax.data[6] = 0.523598775598;
  jointArrayMin.data[7] = -0.785398163397;
  jointArrayMax.data[7] = 0.523598775598;


  fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
  ikSolverVel = new KDL::ChainIkSolverVel_wdls(chain, 1e-5, 500);
  ikSolverPosJL = new KDL::ChainIkSolverPos_NR_JL(chain, jointArrayMin, jointArrayMax, *fkSolverPos, *ikSolverVel, 500, 1e-5);

  return true;
}

} //namespace SquirrelMotionPlanner
