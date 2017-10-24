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

// ******************** POSE FINDING ********************

bool BasePoseFinder::findJoints()
{
  KDL::JntArray jointArray(chain.getNrOfJoints());
  KDL::Frame frame;

  frame.p.data[0] = 0.257286;
  frame.p.data[1] = -0.391766;
  frame.p.data[2] = 0.380384;

  frame.M.data[0] = -0.605096;
  frame.M.data[3] = 0.795091;
  frame.M.data[6] = -0.0410874;
  frame.M.data[1] = 0.00882153;
  frame.M.data[4] = -0.0449086;
  frame.M.data[7] = -0.998952;
  frame.M.data[2] = -0.796103;
  frame.M.data[5] = -0.604825;
  frame.M.data[8] = 0.0201601;

  Int solution = ikSolverPos->CartToJnt(KDL::JntArray(chain.getNrOfJoints()), frame, jointArray);
  if (solution >= 0)
  {
    for (UInt i = 0; i < jointArray.data.size(); ++i)
      std::cout << "joint_" << i << ": " << jointArray.data[i] << ", ";
    std::cout << std::endl;

    return true;
  }

  std::cout << "No pose Found!!!" << std::endl;
  return false;
}

void BasePoseFinder::findPose(const std::vector<Real> joints)
{
  if (joints.size() != chain.getNrOfJoints())
    return;

  KDL::JntArray jointArray(chain.getNrOfJoints());
  KDL::Frame frame;

  for (UInt i = 0; i < joints.size(); ++i)
    jointArray.data[i] = joints[i];

  if (fkSolverPos->JntToCart(jointArray, frame) >= 0)
  {
//    Real rotX, rotY, rotZ;
//    Real roll, pitch, yaw;
//    frame.M.GetEulerZYX(rotZ, rotY, rotX);
//    frame.M.GetRPY(roll, pitch, yaw);
    std::cout << frame.p[0] << ", " << frame.p[1] << ", " << frame.p[2] << std::endl << std::endl;

    std::cout << frame.M.data[0] << ", " << frame.M.data[1] << ", " << frame.M.data[2] << std::endl;
    std::cout << frame.M.data[3] << ", " << frame.M.data[4] << ", " << frame.M.data[5] << std::endl;
    std::cout << frame.M.data[6] << ", " << frame.M.data[7] << ", " << frame.M.data[8] << std::endl;
  }
}

// ******************** INITIALIZATION ********************

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
  bool readFine = kdl_parser::treeFromParam("/squirrel_8dof_planning/robot_description", kdlTree);
  if (!readFine)
  {
    ROS_ERROR("Could not parse urdf.");
    return false;
  }

  if (!kdlTree.getChain("base_y_link", "hand_wrist_link", chain))
  {
    ROS_ERROR("Could not find chain between base_y_link and hand_wrist_link");
    return false;
  }

  jointArrayMin.resize(6);
  jointArrayMax.resize(6);

  jointArrayMin.data[0] = -M_PI;
  jointArrayMax.data[0] = M_PI;
  jointArrayMin.data[1] = -1.2;
  jointArrayMax.data[1] = 1.5;
  jointArrayMin.data[2] = -1.7;
  jointArrayMax.data[2] = 2.6;
  jointArrayMin.data[3] = -1.8;
  jointArrayMax.data[3] = 1.8;
  jointArrayMin.data[4] = -2.4;
  jointArrayMax.data[4] = 2.4;
  jointArrayMin.data[5] = -2.9;
  jointArrayMax.data[5] = 2.9;

  fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
  ikSolverVel = new KDL::ChainIkSolverVel_wdls(chain, 1e-5, 500);
  ikSolverPosJL = new KDL::ChainIkSolverPos_NR_JL(chain, jointArrayMin, jointArrayMax, *fkSolverPos, *ikSolverVel, 500, 1e-5);
  ikSolverPos = new KDL::ChainIkSolverPos_LMA(chain, 1e-5, 500, 1e-10);

  return true;
}

// ******************** POSE FINDING ********************

bool BasePoseFinder::setRotation(Tuple3D &axisX, Tuple3D &axisY, KDL::Frame &frame)
{
  if (axisX * axisY >= TINY_FLT)
    return false;

  axisX.normalize();
  axisY.normalize();

  const Tuple3D axisZ = axisX ^ axisY;

  frame.M.data[0] = axisX.x;
  frame.M.data[1] = axisY.x;
  frame.M.data[2] = axisZ.x;
  frame.M.data[3] = axisX.y;
  frame.M.data[4] = axisY.y;
  frame.M.data[5] = axisZ.y;
  frame.M.data[6] = axisX.z;
  frame.M.data[7] = axisY.z;
  frame.M.data[8] = axisZ.z;

  return true;
}

// ******************** INLINES ********************

inline bool BasePoseFinder::jointsInRange(const KDL::JntArray &joints)
{
  const UInt numJoints = joints.data.size();
  for(UInt i = 0; i < numJoints; ++i)
    if(joints.data[i] < jointArrayMin.data[i] || joints.data[i] > jointArrayMax.data[i])
      return false;

  return true;
}

} //namespace SquirrelMotionPlanner
