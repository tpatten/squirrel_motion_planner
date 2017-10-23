#ifndef SQUIRREL_MOTION_PLANNING_BASE_POSE_FINDER_H_
#define SQUIRREL_MOTION_PLANNING_BASE_POSE_FINDER_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdint.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <squirrel_8dof_planner/squirrel_8dof_planner_structures.hpp>

namespace SquirrelMotionPlanner
{

class BasePoseFinder
{
  ros::NodeHandle nh;

  KDL::Tree kdlTree;
  KDL::Chain chain;
  KDL::JntArray jointArrayMax;
  KDL::JntArray jointArrayMin;
  KDL::ChainFkSolverPos_recursive* fkSolverPos;
  KDL::ChainIkSolverVel_wdls* ikSolverVel;
  KDL::ChainIkSolverPos_NR_JL* ikSolverPosJL;

public:
  /**
   * @brief Constructor. Initializes everything.
   */
  BasePoseFinder();

private:

  // ******************** INITIALIZATION ********************

  /**
   * @brief Initializes all solvers and the reachability map.
   * @return Returns false if the robot description cannot be read or the defined chains are not available.
   */
  bool initialize();

  /**
   * @brief Initializes all ROS subscribers and publishers.
   */
  void initializeMessageHandling();

  /**
   * @brief Initializes the chains that are used for the IK solvers.
   * @return Returns false if the defined chains are not available.
   */
  bool initializeKDL();
};

} //namespace SquirrelMotionPlanner

#endif //SQUIRREL_MOTION_PLANNING_BASE_POSE_FINDER_H_

