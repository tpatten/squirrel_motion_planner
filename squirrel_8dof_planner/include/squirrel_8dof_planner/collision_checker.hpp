#ifndef SQUIRREL_MOTION_PLANNING_COLLISION_CHECKER_HPP_
#define SQUIRREL_MOTION_PLANNING_COLLISION_CHECKER_HPP_

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
#include <kdl/chainiksolverpos_lma.hpp>

#include <squirrel_8dof_planner/squirrel_8dof_planner_structures.hpp>

namespace SquirrelMotionPlanner
{

class CollisionChecker
{
  struct TransformLink
  {
    KDL::Frame transform;
    KDL::Frame *transformParent;
    std::string name;

    KDL::Frame transformToParent;
    const KDL::Joint* joint;
    Real* jointValue;
  };

  ros::NodeHandle nh;

  KDL::Tree kdlTree;
  std::vector<Real> jointAngles;
  std::vector<TransformLink> transformLinks;

public:
  /**
   * @brief Constructor. Initializes everything.
   */
  CollisionChecker()
  {
    if (!initialize())
    {
      ros::shutdown();
      return;
    }
  }

  void updateTransforms(const std::vector<Real> &angles)
  {
    for (UInt i = 0; i < jointAngles.size(); ++i)
      jointAngles[i] = angles[i];
    updateTransforms();
  }

private:

  // ******************** INITIALIZATION ********************

  /**
   * @brief Initializes all solvers and the reachability map.
   * @return Returns false if the robot description cannot be read or the defined chains are not available.
   */
  bool initialize()
  {
    if (!initializeKDL())
      return false;

    return true;
  }

  /**
   * @brief Initializes the chains that are used for the IK solvers.
   * @return Returns false if the defined chains are not available.
   */
  bool initializeKDL()
  {
    bool readFine = kdl_parser::treeFromParam("/squirrel_8dof_planning/robot_description", kdlTree);
    if (!readFine)
    {
      ROS_ERROR("Could not parse urdf.");
      return false;
    }

    jointAngles.resize(8, 0.0);
    buildTree();

    return true;
  }

  void buildTree()
  {
    transformLinks.reserve(100);
    transformLinks.push_back(TransformLink());
    transformLinks.back().name = kdlTree.getRootSegment()->second.segment.getName();
    transformLinks.back().transform = KDL::Frame::Identity();
    transformLinks.back().transformParent = NULL;
    transformLinks.back().joint = NULL;
    transformLinks.back().transformToParent = KDL::Frame::Identity();

    expandTree(kdlTree.getRootSegment(), 0);
  }

  void expandTree(const KDL::SegmentMap::const_iterator& segment, UInt indexParent)
  {
    for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator child = segment->second.children.begin(); child != segment->second.children.end();
        ++child)
    {
      transformLinks.push_back(TransformLink());
      transformLinks.back().name = (*child)->second.segment.getName();
      transformLinks.back().transformParent = &transformLinks[indexParent].transform;

      if (transformLinks.back().name == "base_x_link")
      {
        transformLinks.back().joint = &((*child)->second.segment.getJoint());
        transformLinks.back().jointValue = &jointAngles[0];
      }
      else if (transformLinks.back().name == "base_y_link")
      {
        transformLinks.back().joint = &((*child)->second.segment.getJoint());
        transformLinks.back().jointValue = &jointAngles[1];
      }
      else if (transformLinks.back().name == "base_theta_link")
      {
        transformLinks.back().joint = &((*child)->second.segment.getJoint());
        transformLinks.back().jointValue = &jointAngles[2];
      }
      else if (transformLinks.back().name == "arm_link1")
      {
        transformLinks.back().joint = &((*child)->second.segment.getJoint());
        transformLinks.back().jointValue = &jointAngles[3];
      }
      else if (transformLinks.back().name == "arm_motor2")
      {
        transformLinks.back().joint = &((*child)->second.segment.getJoint());
        transformLinks.back().jointValue = &jointAngles[4];
      }
      else if (transformLinks.back().name == "arm_link3")
      {
        transformLinks.back().joint = &((*child)->second.segment.getJoint());
        transformLinks.back().jointValue = &jointAngles[5];
      }
      else if (transformLinks.back().name == "arm_link4")
      {
        transformLinks.back().joint = &((*child)->second.segment.getJoint());
        transformLinks.back().jointValue = &jointAngles[6];
      }
      else if (transformLinks.back().name == "arm_link5")
      {
        transformLinks.back().joint = &((*child)->second.segment.getJoint());
        transformLinks.back().jointValue = &jointAngles[7];
      }
      else
      {
        transformLinks.back().transformToParent = (*child)->second.segment.getFrameToTip();
        transformLinks.back().joint = NULL;
      }

      expandTree(*child, transformLinks.size() - 1);
    }
  }

  void updateTransforms()
  {
    for (std::vector<TransformLink>::iterator transformLink = transformLinks.begin() + 1; transformLink != transformLinks.end(); ++transformLink)
    {
      if (transformLink->joint != NULL)
        transformLink->transform = (*(transformLink->transformParent)) * transformLink->joint->pose(*(transformLink->jointValue));
      else
        transformLink->transform = (*(transformLink->transformParent)) * transformLink->transformToParent;
    }
  }
};

} //namespace SquirrelMotionPlanner

#endif //SQUIRREL_MOTION_PLANNING_COLLISION_CHECKER_HPP_

