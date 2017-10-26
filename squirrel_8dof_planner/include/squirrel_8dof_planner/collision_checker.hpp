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
  struct TransformTree
  {
    std::vector<TransformTree> children;
    TransformTree* parent;
    KDL::Frame transform;
    std::string name;
  };

  ros::NodeHandle nh;

  KDL::Tree kdlTree;
  TransformTree transformTree;
  std::vector<Real> jointAngles;

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


    buildTree();

    jointAngles.resize(8, 0.0);
    long tb = ros::Time::now().toNSec();
    updateTransforms(transformTree);
    std::cout << "time: " << (ros::Time::now().toNSec() - tb) / 1000000.0 << "ms." << std::endl;

    return true;
  }

  void buildTree()
  {
    transformTree.name = kdlTree.getRootSegment()->second.segment.getName();
    transformTree.transform = KDL::Frame::Identity();
    expandTree(kdlTree.getRootSegment(), transformTree);
  }

  void expandTree(const KDL::SegmentMap::const_iterator& segment, TransformTree &tree)
  {
    TransformTree treeChild;
    for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator child = segment->second.children.begin(); child != segment->second.children.end();
        ++child)
    {
      treeChild.name = (*child)->second.segment.getName();
      treeChild.parent = &tree;
      treeChild.transform = (*child)->second.segment.getFrameToTip() * tree.transform;
      tree.children.push_back(treeChild);

      expandTree(*child, tree.children.back());
    }
  }

  void updateTransforms(TransformTree &tree)
  {
    for(std::vector<TransformTree>::iterator child = tree.children.begin(); child != tree.children.end(); ++child)
    {
      if((*child).name == "base_x_link")
        (*child).transform = kdlTree.getSegment("base_x_link")->second.segment.getJoint().pose(jointAngles[0]) * tree.transform;
      else if((*child).name == "base_y_link")
        (*child).transform = kdlTree.getSegment("base_y_link")->second.segment.getJoint().pose(jointAngles[1]) * tree.transform;
      else if((*child).name == "base_theta_link")
        (*child).transform = kdlTree.getSegment("base_theta_link")->second.segment.getJoint().pose(jointAngles[2]) * tree.transform;
      else if((*child).name == "arm_link1")
        (*child).transform = kdlTree.getSegment("arm_link1")->second.segment.getJoint().pose(jointAngles[3]) * tree.transform;
      else if((*child).name == "arm_motor2")
        (*child).transform = kdlTree.getSegment("arm_motor2")->second.segment.getJoint().pose(jointAngles[4]) * tree.transform;
      else if((*child).name == "arm_link3")
        (*child).transform = kdlTree.getSegment("arm_link3")->second.segment.getJoint().pose(jointAngles[5]) * tree.transform;
      else if((*child).name == "arm_link4")
        (*child).transform = kdlTree.getSegment("arm_link4")->second.segment.getJoint().pose(jointAngles[6]) * tree.transform;
      else if((*child).name == "arm_link5")
        (*child).transform = kdlTree.getSegment("arm_link5")->second.segment.getJoint().pose(jointAngles[7]) * tree.transform;
      else
        (*child).transform = kdlTree.getSegment((*child).name)->second.segment.getFrameToTip() * tree.transform;

      updateTransforms(*child);
    }
  }

  void printTreeNames(const TransformTree &tree, std::string indent) const
  {
    indent += "  ";
    std::cout << indent << tree.name << std::endl;
    for(std::vector<TransformTree>::const_iterator child = tree.children.begin(); child != tree.children.end(); ++child)
      printTreeNames(*child, indent);
  }
};

} //namespace SquirrelMotionPlanner

#endif //SQUIRREL_MOTION_PLANNING_COLLISION_CHECKER_HPP_

