#ifndef SQUIRREL_8DOF_PLANNER_H_
#define SQUIRREL_8DOF_PLANNER_H_

#include <squirrel_8dof_planner/squirrel_8dof_planner_structures.h>

#include <string>
#include <vector>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit_msgs/PlanningScene.h>
#include <interactive_markers/interactive_marker_server.h>
#include <birrt_star_algorithm/birrt_star.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

namespace SquirrelMotionPlanner
{

class Planner
{
  /*
   * ROS message handling
   */
  ros::NodeHandle nh, nhPrivate;
  ros::Publisher publisherOccupancyMap, publisherPlanningScene, publisher2DPath, publisherTrajectory;
  ros::Subscriber subscriberBase, subscriberArm, subscriberGoalEndEffector, subscriberGoalPose, subscriberGoalMarkerExecute;
  interactive_markers::InteractiveMarkerServer interactiveMarkerServer;
  visualization_msgs::InteractiveMarker interactiveMarker;
  std::vector<Real> poseGoalMarker;

  /*
   * General search settings
   */
  std::vector<Real> poseInit;
  Real distanceBirrtStarPlanning;
  Real obstacleInflationRadius;

  /*
   *  Current robot and planning poses
   */
  std::vector<Real> poseCurrent;
  std::vector<Real> poseGoal;
  std::vector<std::vector<Real> > poses;

  /*
   * Octomap
   */
  octomap::OcTree octree;
  std::vector<std::vector<bool> > occupancyMap;
  Real minX, minY, minZ, maxX, maxY, maxZ;
  Int cellMinX, cellMinY, cellMaxX, cellMaxY, cellMinZ, cellMaxZ;

  /*
   * AStar
   */
  AStarPath2D AStarPath;
  Cell2D startCell, goalCell;
  std::vector<std::vector<AStarNode> > AStarNodes;
  std::vector<Cell2D> openListNodes;
  Real AStarPathSmoothingFactor; //good value is between 1.5 and 2.5
  Real AStarPathSmoothingDistance; //maximum distance from intermediate corners that is used for smoothing
  Real AStarPathSmoothedPointDistance; //approximate final distance between points

  /*
   * Bi2RRTStar
   */
  birrt_star_motion_planning::BiRRTstarPlanner birrtStarPlanner;
  Int birrtStarPlanningNumber;

public:

  /**
   * @brief Constructur initializes everything, mainly calls all initialize* methods. All planning and communication is done via ROS.
   */
  Planner();

private:

  /**
   * @brief Loads all necessary parameters from the ros parameter server and initializeses further internal parameters.
   */
  void initializeParameters();

  /**
   * @brief Advertises and subscribes to all topics used for getting the current robot pose and handle planning requests.
   */
  void initializeMessageHandling();

  /**
   * @brief Initializeses the map used for the 2D astar planning.
   */
  void initializeAStarPlanning();

  /**
   * @brief Initializses and interactive marker that can be used for planning to specific end effector poses in rviz.
   */
  void initializeInteractiveMarker();

  /**
   * @brief Publishes the loaded octomap to the planning_scene topic for collision checks of moveit used in birrtstar.
   */
  void loadOctomapToMoveit();

  /**
   * @brief Publishes the occupancy map that was generated from the octomap, which is used for the 2D planning.
   */
  void publishOccupancyMap() const;

  /**
   * @brief Publishes a 2D path that follows the base of the full trajectory after planning.
   */
  void publish2DPath() const;

  /**
   * @brief Publishes a vector of subsequent poses from the current robot position to the requested goal after planning.
   */
  void publishTrajectory() const;

  /**
   * @brief Handler for the base position of the robot.
   * @param msg ROS message that contains the base pose information.
   */
  void subscriberBaseHandler(const sensor_msgs::JointState &msg);

  /**
   * @brief Handler for the arm joints of the robot.
   * @param msg ROS message that contains the joint angles of  the arm information.
   */
  void subscriberArmHandler(const sensor_msgs::JointState &msg);

  /**
   * @brief Handler for starting the planner to the requested end effector position.
   * @param msg ROS message that contains the 6D pose of the requested end effector goal position given in [x,y,z,R,P,Y].
   */
  void subscriberGoalEndEffectorHandler(const std_msgs::Float64MultiArray &msg);

  /**
   * @brief Used to signal a request for finding a trajectory to the current interactive marker pose for the endeffector.
   * @param msg Emtpy message used only as a signal.
   */
  void subscriberGoalMarkerExecuteHandler(const std_msgs::Empty &msg);

  /**
   * @brief Handler for starting the planner to the requested goal pose.
   * @param msg ROS message that contains the full 8D pose of the robot given as
   * [base_x, base_y, base_theta, arm_joint1, arm_joint2, arm_joint3, arm_joint4, arm_joint5].
   */
  void subscriberGoalPoseHandler(const std_msgs::Float64MultiArray &msg);

  /**
   * @brief Handles the feedback message of the interactive marker.
   * @param msg Contains the 6D pose of the marker given in 7D with the 3D pose and 4D quaternion.
   */
  void interactiveMarkerHandler(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg);

  /**
   * @brief Loops through the loaded octree and creates an occupancy map if any node is occupied between minZ and maxZ.
   * Also adds all neighbours to the AStarNodes map.
   */
  void create2DMap();

  /**
   * @brief Loops through the occupancy map and inflates it by a radius given by obstacleInflationRadius.
   */
  void inflate2DMap();

  /**
   * @brief Finds a combined 2D and 8D trajectory from the current robot pose
   * such that it reaches a requested end effector pose, saves it as an 8D trajectory to poses, and publishes the trajectory.
   * @param poseEndEffector The 6D pose of the end effector to which the robot should plan, given in [x, y, z, R, P, Y].
   * @return False if no trajectory could be found or if no free pose could be found to the end effector, returns true otherwise.
   */
  bool findTrajectoryEndEffector(const std::vector<Real> &poseEndEffector);

  /**
   * @brief Finds a combined 2D and 8D trajectory from the current robot pose
   * such that it reaches a requested goal pose, saves it to poses, and publishes the trajectory.
   * @param poseGoalNew The 8D goal pose to which the robot should plan.
   * @return False if no trajectory could be found and true otherwise.
   */
  bool findTrajectoryPose(const std::vector<Real> &poseGoalNew);

  /**
   * @brief Uses inverse kinematic to find a possible 8D pose for a requested 6D end effector pose.
   * @param poseEndEffector The 6D pose, given in [x, y, z, R, P, Y], for which a free pose should be found.
   * @return Returns true if a free pose could be found and false otherwise.
   */
  bool findGoalPose(const std::vector<Real> &poseEndEffector);

  /**
   * @brief Finds a 2D and 8D trajectory to the goal pose saved in poseGoal,
   * it is called by both findTrajectoryPose and findTrajectoryEndEffector which both save a new goal pose to poseGoal.
   * @return True if a free trajectory could be found, returns false if either the 2D or the 8D search fails.
   */
  bool findTrajectory();

  /**
   * @brief Finds a 2D path from the cell of the current robot position (poseCurrent) to the cell of poseGoal,
   * then shortens the path by distanceBirrtStarPlanning, smoothens the path and saves it as 8D poses using poseInit for the arm joints.
   * @return True of a free path could be found, false otherwise.
   */
  bool findTrajectoryRetractedArm();

  /**
   * @brief Finds an 8D path using birrt star from the last item in poses to poseGoal and also adds the remaining path to poses.
   * @return True if a path could be found, false if the planner could not be initialized or no plan could be found.
   */
  bool findTrajectoryBirrtStar();

  /**
   * @brief Finds an A* actual path between two points and saves it to AStarPath.
   * If no path could be found, the flag valid is set to false and true otherwise.
   * @param startPoint The start point for the path planning in metric coordinates.
   * @param goalPoint The goal point for the path planning in metric coordinates.
   */
  void findAStarPath(const Tuple2D startPoint, const Tuple2D goalPoint);

  /**
   * @brief Expands an AStarNode by its neighbours, if they are free and adds them to openList.
   * @param node The AStarNode that is to be expanded by its neighbours.
   */
  void expandAStarNode(const AStarNode &node);

  /**
   * @brief Inserts a node to openList and sorts it using an integer based binary tree according to its F cost value.
   * @param node AStarNode to be inserted into the open list.
   */
  void openListInsertNode(AStarNode &node);

  /**
   * @brief Updates a node in openList and sorts it using an integer based binary tree according to its F cost value.
   * @param node AStarNode to be updated within the open list.
   */
  void openListUpdateNode(AStarNode &node);

  /**
   * @brief Removes the front item of openList and resorts it if necessary.
   */
  void openListRemoveFrontNode();

  /**
   * @brief Goes over AStarNodes starting at the goal node, finds the actual cells belonging to the path, and saves it to AStarPath
   */
  void constructAStarPath();

  /**
   * @brief Loops through AStarPath, finds shortest connected line segments, smoothens corners between segments, and saves the trajectory to poses.
   */
  void getSmoothTrajFromAStarPath();

  /**
   * @brief Moves directly between two points and checks if all cells on the connecting line are free according to occupancyMap.
   * @param pointStart The starting point in metric coordinates of the line segment.
   * @param pointEnd The end point in metric coordinates of the line segment.
   * @return False if any occpuied call is found, true otherwise.
   */
  bool isConnectionLineFree(const Tuple2D &pointStart, const Tuple2D &pointEnd);


  // ******************** INLINES ********************


  /**
   * @brief Loads a parameter from the ROS parameter server and saves it to a member varible .
   * If the parameter could not be found or has an invalid name the default value is used instead.
   * @param name The name of the paramter to be loaded by the global node handle nh.
   * @param member The member variable to which the parameter should be saved.
   * @param defaultValue The value which is used saved to member in case the parameter could not be loaded.
   */
  void loadParameter(const string &name, Real &member, const Real &defaultValue);

  /**
   * @brief Checks if an octree node is occupied in the currently loaded octree.
   * @param key The octomap::OcTreeKey to the octree node to be checked.
   * @return Returns true if the node saved in key is occupied, false if the node is occupied or NULL.
   */
  bool isOctreeNodeOccupied(const octomap::OcTreeKey &key) const;

  /**
   * @brief Converts a cell to metric coordinates according to minX, minY and the resolution of occupancyMap (or octree, which is the same)
   * @param cell The cell which is to converted into a metric point.
   * @return Retuns a Tuple2D which contains the metric coordinates of the cell.
   */
  Tuple2D getPointFromCell(const Cell2D &cell) const;

  /**
   * @brief Converts a metric point to an integer cell according to minX, minY and the resolution of occupancyMap (or octree, which is the same)
   * @param point The point which is to converted into an integer cell.
   * @return Retuns a Cell2D which contains the integer coordinates of the point.
   */
  Cell2D getCellFromPoint(const Tuple2D &point) const;

  /**
   * @brief Finds the distance between two cells in cell coordinates.
   * The distance is not metric, but according to the rule that the distance between two adjecent cells is 1.
   * @param cell1 The first cell for finding the distance.
   * @param cell2 The second cell for finding the distance.
   * @return Returns the distance between two cells.
   */
  Real distanceCells(const Cell2D &cell1, const Cell2D &cell2);

  /**
   * @brief Checks if a cell is occpuied according to occupancyMap.
   * @param cell The cell to be checked for occupancy.
   * @return True if the corresponding cell in occupancyMap is true, false otherwise.
   */
  bool isCellOccupied(const Cell2D &cell) const;

  /**
   * @brief Updates the F cost value of an AStarNode accoring to its G cost and an optimal admissable heuristics for grid maps.
   * @param node AStarNode for which the F cost value is to be updated.
   */
  void updateFCost(AStarNode &node) const;
};

} //namespace SquirrelMotionPlanner

#endif /* SQUIRREL_8DOF_PLANNER_PLANNER_H_ */
