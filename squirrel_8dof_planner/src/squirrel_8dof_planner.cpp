#include <squirrel_8dof_planner/squirrel_8dof_planner.h>

namespace SquirrelMotionPlanner
{

// ******************** PUBLIC MEMBERS ********************

Planner::Planner() :
    nhPrivate("~"), birrtStarPlanner("robotino_robot"), interactiveMarkerServer("endeffector_goal")
{
  ros::Duration(5).sleep();

  initializeParameters();
  if (posesFolding.size() == 0)
  {
    ros::shutdown();
    return;
  }

  initializeMessageHandling();
  initializeInteractiveMarker();

  UInt publisherCounter = 0;
  while (publisherPlanningScene.getNumSubscribers() == 0)
  {
    ros::WallDuration(1.0).sleep();
    ++publisherCounter;
    if (publisherCounter > 20)
      break;
  }
}

// ******************** PRIVATE MEMBERS ********************

void Planner::initializeParameters()
{
  poseCurrent.resize(8, 0.0);
  poseGoal.resize(8, 0.0);
  poseGoalMarker.resize(6, 0.0);
  birrtStarPlanningNumber = 0;

  std::vector<Real> posesFoldingList;
  loadParameter("trajectory_folding_arm", posesFoldingList, std::vector<Real>());
  UInt counter = 0;
  if (posesFoldingList.size() % 5 == 0)
  {
    posesFolding.resize(posesFoldingList.size() / 5, std::vector<Real>(5));
    for (UInt i = 0; i < posesFolding.size(); ++i)
      for (UInt j = 0; j < 5; ++j)
      {
        posesFolding[i][j] = posesFoldingList[counter];
        ++counter;
      }
  }
  else
  {
    ROS_ERROR("Parameter list 'trajectory_folding_arm' is not divisible by 5. Folding arm trajectory has not been loaded.");
  }

  loadParameter("occupancy_height_min", mapMinZ, 0.0);
  loadParameter("occupancy_height_max", mapMaxZ, 2.0);
  loadParameter("astar_safety_distance", obstacleInflationRadius, 0.3);
  loadParameter("astar_smoothing_factor", AStarPathSmoothingFactor, 2.0);
  loadParameter("astar_smoothing_distance", AStarPathSmoothingDistance, 0.2);
  loadParameter("astar_final_smoothed_point_distance", AStarPathSmoothedPointDistance, 0.02);
  loadParameter("distance_birrt_star_planning", distance8DofPlanning, 1.2);
}

void Planner::initializeMessageHandling()
{
  publisherPlanningScene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  publisherOccupancyMap = nhPrivate.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1);
  publisher2DPath = nhPrivate.advertise<nav_msgs::Path>("path_2d", 10);
  publisherTrajectory = nhPrivate.advertise<std_msgs::Float64MultiArray>("robot_trajectory", 10);
  subscriberBase = nh.subscribe("/base/joint_states", 1, &Planner::subscriberBaseHandler, this);
  subscriberArm = nh.subscribe("/arm_controller/joint_states", 1, &Planner::subscriberArmHandler, this);
  subscriberGoalMarkerExecute = nhPrivate.subscribe("goal_marker_execute", 1, &Planner::subscriberGoalMarkerExecuteHandler, this);
  subscriberGoal = nhPrivate.subscribe("goal", 1, &Planner::subscriberGoalHandler, this);
  serviceClientOctomap = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");
}

void Planner::initializeAStarPlanning()
{
  createOccupancyMapFromOctomap();
  inflateOccupancyMap();
}

void Planner::initializeInteractiveMarker()
{
  interactiveMarker.header.frame_id = "hand_wrist_link";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "endeffector_goal_marker";

  visualization_msgs::InteractiveMarkerControl markerControl;
  markerControl.always_visible = true;

  visualization_msgs::Marker endeffectorMarker;
  endeffectorMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
  endeffectorMarker.color.r = endeffectorMarker.color.b = 0.0;
  endeffectorMarker.color.g = 0.8;
  endeffectorMarker.color.a = 0.5;
  endeffectorMarker.pose.position.x = 0.0;
  endeffectorMarker.pose.position.y = 0.0;
  endeffectorMarker.pose.position.z = 0.0;
  endeffectorMarker.pose.orientation.x = 0.0;
  endeffectorMarker.pose.orientation.y = 0.0;
  endeffectorMarker.pose.orientation.z = 0.0;
  endeffectorMarker.pose.orientation.w = 1.0;
  endeffectorMarker.scale.x = 1.0;
  endeffectorMarker.scale.y = 1.0;
  endeffectorMarker.scale.z = 1.0;
  endeffectorMarker.mesh_resource = "package://squirrel_8dof_planner/config/squirrel-hand.dae";
  endeffectorMarker.mesh_use_embedded_materials = false;
  markerControl.markers.push_back(endeffectorMarker);
  interactiveMarker.controls.push_back(markerControl);

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  interactiveMarkerServer.insert(interactiveMarker, boost::bind(&Planner::interactiveMarkerHandler, this, _1));
  interactiveMarkerServer.applyChanges();
}

void Planner::publishOccupancyMap() const
{
  if (publisherOccupancyMap.getNumSubscribers() == 0)
    return;

  nav_msgs::OccupancyGrid msg;

  msg.info.width = occupancyMap.size();
  msg.info.height = occupancyMap[0].size();
  msg.info.resolution = mapResolution;
  msg.info.origin.position.x = mapMinX;
  msg.info.origin.position.y = mapMinY;
  msg.info.origin.position.z = 0.0;
  msg.info.map_load_time = ros::Time::now();
  msg.header.seq = 0;
  msg.header.frame_id = "origin";
  msg.header.stamp = ros::Time::now();

  msg.data.resize(msg.info.width * msg.info.height);
  UInt counter = 0;
  for (UInt j = 0; j < occupancyMap[0].size(); ++j)
  {
    for (UInt i = 0; i < occupancyMap.size(); ++i)
    {
      msg.data[counter] = occupancyMap[i][j] == true ? 100 : 0;
      ++counter;
    }
  }

  publisherOccupancyMap.publish(msg);
}

void Planner::publish2DPath() const
{
  if (publisher2DPath.getNumSubscribers() == 0)
    return;

  nav_msgs::Path msg;

  msg.header.frame_id = "origin";
  msg.header.seq = 0;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.pose.position.z = 0.01;
  for (int i = 0; i < posesTrajectory.size(); ++i)
  {
    poseStamped.pose.position.x = posesTrajectory[i][0];
    poseStamped.pose.position.y = posesTrajectory[i][1];
    msg.poses.push_back(poseStamped);
  }

  publisher2DPath.publish(msg);
}

void Planner::publishTrajectory() const
{
  if (publisherTrajectory.getNumSubscribers() == 0 || posesTrajectory.size() <= 1)
    return;

  std_msgs::Float64MultiArray msg;

  msg.layout.data_offset = 0;
  msg.layout.dim.resize(2);
  msg.layout.dim[0].label = "pose";
  msg.layout.dim[0].size = posesTrajectory.size() - 1;
  msg.layout.dim[0].stride = (posesTrajectory.size() - 1) * 8;
  msg.layout.dim[1].label = "joint";
  msg.layout.dim[1].size = 8;
  msg.layout.dim[1].stride = 8;

  msg.data.resize((posesTrajectory.size() - 1) * 8);

  UInt index = 0;
  for (UInt i = 1; i < posesTrajectory.size(); ++i)
  {
    for (UInt j = 0; j < 8; ++j)
    {
      msg.data[index] = posesTrajectory[i][j];
      ++index;
    }
  }

  publisherTrajectory.publish(msg);
}

void Planner::subscriberBaseHandler(const sensor_msgs::JointState &msg)
{
//  poseCurrent[0] = msg.position[0];
//  poseCurrent[1] = msg.position[1];
//  poseCurrent[2] = msg.position[2];
}

void Planner::subscriberArmHandler(const sensor_msgs::JointState &msg)
{
  poseCurrent[3] = msg.position[0];
  poseCurrent[4] = msg.position[1];
  poseCurrent[5] = msg.position[2];
  poseCurrent[6] = msg.position[3];
  poseCurrent[7] = msg.position[4];
}

void Planner::subscriberFoldArmHandler(const std_msgs::Empty &msg)
{
  posesTrajectory.clear();
  if (!findTrajectoryFoldArm())
  {
    ROS_WARN("No 8D path could be found to the stretched position.");
    return;
  }

  publishTrajectory();
}

void Planner::subscriberGoalHandler(const std_msgs::Float64MultiArray &msg)
{
  if (msg.data.size() == 8)
  {
    ROS_INFO("Start planning to pose: %f, %f, %f, %f, %f, %f, %f, %f", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
    if (!serviceCallGetOctomap())
      return;

    poseGoal = msg.data;

    findTrajectoryFull();
  }
  else if (msg.data.size() == 6)
  {
    ROS_INFO("Start planning to endeffector pose: %f, %f, %f, %f, %f, %f", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
    if (!serviceCallGetOctomap())
      return;

    if (!findGoalPose(msg.data))
      return;

    findTrajectoryFull();
  }
  else
    ROS_WARN("Could not start planning, because the message has a wrong dimension. Provided dimension: %u, expected dimension: 8 (plan to pose) or 6 (plan to end effector).",
             static_cast<UInt>(msg.data.size()));
}

void Planner::subscriberGoalMarkerExecuteHandler(const std_msgs::Empty &msg)
{
  if (!serviceCallGetOctomap())
    return;

  if (!findGoalPose(poseGoalMarker))
    return;

  findTrajectoryFull();
}

bool Planner::serviceCallGetOctomap()
{
  octomap_msgs::GetOctomapRequest req;
  octomap_msgs::GetOctomapResponse res;
  if (!serviceClientOctomap.call(req, res))
  {
    ROS_ERROR("Could not receive a new octomap from 'octomap_server_node', planning will not be executed.");
    return false;
  }

  octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(res.map));
  if (octree == NULL)
  {
    ROS_ERROR("Received octomap is empty, planning will not be executed.");
    return false;
  }

  createOccupancyMapFromOctomap();
  inflateOccupancyMap();
  createAStarNodesMap();

  moveit_msgs::PlanningScene msgScene;
  msgScene.name = "octomap_scene";
  msgScene.is_diff = true;
  msgScene.world.octomap.header.frame_id = "origin";
  msgScene.world.octomap.header.stamp = ros::Time::now();
  msgScene.world.octomap.header.seq = 0;
  msgScene.world.octomap.octomap.header = msgScene.world.octomap.header;
  msgScene.world.octomap.origin.orientation.w = 1.0;
  msgScene.world.octomap.origin.orientation.x = 0.0;
  msgScene.world.octomap.origin.orientation.y = 0.0;
  msgScene.world.octomap.origin.orientation.z = 0.0;
  msgScene.world.octomap.origin.position.x = 0.0;
  msgScene.world.octomap.origin.position.y = 0.0;
  msgScene.world.octomap.origin.position.z = 0.0;
  msgScene.world.octomap.octomap = res.map;

  publisherPlanningScene.publish(msgScene);
  publishOccupancyMap();

  delete octree;
  return true;
}

void Planner::interactiveMarkerHandler(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg)
{
  poseGoalMarker[0] = msg->pose.position.x;
  poseGoalMarker[1] = msg->pose.position.y;
  poseGoalMarker[2] = msg->pose.position.z;
  tf::Matrix3x3 mat(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
  double r, p, y;
  mat.getRPY(r, p, y);
  poseGoalMarker[3] = r;
  poseGoalMarker[4] = p;
  poseGoalMarker[5] = y;
}

void Planner::createOccupancyMapFromOctomap()
{
  Real octreeMinZ, octreeMaxZ;
  Int octreeKeyMinX, octreeKeyMinY, octreeKeyMinZ;
  Int octreeKeyMaxX, octreeKeyMaxY, octreeKeyMaxZ;

  octree->getMetricMin(mapMinX, mapMinY, octreeMinZ);
  octree->getMetricMax(mapMaxX, mapMaxY, octreeMaxZ);
  mapResolution = octree->getResolution();
  mapResolutionRecip = 1.0 / mapResolution;

  octomap::OcTreeKey keyMinPlane = octree->coordToKey(mapMinX + mapResolution * 0.5, mapMinY + mapResolution * 0.5, octreeMinZ + mapResolution * 0.5);
  octomap::OcTreeKey keyMaxPlane = octree->coordToKey(mapMaxX - mapResolution * 0.5, mapMaxY - mapResolution * 0.5, octreeMaxZ - mapResolution * 0.5);

  octreeKeyMinX = keyMinPlane[0];
  octreeKeyMinY = keyMinPlane[1];
  octreeKeyMaxX = keyMaxPlane[0];
  octreeKeyMaxY = keyMaxPlane[1];
  octreeKeyMinZ = octree->coordToKey(0.0, 0.0, mapMinZ + mapResolution * 0.5)[2];
  octreeKeyMaxZ = octree->coordToKey(0.0, 0.0, mapMaxZ - mapResolution * 0.5)[2];

  occupancyMap.clear();
  occupancyMap.resize(octreeKeyMaxX - octreeKeyMinX + 1, std::vector<bool>(octreeKeyMaxY - octreeKeyMinY + 1, false));

  octomap::OcTreeKey key;
  for (UInt x = 0; x < occupancyMap.size(); ++x)
  {
    for (UInt y = 0; y < occupancyMap[0].size(); ++y)
    {
      for (UInt z = 0; z < octreeKeyMaxZ - octreeKeyMinZ; ++z)
      {
        key[0] = x + octreeKeyMinX;
        key[1] = y + octreeKeyMinY;
        key[2] = z + octreeKeyMinZ;
        if (isOctreeNodeOccupied(key))
        {
          occupancyMap[x][y] = true;
          z = octreeKeyMaxZ - octreeKeyMinZ;
        }
      }
    }
  }
}

void Planner::inflateOccupancyMap()
{
  std::vector<std::vector<Real> > inflationMap(occupancyMap.size(), std::vector<Real>(occupancyMap[0].size(), -1.0));
  std::deque<Cell2D> queue;

  for (UInt x = 0; x < occupancyMap.size(); ++x)
    for (UInt y = 0; y < occupancyMap[0].size(); ++y)
      if (occupancyMap[x][y])
      {
        queue.push_back(Cell2D(x, y));
        inflationMap[x][y] = 0.0;
      }

  while (queue.size() > 0)
  {
    const Cell2D cellCenter = queue[0];
    queue.pop_front();

    Cell2D cell(cellCenter.x - 1, cellCenter.y - 1);

    if (cell.x < inflationMap.size() && cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + M_SQRT2 * mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y;
    if (cell.x < inflationMap.size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y + 1;
    if (cell.x < inflationMap.size() && cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + M_SQRT2 * mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x;
    if (cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x + 1;
    if (cell.x < inflationMap.size() && cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + M_SQRT2 * mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y;
    if (cell.x < inflationMap.size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y - 1;
    if (cell.x < inflationMap.size() && cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + M_SQRT2 * mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x;
    if (cell.y < inflationMap[0].size())
    {
      Real newDistance = inflationMap[cellCenter.x][cellCenter.y] + mapResolution;
      if (newDistance < obstacleInflationRadius && (inflationMap[cell.x][cell.y] < 0.0 || inflationMap[cell.x][cell.y] > newDistance))
      {
        inflationMap[cell.x][cell.y] = newDistance;
        queue.push_back(Cell2D(cell.x, cell.y));
      }
    }
  }

  for (UInt x = 0; x < inflationMap.size(); ++x)
    for (UInt y = 0; y < inflationMap[0].size(); ++y)
      if (inflationMap[x][y] > 0.0)
        occupancyMap[x][y] = true;
}

void Planner::createAStarNodesMap()
{
  AStarNodes.clear();
  AStarNodes.resize(occupancyMap.size(), std::vector<AStarNode>(occupancyMap[0].size()));

  for (UInt x = 0; x < AStarNodes.size(); ++x)
  {
    for (UInt y = 0; y < AStarNodes[0].size(); ++y)
    {
      AStarNodes[x][y].cell = Cell2D(x, y);
      if (occupancyMap[x][y])
      {
        AStarNodes[x][y].occupied = AStarNodes[x][y].closed = true;
        continue;
      }

      if (x + 1 < AStarNodes.size() && y + 1 < AStarNodes[0].size() && !occupancyMap[x + 1][y + 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x + 1][y + 1], M_SQRT2));
      if (x + 1 < AStarNodes.size() && !occupancyMap[x + 1][y])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x + 1][y], 1.0));
      if (x + 1 < AStarNodes.size() && y - 1 < AStarNodes[0].size() && !occupancyMap[x + 1][y - 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x + 1][y - 1], M_SQRT2));
      if (y + 1 < AStarNodes[0].size() && !occupancyMap[x][y + 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x][y + 1], 1.0));
      if (y - 1 < AStarNodes[0].size() && !occupancyMap[x][y - 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x][y - 1], 1.0));
      if (x - 1 < AStarNodes.size() && y + 1 < AStarNodes[0].size() && !occupancyMap[x - 1][y + 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x - 1][y + 1], M_SQRT2));
      if (x - 1 < AStarNodes.size() && !occupancyMap[x - 1][y])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x - 1][y], 1.0));
      if (x - 1 < AStarNodes.size() && y - 1 < AStarNodes[0].size() && !occupancyMap[x - 1][y - 1])
        AStarNodes[x][y].neighbors.push_back(std::make_pair(&AStarNodes[x - 1][y - 1], M_SQRT2));
    }
  }
}

bool Planner::findGoalPose(const std::vector<Real> &poseEndEffector)
{
  std::vector<std::pair<Real, Real> > endEffectorDeviations(6);
  endEffectorDeviations[0] = std::make_pair<Real, Real>(-0.005, 0.005);
  endEffectorDeviations[1] = std::make_pair<Real, Real>(-0.005, 0.005);
  endEffectorDeviations[2] = std::make_pair<Real, Real>(-0.005, 0.005);
  endEffectorDeviations[3] = std::make_pair<Real, Real>(-0.025, 0.025);
  endEffectorDeviations[4] = std::make_pair<Real, Real>(-0.025, 0.025);
  endEffectorDeviations[5] = std::make_pair<Real, Real>(-0.025, 0.025);

  std::vector<Real> poseInitializer(8);
  copyArmToRobotPose(posesFolding.back(), poseInitializer);
  poseInitializer[0] = poseCurrent[0];
  poseInitializer[1] = poseCurrent[1];
  poseInitializer[2] = poseCurrent[2];

  poseGoal = birrtStarPlanner.getFullPoseFromEEPose(poseEndEffector, endEffectorDeviations, poseInitializer);
  if (poseGoal.size() != 8)
  {
    ROS_WARN("No free goal configuration could be found for the requested end effector pose.");
    return false;
  }
  return true;
}

void Planner::findTrajectoryFull()
{
  const Cell2D goalCell = getCellFromPoint(Tuple2D(poseGoal[0], poseGoal[1]));
  if (occupancyMap[goalCell.x][goalCell.y])
  {
    ROS_WARN("The requested goal configuration is occupied. No plan could be found.");
    return;
  }

  posesTrajectory.clear();

  if (Tuple2D(poseGoal[0], poseGoal[1]).distance(Tuple2D(poseCurrent[0], poseCurrent[1])) < distance8DofPlanning)
  {
    if (!findTrajectory8D(poseCurrent, poseGoal))
    {
      ROS_WARN("No 8D path could be found to the requested goal pose.");
      return;
    }
  }
  else
  {
//    if (!findTrajectoryFoldArm())
//    {
//      ROS_WARN("No 8D path could be found to the stretched position.");
//      return;
//    }

    if (!findTrajectory2D())
    {
      ROS_WARN("No 2D path could be found to the requested goal pose.");
      return;
    }

    if (!findTrajectory8D(posesTrajectory.back(), poseGoal))
    {
      ROS_WARN("No 8D path could be found to the requested goal pose.");
      return;
    }
  }

  publish2DPath();
  publishTrajectory();
}

bool Planner::findTrajectoryFoldArm()
{
  std::vector<Real> poseTmp(poseCurrent);
  copyArmToRobotPose(posesFolding[0], poseTmp);

  if (!findTrajectory8D(poseCurrent, poseTmp))
    return false;

  for (UInt i = 0; i < posesFolding.size(); ++i)
  {
    posesTrajectory.push_back(poseCurrent);
    copyArmToRobotPose(posesFolding[i], posesTrajectory.back());
  }

  indexLastFolding = posesTrajectory.size() - 1;

  return true;
}

bool Planner::findTrajectory2D()
{
  findAStarPath(Tuple2D(poseCurrent[0], poseCurrent[1]), Tuple2D(poseGoal[0], poseGoal[1]));
  if (!AStarPath.valid)
    return false;

  Real distanceFromGoal = 0.0;
  while (distanceFromGoal < distance8DofPlanning && AStarPath.cells.size() > 2)
  {
    distanceFromGoal += AStarPath.points.back().distance(AStarPath.points.end()[-2]);
    AStarPath.points.pop_back();
    AStarPath.cells.pop_back();
  }

  getSmoothTrajFromAStarPath();
  return true;
}

bool Planner::findTrajectory8D(const std::vector<Real> &poseStart, const std::vector<Real> &poseGoal)
{
  if (birrtStarPlanningNumber > 0)
    birrtStarPlanner.reset_planner_and_config();

  std::vector<Real> dimX(2), dimY(2);
  dimX[0] = mapMinX;
  dimX[1] = mapMaxX;
  dimY[0] = mapMinY;
  dimY[1] = mapMaxY;

  birrtStarPlanner.setPlanningSceneInfo(dimX, dimY, "scenario");

  if (!birrtStarPlanner.init_planner(poseStart, poseGoal, 1))
    return false;

  if (!birrtStarPlanner.run_planner(1, false, 400, false, 0.0, birrtStarPlanningNumber))
    return false;

  std::vector<std::vector<Real> > &trajectoryBirrtStar = birrtStarPlanner.getJointTrajectoryRef();

  for (UInt i = 1; i < trajectoryBirrtStar.size(); ++i)
    posesTrajectory.push_back(trajectoryBirrtStar[i]);

  ++birrtStarPlanningNumber;
  return true;
}

void Planner::findAStarPath(const Tuple2D startPoint, const Tuple2D goalPoint)
{
  AStarPath.valid = false;
  AStarPath.cells.clear();
  AStarPath.points.clear();

  const UInt sizeX = AStarNodes.size();
  const UInt sizeY = AStarNodes[0].size();
  for (UInt x = 0; x < sizeX; ++x)
    for (UInt y = 0; y < sizeY; ++y)
      if (!AStarNodes[x][y].occupied)
        AStarNodes[x][y].open = AStarNodes[x][y].closed = false;

  AStarCellStart = getCellFromPoint(startPoint);
  AStarCellGoal = getCellFromPoint(goalPoint);

  if (AStarCellStart == AStarCellGoal)
  {
    AStarPath.cells.push_back(AStarCellGoal);
    AStarPath.points.push_back(getPointFromCell(AStarCellGoal));
    AStarPath.valid = true;
    return;
  }

  openListNodes.clear();
  openListNodes.push_back(Cell2D(-1, -1));
  openListNodes.push_back(AStarCellStart);

  //initialize starting nodes
  AStarNodes[AStarCellStart.x][AStarCellStart.y].cellParent = Cell2D(-1, -1);
  AStarNodes[AStarCellStart.x][AStarCellStart.y].g = 0;
  AStarNodes[AStarCellStart.x][AStarCellStart.y].indexOpenList = 1;
  updateFCost(AStarNodes[AStarCellStart.x][AStarCellStart.y]);

  while (openListNodes.size() > 1)
  {
    if (AStarNodes[openListNodes[1].x][openListNodes[1].y].cell == AStarCellGoal)
    {
      constructAStarPath();
      AStarPath.valid = true;
      return;
    }

    openListRemoveFrontNode();
    AStarNodes[openListNodes[1].x][openListNodes[1].y].closed = true;
    expandAStarNode(AStarNodes[openListNodes[1].x][openListNodes[1].y]);
  }

  ROS_WARN("No path found after expanding all nodes.");
}

void Planner::expandAStarNode(const AStarNode &node)
{
  for (UInt i = 0; i < node.neighbors.size(); ++i)
  {
    AStarNode &nodeNew = *node.neighbors[i].first;
    if (!nodeNew.closed)
    {
      if (!nodeNew.open)
      {
        nodeNew.open = true;
        nodeNew.cellParent = node.cell;
        nodeNew.g = node.g + node.neighbors[i].second;
        updateFCost(nodeNew);
        openListInsertNode(nodeNew);
      }
      else if (nodeNew.g > node.g + node.neighbors[i].second)
      {
        nodeNew.cellParent = node.cell;
        nodeNew.g = node.g + node.neighbors[i].second;
        updateFCost(nodeNew);
        openListUpdateNode(nodeNew);
      }
    }
  }
}

void Planner::openListInsertNode(AStarNode &node)
{
  openListNodes.push_back(node.cell);
  UInt index = openListNodes.size() - 1;
  UInt indexHalf = index / 2;
  node.indexOpenList = index;

  while (indexHalf > 0 && AStarNodes[openListNodes[index].x][openListNodes[index].y].f < AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].f)
  {
    Cell2D nodeBuffer = openListNodes[index];
    openListNodes[index] = openListNodes[indexHalf];
    openListNodes[indexHalf] = nodeBuffer;
    AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexHalf;
    AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].indexOpenList = index;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void Planner::openListUpdateNode(AStarNode &node)
{
  UInt index = node.indexOpenList;
  UInt indexHalf = index / 2;

  while (indexHalf > 0 && AStarNodes[openListNodes[index].x][openListNodes[index].y].f < AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].f)
  {
    Cell2D nodeBuffer = openListNodes[index];
    openListNodes[index] = openListNodes[indexHalf];
    openListNodes[indexHalf] = nodeBuffer;
    AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexHalf;
    AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].indexOpenList = index;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void Planner::openListRemoveFrontNode()
{
  openListNodes[1] = openListNodes.back();
  openListNodes.pop_back();
  AStarNodes[openListNodes[1].x][openListNodes[1].y].indexOpenList = 1;

  UInt index = 1, indexDouble = 2, indexDoubleOne = 3;
  while (indexDouble < openListNodes.size())
  {
    if (indexDoubleOne == openListNodes.size())
    {
      if (AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f)
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDouble];
        openListNodes[indexDouble] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDouble;
        AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].indexOpenList = index;
        index = indexDouble;
      }
      else
        break;
    }
    else if (AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f
        || AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].f)
    {
      if (AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f < AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].f)
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDouble];
        openListNodes[indexDouble] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDouble;
        AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].indexOpenList = index;
        index = indexDouble;
      }
      else
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDoubleOne];
        openListNodes[indexDoubleOne] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDoubleOne;
        AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].indexOpenList = index;
        index = indexDoubleOne;
      }
    }
    else
      break;
    indexDouble = index * 2;
    indexDoubleOne = indexDouble + 1;
  }
}

void Planner::constructAStarPath()
{
  std::deque<Cell2D> cellList;

  Cell2D cellTemp = AStarCellGoal;
  while (true)
  {
    cellList.push_front(cellTemp);
    if (cellTemp == AStarCellStart)
      break;
    cellTemp = AStarNodes[cellTemp.x][cellTemp.y].cellParent;
  }

  for (std::deque<Cell2D>::const_iterator it = cellList.begin(); it != cellList.end(); ++it)
  {
    AStarPath.cells.push_back(*it);
    AStarPath.points.push_back(getPointFromCell(*it));
  }
}

void Planner::getSmoothTrajFromAStarPath()
{
  std::vector<LineSegment2D> pathSegments;
  std::vector<ParametricFunctionCubic2D> connections;
  UInt indexCurrent = 0;
  Tuple2D startPointCurrent = AStarPath.points[0];

  //find maximal free connecting way points along astar path
  for (UInt i = 1; i < AStarPath.points.size(); ++i)
  {
    if (isConnectionLineFree(startPointCurrent, AStarPath.points[i]))
      continue;

    pathSegments.push_back(LineSegment2D(startPointCurrent, AStarPath.points[i - 1]));
    startPointCurrent = AStarPath.points[i - 1];
    --i;
  }
  pathSegments.push_back(LineSegment2D(startPointCurrent, AStarPath.points.back()));

  if (pathSegments.size() > 1)
  {
    //shorten path segments to lenghts which are kept
    std::vector<Tuple2D> intersectionPoints;
    intersectionPoints.push_back(pathSegments[0].end);
    pathSegments[0].clipEnd(AStarPathSmoothingDistance);
    for (UInt i = 1; i < pathSegments.size() - 1; ++i)
    {
      intersectionPoints.push_back(pathSegments[i].end);
      pathSegments[i].clipBoth(AStarPathSmoothingDistance);
    }
    pathSegments.back().clipStart(AStarPathSmoothingDistance);

    //find smooth parametric cubic function for each intermediate waypoint
    for (UInt i = 0; i < pathSegments.size() - 1; ++i)
      connections.push_back(ParametricFunctionCubic2D(pathSegments[i].end, pathSegments[i + 1].start, intersectionPoints[i], AStarPathSmoothingFactor, 8));
  }

  //find full path length with smoothed corners
  Real pathLengthFull = 0.0;
  for (UInt i = 0; i < connections.size(); ++i)
    pathLengthFull += connections[i].length;
  for (UInt i = 0; i < pathSegments.size(); ++i)
    pathLengthFull += pathSegments[i].length;

  const Real pointDistance = pathLengthFull / round(pathLengthFull / AStarPathSmoothedPointDistance);

  std::vector<Real> poseTmp(8);
  copyArmToRobotPose(posesFolding.back(), poseTmp);

  //generate points on full smoothed path
  Real distanceOnCurrent = 0.0, distanceTotal = 0.0;
  Tuple2D pointTmp;
  poseTmp[0] = pathSegments[0].start.x;
  poseTmp[1] = pathSegments[0].start.y;
  poseTmp[2] = pathSegments[0].angle;
  posesTrajectory.push_back(poseTmp);
  for (UInt i = 0; i < pathSegments.size() - 1; ++i)
  {
    poseTmp[2] = pathSegments[i].angle;
    while (true)
    {
      distanceOnCurrent += pointDistance;
      if (distanceOnCurrent > pathSegments[i].length)
      {
        distanceOnCurrent = pathSegments[i].length - distanceOnCurrent;
        break;
      }
      pointTmp = pathSegments[i].getPointAbsolute(distanceOnCurrent);
      poseTmp[0] = pointTmp.x;
      poseTmp[1] = pointTmp.y;
      posesTrajectory.push_back(poseTmp);
    }
    while (true)
    {
      distanceOnCurrent += pointDistance;
      if (distanceOnCurrent > connections[i].length)
      {
        distanceOnCurrent = connections[i].length - distanceOnCurrent;
        break;
      }
      pointTmp = connections[i].getPointAbsolute(distanceOnCurrent);
      poseTmp[0] = pointTmp.x;
      poseTmp[1] = pointTmp.y;
      poseTmp[2] = connections[i].getAngleAbsolute(distanceOnCurrent);
      posesTrajectory.push_back(poseTmp);
    }
  }
  poseTmp[2] = pathSegments.back().angle;
  while (true)
  {
    distanceOnCurrent += pointDistance;
    if (distanceOnCurrent > pathSegments.back().length)
      break;
    pointTmp = pathSegments.back().getPointAbsolute(distanceOnCurrent);
    poseTmp[0] = pointTmp.x;
    poseTmp[1] = pointTmp.y;
    posesTrajectory.push_back(poseTmp);
  }
}

bool Planner::isConnectionLineFree(const Tuple2D &pointStart, const Tuple2D &pointEnd)
{
  Tuple2D direction = pointEnd - pointStart;
  Real distanceMax = sqrt(direction.x * direction.x + direction.y * direction.y);
  Real searchFactor = mapResolution * 0.25;
  direction.x = direction.x * searchFactor / distanceMax;
  direction.y = direction.y * searchFactor / distanceMax;

  Tuple2D pointCurrent = pointStart;
  Real distanceCurrent = 0;
  while (distanceCurrent < distanceMax)
  {
    distanceCurrent += searchFactor;
    pointCurrent += direction;
    if (isCellOccupied(getCellFromPoint(pointCurrent)))
      return false;
  }

  return true;
}

// ******************** INLINES ********************

inline void Planner::loadParameter(const string &name, Real &member, const Real &defaultValue)
{
  try
  {
    if (!nh.getParam(name, member))
    {
      ROS_ERROR("Can't find ROS parameter '%s'. Using the default value '%.4f' instead.", (nh.getNamespace() + "/" + name).c_str(), defaultValue);
      member = defaultValue;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("ROS parameter '%s' has an invalid format. Using the default value '%.4f' instead.", (nh.getNamespace() + "/" + name).c_str(), defaultValue);
    member = defaultValue;
  }
}

inline void Planner::loadParameter(const string &name, std::vector<Real> &member, const std::vector<Real> &defaultValue)
{
  try
  {
    if (!nh.getParam(name, member))
    {
      ROS_ERROR("Can't find ROS parameter '%s'. Using the default value instead.", (nh.getNamespace() + "/" + name).c_str());
      member = defaultValue;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("ROS parameter '%s' has an invalid format. Using the default value instead.", (nh.getNamespace() + "/" + name).c_str());
    member = defaultValue;
  }
}

inline bool Planner::isOctreeNodeOccupied(const octomap::OcTreeKey &key) const
{
  octomap::OcTreeNode* node = octree->search(key);

  if (node != NULL)
    return octree->isNodeOccupied(node);
  else
    return false;
}

inline Tuple2D Planner::getPointFromCell(const Cell2D &cell) const
{
  return Tuple2D(mapMinX + (cell.x + 0.5) * mapResolution, mapMinY + (cell.y + 0.5) * mapResolution);
}

inline Cell2D Planner::getCellFromPoint(const Tuple2D &point) const
{
  return Cell2D((UInt)((point.x - mapMinX) * mapResolutionRecip), (UInt)((point.y - mapMinY) * mapResolutionRecip));
}

inline Real Planner::distanceCells(const Cell2D &cell1, const Cell2D &cell2)
{
  return sqrt(((Real)cell1.x - (Real)cell2.x) * ((Real)cell1.x - (Real)cell2.x) + ((Real)cell1.y - (Real)cell2.y) * ((Real)cell1.y - (Real)cell2.y));
}

inline bool Planner::isCellOccupied(const Cell2D &cell) const
{
  return occupancyMap[cell.x][cell.y];
}

inline void Planner::updateFCost(AStarNode &node) const
{
  Real dx = abs((Real)node.cell.x - (Real)AStarCellGoal.x);
  Real dy = abs((Real)node.cell.y - (Real)AStarCellGoal.y);
  node.f = node.g + std::min(dx, dy) * M_SQRT2 + abs(dx - dy);
}

inline void Planner::copyArmToRobotPose(const std::vector<Real> &poseArm, std::vector<Real> &poseRobot)
{
  poseRobot[3] = poseArm[0];
  poseRobot[4] = poseArm[1];
  poseRobot[5] = poseArm[2];
  poseRobot[6] = poseArm[3];
  poseRobot[7] = poseArm[4];
}

} //namespace SquirrelMotionPlanner
