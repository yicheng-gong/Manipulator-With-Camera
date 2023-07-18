/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

///////////////////////////////////////////////////////////////////////////////
SceneObject::SceneObject()
{
  orientation.w = 1;
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;
}
///////////////////////////////////////////////////////////////////////////////
Basket::Basket(std::string object_name, std::string object_color,
  geometry_msgs::Point location)
{
  collision_size_.x = 0.1;
  collision_size_.y = 0.1;
  collision_size_.z = 0.1;
  this->object_name = object_name;
  this->object_color = object_color;
  this->location = location;
}
///////////////////////////////////////////////////////////////////////////////
Cube::Cube(std::string object_name, std::string object_color,
  geometry_msgs::Point location)
{
  collision_size_.x = 0.04;
  collision_size_.y = 0.04;
  collision_size_.z = 0.04;
  this->object_name = object_name;
  this->object_color = object_color;
  this->location = location;
}

pcl::visualization::CloudViewer viewer ("Cluster viewer");
///////////////////////////////////////////////////////////////////////////////
CW1::CW1(ros::NodeHandle nh):
  g_cloud_ptr_ (new PointC), // input point cloud
  g_cloud_filtered_ (new PointC) // filtered point cloud
{
  /* class constructor */
  Home();
  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &CW1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &CW1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &CW1::t3_callback, this);

  AddGroundCollision();

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////
void
CW1::AddGroundCollision()
{
  // add ground collision to make sure it will not hit ground
  geometry_msgs::Point point;
  geometry_msgs::Vector3 shape;
  geometry_msgs::Quaternion orientation;
  point.x = 0.3;
  point.y = 0;
  point.z = 0;
  shape.x = 1.2;
  shape.y = 1.2;
  shape.z = 0.02;
  orientation.w = 1;
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;
  AddCollisionObject("ground", point ,shape, orientation);
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  
  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  bool success = T1Execution(request,response);
  if(!success)
  {
    ROS_ERROR("TASK 1 EXECUTION FAIL!");
    return false;
  }
  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  bool success = T2Execution(request,response);
  if(!success)
  {
    ROS_ERROR("TASK 2 EXECUTION FAIL!");
    return false;
  }
  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  bool success = T3Execution(request,response);
  if(!success)
  {
    ROS_ERROR("TASK 3 EXECUTION FAIL!");
    return false;
  }
  return success;

}

///////////////////////////////////////////////////////////////////////////////
bool
CW1::T1Execution(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response)
{ 
  // step 1: add data to our vector 
  cube_vector_.clear();
  basket_vector_.clear();
  cube_vector_.push_back(Cube("cube_0", "random", request.object_loc.pose.position));
  basket_vector_.push_back(Basket("basket_0", "random", request.goal_loc.point));
  // step 2:add collisions
  if(!AddCollisionsInScene(cube_vector_, basket_vector_))
  {
    ROS_ERROR("Task 1 AddCollisionsInScene fail");
    return false;
  }
    
  // step 3: perform pick & place 
  if(!PickAndPlace(cube_vector_[0], basket_vector_[0]))
  {
    ROS_ERROR("Task 1 PickAndPlace fail");
    return false;
  }

  // step 4: remove collisions
  if (!RemoveCollisionObject(basket_vector_[0].object_name))
  {
    ROS_ERROR("Task 1 RemoveCollisionObject fail");
    return false;
  }
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool 
CW1::T2Execution(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  // step 1: add data to our vector
  basket_vector_.clear();
  if(!ScanObjects(request.basket_locs))
  {
    ROS_ERROR("Task 2 ScanObjects fail");
    return false;
  }
  
  // step 2: create response
  for(Basket eachbasket : basket_vector_)
  {
    response.basket_colours.push_back(eachbasket.object_color);
    ROS_INFO_STREAM(std::fixed<< eachbasket.object_color);
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool
CW1::T3Execution(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  cube_vector_.clear();
  basket_vector_.clear();
  // run to best pose that scan whole scene
  if(!ScanPose())
  {
    ROS_ERROR("Task 3 ScanPose fail");
    return false;
  }
  // find cube and basket
  ros::Duration(1.0).sleep();
  if(!ColorClassify(g_cloud_filtered_))
  {
    ROS_ERROR("Task 3 ColorClassify fail");
    return false;
  }
  if(!StoreCubeAndBasket(object_map_))
  {
    ROS_ERROR("Task 3 StoreCubeAndBasket fail");
    return false;
  }
  
  ROS_INFO_STREAM(std::fixed<<"basket vector:" <<basket_vector_.size() << "cube vector"<< cube_vector_.size()); 
  Home();
  // add collision 
  if(!AddCollisionsInScene(cube_vector_, basket_vector_))
  {
    ROS_ERROR("Task 3 AddCollisionsInScene fail");
    return false;
  }

  // step 3: perform pick & place 
  for (std::size_t i = 0; i < cube_vector_.size(); i++)
  {
    for (std::size_t j = 0; j < basket_vector_.size(); j++)
    {
      if(cube_vector_[i].object_color == basket_vector_[j].object_color)
      {
        if(!PickAndPlace(cube_vector_[i], basket_vector_[j]))
        {
          ROS_ERROR("Task 3 PickAndPlace fail");
          return false;
        }
        break;
      }
    }
  }
  
  // step 4: remove collisions
  for (size_t i = 0; i < basket_vector_.size(); i++)
  {
    if (!RemoveCollisionObject(basket_vector_[i].object_name))
    {
      ROS_ERROR("Task 3 RemoveCollisionObject fail");
      return false;
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool 
CW1::Home()
{
  std::vector<double> home_joint_state = {0.004299755509283187, -0.41308541267892807,
  -0.004084591509041502, -2.586860740447717, -0.0021135966422249908, 2.170718510702187, 
  0.7862666539632421};
  arm_group_.setJointValueTarget(home_joint_state);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "SUCCESS" : "FAILED");

  // execute the planned path
  arm_group_.move();
  return success;
}

////////////////////////////////////////////////////////////////////////////////
bool 
CW1::MoveArm(geometry_msgs::Pose target_pose)
{
  // set orientation of EE
  target_pose.orientation.w = 0;
  target_pose.orientation.x = 0.9238795;
  target_pose.orientation.y = -0.3826834;
  target_pose.orientation.z = 0;
  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "SUCCESS" : "FAILED");

  // execute the planned path
  arm_group_.execute(my_plan);
  

  return success;
}

////////////////////////////////////////////////////////////////////////////////
bool 
CW1::MoveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double each_joint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripper_joint_targets(2);
  gripper_joint_targets[0] = each_joint;
  gripper_joint_targets[1] = each_joint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripper_joint_targets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "SUCCESS" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////////////////
bool 
CW1::Pick(Cube cube_object)
{
  geometry_msgs::Pose target_pose;


  if(!MoveGripper(gripper_open_))
    return false;

  
  // offset from joint to EE
  target_pose.position = cube_object.location;
  target_pose.position.z = pick_offset_z_;
  
  // move to setting place
  if(!MoveArm(target_pose))
    return false;
  RemoveCollisionObject(cube_object.object_name);
  if(!MoveGripper(gripper_closed_))
    return false;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool 
CW1::Place(Basket basket_object)
{
  geometry_msgs::Pose target_pose;
  target_pose.position = basket_object.location;
  target_pose.position.z = place_offset_z_;
  if(!MoveArm(target_pose))
    return false;
  if(!MoveGripper(gripper_open_))
    return false;
  return true;
}
////////////////////////////////////////////////////////////////////////////////
bool 
CW1::AddCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // Hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  bool success = planning_scene_interface_.applyCollisionObjects(object_vector);

  return success;
}

////////////////////////////////////////////////////////////////////////////////
bool 
CW1::RemoveCollisionObject(std::string object_name)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;
  object_vector.push_back(collision_object);
  
  bool success = planning_scene_interface_.applyCollisionObjects(object_vector);
  return success;
}

////////////////////////////////////////////////////////////////////////////////
bool
CW1::PickAndPlace(Cube object_cube, Basket object_basket)
{
  if(!Home())
      ROS_ERROR("Home Failed");

  if(!Pick(object_cube))
    return false;

  if(!Home())
      ROS_ERROR("Home Failed");

  if (!Place(object_basket))
    return false;

  if(!Home())
      ROS_ERROR("Home Failed");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool
CW1::AddCollisionsInScene(std::vector<Cube> cube_vector_,
  std::vector<Basket> basket_vector_)
{
  for (std::size_t i = 0; i < cube_vector_.size(); i++)
  {
    if (!AddCollisionObject(cube_vector_[i].object_name, cube_vector_[i].location,
      cube_vector_[i].collision_size_, cube_vector_[i].orientation))
      return false;
  }
  
  // add basket collision
  for (std::size_t i = 0; i < basket_vector_.size(); i++)
  {
    if (!AddCollisionObject(basket_vector_[i].object_name, basket_vector_[i].location, 
      basket_vector_[i].collision_size_, basket_vector_[i].orientation))
      return false;
  }
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool
CW1::ScanObjects(std::vector<geometry_msgs::PointStamped> &baskect_loc)
{
  geometry_msgs::Pose target_pose;
  
  ROS_INFO_STREAM(std::fixed<<"Start scanning "<<baskect_loc.size()<< "points");
  //if (!pathPlanning(baskect_loc)) return false;
  for (std::size_t i = 0; i < baskect_loc.size(); i++)
  {
    if(!Home()) 
      ROS_ERROR("Home Failed");
    target_pose.position = baskect_loc[i].point;
    target_pose.position.z = sensor_zoffset_;
    if(!MoveArm(target_pose))
      return false;
    ROS_INFO_STREAM(std::fixed << "\033[44;37m" << "Reached the "<<i<< " point" << "\033[0m");
    // write here the pcl function that can justify the color
    
    ros::Duration(1.0).sleep();
    // viewer.showCloud(g_cloud_filtered_);
    std::string object_color = SensorJugColor(g_cloud_filtered_);
    // push result to our vector
    basket_vector_.push_back(Basket("basket_" + std::to_string(i), 
      object_color, baskect_loc[i].point));
  }
  if(!Home()) 
    ROS_ERROR("Home Failed");
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool
CW1::ApplyVX (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  pcl::VoxelGrid<PointT> g_vx;

  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz_, g_vg_leaf_sz_, g_vg_leaf_sz_);
  g_vx.filter (*out_cloud_ptr);
  
  return true;
}


////////////////////////////////////////////////////////////////////////////////
void
CW1::SensorMsgsCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr_);

  // Perform the filtering
  ApplyVX (g_cloud_ptr_, g_cloud_filtered_); 
  
  // Segment plane and cylinder
  //findNormals (g_cloud_filtered);
  //segPlane (g_cloud_filtered);
  //segCylind (g_cloud_filtered);
  //findCylPose (g_cloud_cylinder);
  viewer.showCloud(g_cloud_filtered_);

  return;
}

////////////////////////////////////////////////////////////////////////////////
bool
CW1::IsNotInRange(PointT sensor_points)
{
  return sensor_points.x < -0.02 ||  sensor_points.x > 0.02 ||
         sensor_points.y < 0     ||  sensor_points.y > 0.04;
}



std::string
CW1::JugColor(PointT sensor_points)
{
  if (double(sensor_points.r)> double(sensor_points.g)*2 
    && double(sensor_points.r) > double(sensor_points.b)*2) 
    return "red"; 
  if (double(sensor_points.b)> double(sensor_points.r)*2 
    && double(sensor_points.b) > double(sensor_points.g)*2) 
    return "blue"; 
  if (double(sensor_points.b)> double(sensor_points.g)*2 
    && double(sensor_points.r) > double(sensor_points.g)*2) 
    return "purple"; 
  return "none";
}

///////////////////////////////////////////////////////////////////////////////
std::string
CW1::SensorJugColor(PointCPtr sensor_cloud)
{
  // viewer.showCloud(sensor_cloud);
  float redCount=0;
  float blueCount=0;
  float purpleCount=0;
  float noneCount=0;
  std::string colorName;
  for (const auto &sensor_points : sensor_cloud->points)
  {
    if(IsNotInRange(sensor_points) == false)
    {
      colorName = JugColor(sensor_points);
      if(colorName == "purple") purpleCount++;
      else if(colorName == "blue") blueCount++;
      else if(colorName == "red") redCount++;
      else if(colorName == "none") noneCount++;
    }
  }
  float redProb = redCount/(redCount+blueCount+purpleCount+noneCount);
  float blueProb = blueCount/(redCount+blueCount+purpleCount+noneCount);
  float purpleProb = purpleCount/(redCount+blueCount+purpleCount+noneCount);
  float noneProb = noneCount/(redCount+blueCount+purpleCount+noneCount);
  ROS_INFO("Red Probability: %f", redProb);
  ROS_INFO("BLue Probability: %f", blueProb);
  ROS_INFO("Purple Probability: %f", purpleProb);
  ROS_INFO("None Probability: %f", noneProb);
  float maxProb = std::max(noneProb, std::max(purpleProb, std::max(redProb, blueProb)));
  std::string object_color;
  if(redProb==maxProb) object_color = "red";
  else if(blueProb==maxProb) object_color = "blue";
  else if(purpleProb==maxProb) object_color = "purple";
  else object_color = "none";
  ROS_INFO_STREAM(std::fixed << "\033[44;37m" << "Color named " << object_color << " has the maximum probability "<< "\033[0m");
  return object_color;
}


///////////////////////////////////////////////////////////////////////////////
bool
CW1::ScanPose()
{
  geometry_msgs::Pose scanning_pose;
  scanning_pose.position.x = 0.4;
  scanning_pose.position.y = 0;
  scanning_pose.position.z = 0.87;
  if(!MoveArm(scanning_pose)) 
    return false;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool
CW1::ColorClassify(PointCPtr sensor_cloud)
{
  PointC red_object;
  PointC blue_object;
  PointC purple_object;
  std::string colorName;
  
  for (const auto &sensor_points : sensor_cloud->points)
  {
    colorName = JugColor(sensor_points);
    if(colorName == "red") 
      red_object.push_back(sensor_points);
    else if(colorName == "blue") 
      blue_object.push_back(sensor_points);
    else if(colorName == "purple") 
      purple_object.push_back(sensor_points);
  }
  object_map_ = {
  { "red", red_object },
  { "blue", blue_object },
  { "purple", purple_object } };
  return true;
}


///////////////////////////////////////////////////////////////////////////////
bool 
CW1::StoreCubeAndBasket(std::map<std::string,PointC> object_map_)
{
  geometry_msgs::Point cube_center;
  geometry_msgs::Point basket_center;
  int basket_idx = 0;
  int cube_idx = 0;

  // cluster
  
  for (auto it=object_map_.begin(); it!=object_map_.end(); ++it)
  { 
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setInputCloud(it->second.makeShared());
    ec.setClusterTolerance(0.02f); // set cluster tolerance
    ec.setMinClusterSize(20); // set minimum cluster size
    ec.setMaxClusterSize(5000000); // set maximum cluster size
    ec.extract(cluster_indices);
    ROS_INFO_STREAM(std::fixed<<it->first+" num of cluster "<<cluster_indices.size()<<" end");
    
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
      std::vector<float> x_range;
      std::vector<float> y_range;
      for (auto point_idx : cluster_indices[i].indices)
      {
        x_range.push_back(it->second.points[point_idx].x);
        y_range.push_back(it->second.points[point_idx].y);
      }

      geometry_msgs::Point obj_center;
      double x = (*std::max_element(x_range.begin(), x_range.end()) + 
        *std::min_element(x_range.begin(), x_range.end())) / 2;
      double y = (*std::max_element(y_range.begin(), y_range.end()) + 
        *std::min_element(y_range.begin(), y_range.end())) / 2;
      // tranformation 
      obj_center.z = 0.02;
      obj_center.x = -(y - 0.4) + 0.04;
      obj_center.y = -x; 
      // juudge box and basket
      if(*std::max_element(x_range.begin(), x_range.end()) - 
        *std::min_element(x_range.begin(), x_range.end()) > 0.05)
      {
        basket_vector_.push_back(Basket("basket_"+std::to_string(basket_idx), it->first, obj_center));
        basket_idx++;
      }
      else
      {
        cube_vector_.push_back(Cube("cube_" + std::to_string(cube_idx), it->first, obj_center));
        cube_idx++;
      }
    }
  
  
  }
  return true;
}
