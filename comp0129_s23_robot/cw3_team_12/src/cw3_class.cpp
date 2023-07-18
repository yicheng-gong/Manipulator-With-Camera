/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

#include <cw3_class.h> // change to your team name here!

///////////////////////////////////////////////////////////////////////////////
SceneObject::SceneObject(geometry_msgs::Point location, double x_len, double y_len, PointCPtr obj_pc)
{
  // base class to be inherient
  location_ = location;
  x_len = x_len;
  y_len = y_len; 
  object_pc = obj_pc;
}

///////////////////////////////////////////////////////////////////////////////

Basket::Basket(geometry_msgs::Point location)
{
  // constructor of basket 
  collision_size_.x = 0.35;
  collision_size_.y = 0.35;
  collision_size_.z = 0.05;
  orientation_.x = 0;
  orientation_.y = 0;
  orientation_.z = 0;
  orientation_.w = 1;
  object_color_ = "brown";
  object_type_ = "basket";
  this->location_ = location;
}

///////////////////////////////////////////////////////////////////////////////

Object::Object(std::string object_type, double obj_size ,std::string object_color,
  geometry_msgs::Point location, geometry_msgs::Quaternion orientation)
{
  // construcr cross or nought
  this->object_type_ = object_type;
  this->object_size_ = obj_size;
  this->object_color_ = object_color;
  this->location_ = location;
  this->orientation_ = orientation;
}

///////////////////////////////////////////////////////////////////////////////

CW3::CW3(ros::NodeHandle nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_plane (new PointC), // plane point cloud
  g_tree_ptr (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  debug_ (false)
{
  /* class constructor */
  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &CW3::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &CW3::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &CW3::t3_callback, this);

  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file
  scanning_ori.w = 0;
  scanning_ori.x = 0.9238795;
  scanning_ori.y = -0.3826834;
  scanning_ori.z = 0;
  Home();
  
  // add ground collision
  AddGroundCollision();
  ROS_INFO("CW3 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::t1_callback(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response) 
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
CW3::t2_callback(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  bool success = T2Execution(request,response);
  if(!success)
  {
    ROS_ERROR("TASK 2 EXECUTION FAIL!");
    return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::t3_callback(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  bool success = T3Execution(request,response);
  if(!success)
  {
    ROS_ERROR("TASK 3 EXECUTION FAIL!");
    return false;
  }
  return true;
}


////////////////////////////////////////////////////////////////////////////////

bool 
CW3::Home()
{
  std::vector<double> home_joint_state = {0, -0.261799,
  0, -2.0944, 0, 1.8326, 0.785398};
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
std::vector<pcl::PointIndices> 
CW3::findCluster(PointCPtr &in_cloud_ptr, int point_num)
{
  // process object 
  *in_cloud_ptr = getObjcluster(in_cloud_ptr);
  *in_cloud_ptr = getObjcluster(in_cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setInputCloud(in_cloud_ptr);
  ec.setClusterTolerance(0.02f); // set cluster tolerance
  ec.setMinClusterSize(point_num); // set minimum cluster size
  ec.setMaxClusterSize(5000000); // set maximum cluster size
  ec.extract(cluster_indices);
  
  return cluster_indices;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::T1Execution(cw3_world_spawner::Task1Service::Request &request,
  cw3_world_spawner::Task1Service::Response &response)
{ 
  if(!Home())
    ROS_ERROR("Home Failed");
  
  object_vector_.clear();
  basket_vector_.clear();

  // step 1: store the info of object and basket
  Object to_pick = goConfigObject(request.object_point.point);
  to_pick.collision_center_ = request.object_point.point;
  Basket to_place = Basket(request.goal_point.point);

  object_vector_.push_back(to_pick);
  basket_vector_.push_back(to_place);

  //step 2: add collisions
  if(!AddCollisionsInScene(object_vector_, basket_vector_))
  {
    ROS_ERROR("Task 1 AddCollisionsInScene fail");
    return false;
  }
  
  // step 3: grasp and place the object
  PickAndPlace(object_vector_[0], basket_vector_[0]);
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::T2Execution(cw3_world_spawner::Task2Service::Request &request,
  cw3_world_spawner::Task2Service::Response &response)
{ 
  // clear object vector
  object_vector_.clear();

  // step 1:  config the reference object
  for (auto &&each_object : request.ref_object_points)
  {
    object_vector_.push_back(goConfigObject(each_object.point));
  }
  // step 2: config mystery object
  Object mystery_object = goConfigObject(request.mystery_object_point.point);
  
  // step 3: loop to check if they same 
  for (int i = 0; i < object_vector_.size(); i++)
  {
    if (mystery_object.object_type_ == object_vector_[i].object_type_)
    {
      response.mystery_object_num = i+1;
      ROS_INFO_STREAM(std::fixed << "\033[44;37m" << "matches obj num is: " << i+1 << "\033[0m");
      break;
    }
  }

  // home after the operation
  if(!Home())
    ROS_ERROR("Home Failed");
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::T3Execution(cw3_world_spawner::Task3Service::Request &request,
  cw3_world_spawner::Task3Service::Response &response)
{ 
  // home first 
  if(!Home())
    ROS_ERROR("Home Failed");
  // clear global variable to use them
  basket_vector_.clear();
  object_vector_.clear();
  obstacle_vector_.clear();
  (*g_cloud_plane).clear();

  // step 1: move arm to scan 6 poses 
  for (int i = 0; i < 6; i++)
  {
    if(!ScanPose(i))
    {
      ROS_ERROR("Task 3 ScanPose fail");
      return false;
    }
  }
  
  // when calculating the objects, start a thread to home, save time
  std::thread td_home(&CW3::Home,this);

  // step 2: get rid of obstecles, filter out noises, start process data and tore them in vector
  PointCPtr filtered_cloud_point(new PointC);
  PointCPtr obstcle_cloud_point(new PointC);
  for (auto &&pt : *g_cloud_plane)
  { 
    if (pt.x>-0.25 && pt.x<0.25 && pt.y>-0.25 && pt.y<0.25)
      continue;
    
    if (pt.r <=50&& pt.g<=50&& pt.b<=50)
    {
      (*obstcle_cloud_point).push_back(pt);
    }
    else
    {
      (*filtered_cloud_point).push_back(pt);
    }
  }

  // find cluster of object and basket first 
  std::vector<pcl::PointIndices> cluster_indices = findCluster(filtered_cloud_point, 4000);
  
  // then store cluster information in vector 
  std::vector<SceneObject> undefined_obj_vector = preProcessCluster(filtered_cloud_point, cluster_indices);
  
  // store basket and object, and calculate crossnum and nought num 
  StoreBasketAndObject(undefined_obj_vector);

  // step 3: add collisions
  if(!AddCollisionsInScene(object_vector_, basket_vector_))
  {
    ROS_ERROR("Task 3 AddCollisionsInScene fail");
    return false;
  }
  
  // for main thread and homw thread to meet
  td_home.join();

  // step 4: pick moat commen one and return the response value
  for (auto &&obj : object_vector_)
  {
    if (cross_num >nought_num)
    {
      response.num_most_common_shape = cross_num;
      if (obj.object_type_ == cross_type_name_)
      {
        PickAndPlace(obj, basket_vector_[0]);
        break;
      }
    }
    else
    {
      response.num_most_common_shape = nought_num;
      if (obj.object_type_ == nought_type_name_)
      {
        PickAndPlace(obj, basket_vector_[0]);
        break;
      }
    }
  }

  // remove all objects collisions
  for (int i = 0; i < object_vector_.size(); i++)
  {
    RemoveCollisionObject(object_vector_[i].object_id_);
  }
  response.total_num_shapes = cross_num + nought_num;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
void
CW3::StoreBasketAndObject(std::vector<SceneObject> undefined_obj_vector)
{
  // find basket and erase it from vector 
  double diag_len = 0;
  int max_idx = 0;
  for (int i = 0; i < undefined_obj_vector.size(); i++)
  {
    if ((pow(undefined_obj_vector[i].x_len, 2) + pow(undefined_obj_vector[i].y_len, 2)) >diag_len)
    {
      max_idx = i;
    }
  }
  undefined_obj_vector[max_idx].location_.z = undefined_obj_vector[max_idx].location_.z - 0.05;
  basket_vector_.push_back(Basket(undefined_obj_vector[max_idx].location_));
  undefined_obj_vector.erase(undefined_obj_vector.begin() + max_idx);
  
  // config Object and push them to object vector
  cross_num = 0;
  nought_num = 0;
  for (int i = 0; i < undefined_obj_vector.size(); i++)
  {
    geometry_msgs::Pose p;
    p.position = undefined_obj_vector[i].location_;
    p.position.z = scanning_offset_z;
    p.orientation = scanning_ori;

    Object configObj = generateObject (undefined_obj_vector[i].object_pc, p, false);
    configObj.collision_center_ = undefined_obj_vector[i].location_;
    if(configObj.object_type_ == nought_type_name_)
    {
      nought_num++;
    }
    else
    {
      cross_num++;
    }
    object_vector_.push_back(configObj);
  }
}

///////////////////////////////////////////////////////////////////////////////
bool
CW3::ScanPose(int num)
{
  PointCPtr buff_ptr(new PointC);
  Eigen::Matrix4f transform_ ;
  geometry_msgs::Pose scanning_pose;
  scanning_pose.orientation = scanning_ori;
  scanning_pose.position.z = 0.65;
  double x_offset = 0;
  double y_offset = 0;
  switch (num)
  {
  case 2:
    scanning_pose.position.x = 0.4;
    scanning_pose.position.y = -0.3;
    x_offset = 0.3;
    break;
  case 3:
    scanning_pose.position.x = 0.4;
    scanning_pose.position.y = 0.3;
    x_offset = -0.3;
    break;
  case 4:
    scanning_pose.position.x = 0;
    scanning_pose.position.y = 0.4;
    x_offset = -0.4;
    y_offset = 0.4 ;
    break;
  case 5:
    scanning_pose.position.x = -0.4;
    scanning_pose.position.y = 0.3;
    x_offset = -0.3;
    y_offset = 0.8;
    break;
  case 0:
    scanning_pose.position.x = -0.4;
    scanning_pose.position.y = -0.3;
    x_offset = 0.3;
    y_offset = 0.8;
    break;
  case 1:
    scanning_pose.position.x = -0;
    scanning_pose.position.y = -0.4;
    x_offset = 0.4;
    y_offset = 0.4;
    break;
  default:
    break;
  }
  // move to preddefined pose 
  if(!MoveArm(scanning_pose)) 
    return false;

  transform_= Eigen::Matrix4f::Identity();
  y_offset-=0.4;
  transform_(0,3) = x_offset;
  transform_(1,3) = y_offset;
  pcl::transformPointCloud(*g_cloud_ptr, *buff_ptr, transform_);
  *g_cloud_plane += *buff_ptr;

  if(!MoveBackToSafe())
    return false;
  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
CW3::AddGroundCollision()
{
  // add ground collision to make sure it will not hit ground
  geometry_msgs::Point point;
  geometry_msgs::Vector3 shape;
  geometry_msgs::Quaternion orientation;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  shape.x = 2;
  shape.y = 2;
  shape.z = 0.02;
  orientation.w = 1;
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;
  AddCollisionObject("ground", point ,shape, orientation);
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::AddCollisionsInScene(std::vector<Object> &object_vector_,
  std::vector<Basket> &basket_vector_)
{
  for (std::size_t i = 0; i < object_vector_.size(); i++)
  {
    object_vector_[i].object_id_ = object_vector_[i].object_type_ + std::to_string(i);
    if (!AddCollision(object_vector_[i].object_type_, object_vector_[i].object_id_,
      object_vector_[i].collision_center_, object_vector_[i].object_size_, 
      object_vector_[i].orientation_))
      return false;
  }
  
  
  // add basket collision
  for (std::size_t i = 0; i < basket_vector_.size(); i++)
  {
    if (!AddCollisionObject(basket_vector_[i].object_type_, basket_vector_[i].location_, 
      basket_vector_[i].collision_size_, basket_vector_[i].orientation_))
      return false;
  }
  
  return true;
}


////////////////////////////////////////////////////////////////////////////////

void
CW3::PointCloudCallBack
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);
  
  return;
}

///////////////////////////////////////////////////////////////////////////////

bool
CW3::ApplyVX (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  pcl::VoxelGrid<PointT> g_vx;

  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void
CW3::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree_ptr);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
CW3::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  g_seg.setNormalDistanceWeight (0.1); //bad style
  g_seg.setMethodType (pcl::SAC_RANSAC);
  g_seg.setMaxIterations (100); //bad style
  g_seg.setDistanceThreshold (0.05); //bad style
  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);
  
  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);
  
}

////////////////////////////////////////////////////////////////////////////////

Object
CW3::generateObject (PointCPtr &in_cloud_ptr, geometry_msgs::Pose ori_pose, bool with_ground)
{

  PointCPtr filtered_cloud_ptr(new PointC);
  std::string obj_type_;
  double rotation_angle;
  double obj_size_;
  geometry_msgs::Pose target_pose;

  // filter the ground point
  if (with_ground)
  {
    *filtered_cloud_ptr = removeGroundPoint(in_cloud_ptr);
  }else
  {
    filtered_cloud_ptr = in_cloud_ptr;
  }

  // find the range of point cloud
  findPCRange(*filtered_cloud_ptr);

  // judge the object type
  obj_type_ = jugObjType(*filtered_cloud_ptr);

  // judge the object size
  obj_size_ = jugObjSize(obj_type_, *filtered_cloud_ptr);

  // compute rotation angle
  rotation_angle = computeRotationAngle(obj_type_, obj_size_, *filtered_cloud_ptr);

  // compute target pose
  target_pose = computeGraspPose(obj_type_, obj_size_, ori_pose, rotation_angle);

  // store value to return 
  
  return Object(obj_type_, obj_size_, "none", target_pose.position, target_pose.orientation);
}

////////////////////////////////////////////////////////////////////////////////

Object
CW3::goConfigObject(geometry_msgs::Point center_location)
{
  geometry_msgs::Pose target_pose;
  target_pose.orientation = scanning_ori;
  target_pose.position = center_location;
  target_pose.position.z = scanning_offset_z;
  target_pose.position.x += camera_offset_x;
  // go to postion
  MoveArm(target_pose);
  
  ros::Duration(1.0).sleep();
  

  // back to picking position
  target_pose.position.x -= camera_offset_x;

  // MoveBackToSafe();
  return generateObject(g_cloud_ptr, target_pose);
}


////////////////////////////////////////////////////////////////////////////////

void
CW3::findPCRange(PointC &in_cloud_points)
{
  // find the point cloud range and center
  x_pc_range.clear();
  y_pc_range.clear();
  z_pc_range.clear();
  double sum_x = 0, sum_y = 0, sum_z = 0;
  for(const auto &point : in_cloud_points)
  {
    sum_x += point.x;
    x_pc_range.push_back(point.x);
    sum_y += point.y;
    y_pc_range.push_back(point.y);
    sum_z += point.z;
    z_pc_range.push_back(point.z);
  }
  center_point.x = sum_x / x_pc_range.size();
  center_point.y = sum_y / y_pc_range.size();
  center_point.z = sum_z / z_pc_range.size();
  return;
}

////////////////////////////////////////////////////////////////////////////////

PointC
CW3::removeGroundPoint(PointCPtr &in_cloud_ptr)
{
  PointC filtered_cloud_point;
  std::vector<float> z_ptr_range;
  for(const auto &point : in_cloud_ptr->points)
  {
    z_ptr_range.push_back(point.z);
  }

  // filter the ground point
  double z_pc_min = *std::min_element(z_ptr_range.begin(), z_ptr_range.end());
  for(const auto &point : in_cloud_ptr->points)
  {
    if(point.z - z_pc_min < 0.003 )
    { 
      filtered_cloud_point.push_back(point);
    }
  }

  return filtered_cloud_point;
}

////////////////////////////////////////////////////////////////////////////////

PointC
CW3::getObjcluster(PointCPtr &in_cloud_ptr)
{
  // find z range of these points
  PointC filtered_cloud_point;
  std::vector<float> z_ptr_range;
  for(const auto &point : in_cloud_ptr->points)
  {
    z_ptr_range.push_back(point.z);
  }

  // filter the ground point
  double z_pc_min = *std::max_element(z_ptr_range.begin(), z_ptr_range.end());
  for(const auto &point : in_cloud_ptr->points)
  {
    if(z_pc_min- point.z  > 0.008 )
    { 
      filtered_cloud_point.push_back(point);
    }
    
  }

  return filtered_cloud_point;
}


////////////////////////////////////////////////////////////////////////////////

std::string
CW3::jugObjType(PointC &in_cloud_points)
{
  // loop judge the object type
  std::string obj_type_ = nought_type_name_;
  for(const auto &point : in_cloud_points)
  {
    if(std::abs(point.x - 0)<0.01 && std::abs(point.y - 0)<0.01)
    {
      obj_type_ = cross_type_name_;
      return obj_type_;
    }
  }
  return obj_type_;
}

////////////////////////////////////////////////////////////////////////////////

double
CW3::jugObjSize(std::string obj_type, PointC &in_cloud_points)
{
  double obj_size_;
  std::vector<double> dist_;
  std::vector<double> thresholds;

  // setting thresholds
  if(obj_type == cross_type_name_)
  {
    thresholds.push_back(std::pow(0.05, 2) + std::pow(0.01, 2));
    thresholds.push_back(std::pow(0.075, 2) + std::pow(0.015, 2));
    thresholds.push_back(std::pow(0.1, 2) + std::pow(0.02, 2));
  }
  else if(obj_type == nought_type_name_)
  {
    thresholds.push_back(std::pow(0.05, 2) + std::pow(0.05, 2));
    thresholds.push_back(std::pow(0.075, 2) + std::pow(0.075, 2));
    thresholds.push_back(std::pow(0.1, 2) + std::pow(0.1, 2));
  }

  // find the point with the maximum distance
  for(const auto &point : in_cloud_points)
  {
    dist_.push_back(std::pow(point.x - center_point.x,2)+std::pow(point.y - center_point.y,2));
  }
  double dist_max_ = *std::max_element(dist_.begin(), dist_.end());

  // define the object size
  dist_.clear();
  for(const auto &threshold : thresholds)
  {
    dist_.push_back(std::abs(threshold - dist_max_));
  }
  auto min_iterator = std::min_element(dist_.begin(), dist_.end());
  int min_index = std::distance(dist_.begin(), min_iterator);
  if(min_index == 0)
    obj_size_ = 0.02;
  else if (min_index == 1)
    obj_size_ = 0.03;
  else if (min_index == 2)
    obj_size_ = 0.04;
  
  return obj_size_;
}

////////////////////////////////////////////////////////////////////////////////

double
CW3::computeRotationAngle(std::string obj_type, double obj_size, PointC &in_cloud_points)
{

  
  // find the maximum y
  std::vector<double> dist_;
  double y_max_ = *std::max_element(y_pc_range.begin(), y_pc_range.end());

  // find the corresponding x and distance
  std::vector<double> x_pc_max, x_pc_min;
  for(const auto &point : in_cloud_points)
  {
    if(point.y == y_max_)
    {
      x_pc_max.push_back(point.x);
      dist_.push_back(std::pow(point.x - center_point.x,2)+std::pow(point.y - center_point.y,2));
    }
  }

  // select the point with the largest distance
  auto max_iterator = std::max_element(dist_.begin(), dist_.end());
  int max_index = std::distance(dist_.begin(), max_iterator);
  double selected_point_x = x_pc_max[max_index];
  double selected_point_y = y_max_;

  // compute detected angle
  double detected_angle = std::acos(selected_point_x /
      std::sqrt(std::pow(selected_point_x,2)+std::pow(selected_point_y,2)));

  // compute rotation angle
  double rotation_angle;
  if(obj_type == nought_type_name_)
  {
    // compute nought rotation angle
    double init_angle = DEG2RAD(45);
    rotation_angle = detected_angle - init_angle;
    if (rotation_angle > DEG2RAD(45)) rotation_angle = rotation_angle - DEG2RAD(90);
  }
  else if(obj_type == cross_type_name_)
  {
    // judge rotation direction
    double distance;
    int left_count = 0, right_count = 0;
    for(const auto &point : in_cloud_points)
    {
      distance = std::pow(point.x - selected_point_x,2)+std::pow(point.y - selected_point_y,2);
      if (distance < std::pow(0.02,2))
      {
        if (point.x < selected_point_x) 
          left_count++;
        else if (point.x > selected_point_x)
          right_count++;
      }
    }

    // compute cross rotation angle
    if(left_count > right_count)
    {
      double init_angle = std::atan2(5,1);
      rotation_angle = detected_angle - init_angle;
    }
    else if(right_count > left_count)
    {
      double init_angle = std::atan2(5,-1);
      rotation_angle = detected_angle - init_angle;
    }
  }
  
  return rotation_angle;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<SceneObject>
CW3::preProcessCluster(PointCPtr &in_cloud_ptr, std::vector<pcl::PointIndices> cluster_indices)
{
  std::vector<SceneObject> undefined_obj_vector;
  // for each cluster find information and push them to vector
  for (size_t i = 0; i < cluster_indices.size(); i++)
  {
    PointCPtr obj_pc(new PointC);
    std::vector<float> x_range;
    std::vector<float> y_range;
    std::vector<float> z_range;

    for (auto point_idx : cluster_indices[i].indices)
    {
      (*obj_pc).push_back((*in_cloud_ptr)[point_idx]);
      x_range.push_back((*in_cloud_ptr)[point_idx].x);
      y_range.push_back((*in_cloud_ptr)[point_idx].y);
      z_range.push_back((*in_cloud_ptr)[point_idx].z);
    }
    
    geometry_msgs::Point center_point;
    double x_min = *std::min_element(x_range.begin(), x_range.end());
    double x_max = *std::max_element(x_range.begin(), x_range.end());
    double y_min = *std::min_element(y_range.begin(), y_range.end());
    double y_max = *std::max_element(y_range.begin(), y_range.end());
    double z_min = *std::min_element(z_range.begin(), z_range.end());
    double z_max = *std::max_element(z_range.begin(), z_range.end());
    double x = ( x_min+ x_max ) / 2;
    double y = (y_min + y_max) / 2;

    PointCPtr buff_ptr(new PointC);
    Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
    transform_(0,3) = -x;
    transform_(1,3) = -y;
    pcl::transformPointCloud(*obj_pc, *buff_ptr, transform_);

    center_point.x = -y -camera_offset_x;
    center_point.y = -x;
    center_point.z = 0.65 - z_max;
    undefined_obj_vector.push_back(SceneObject(center_point, abs(x_max-x_min), abs(y_max-y_min), buff_ptr));
  }
  return undefined_obj_vector;
}

////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
CW3::computeGraspPose(std::string obj_type, double obj_size, geometry_msgs::Pose ori_pose, double rotation_angle)
{
  // quaternion to Euler
  tf2::Quaternion q;
  tf2::fromMsg(ori_pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  geometry_msgs::Pose target_pose;
  m.getRPY(roll, pitch, yaw);
  
  // adjust orientation
  yaw = yaw - rotation_angle;
  q.setRPY(roll, pitch, yaw);
  target_pose.orientation = tf2::toMsg(q);

  // adjust position
  double move_distance;
  target_pose.position.z = ori_pose.position.z;
  if(obj_type == nought_type_name_)
  {
    move_distance = 2 * obj_size;
    target_pose.position.x = ori_pose.position.x - move_distance * std::sin(rotation_angle);
    target_pose.position.y = ori_pose.position.y - move_distance * std::cos(rotation_angle);
  }
  else if (obj_type == cross_type_name_)
  {
    move_distance = 1.5 * obj_size;
    target_pose.position.x = ori_pose.position.x + move_distance * std::cos(rotation_angle);
    target_pose.position.y = ori_pose.position.y - move_distance * std::sin(rotation_angle);
  }

  return target_pose;
}

////////////////////////////////////////////////////////////////////////////////

bool 
CW3::MoveBackToSafe()
{
  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setJointValueTarget(safe_joints);


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
CW3::MoveArm(geometry_msgs::Pose target_pose)
{
  // rotate first joint to face the target
  std::vector<double> joints_current;
  joints_current = arm_group_.getCurrentJointValues();

  double diff = atan2(target_pose.position.y, target_pose.position.x) - joints_current.at(0);
  joints_current.at(0) += diff;
  // store the pose which is safe for planning
  safe_joints = joints_current;
  arm_group_.setJointValueTarget(joints_current);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);
  arm_group_.move();

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "SUCCESS" : "FAILED");

  // execute the planned path
  arm_group_.execute(my_plan);
  
  return success;
}

////////////////////////////////////////////////////////////////////////////////

bool 
CW3::MoveGripper(float width)
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
CW3::Pick(Object target_object)
{
  geometry_msgs::Pose target_pose;

  if(!MoveGripper(gripper_open_))
    return false;

  // offset from joint to EE
  target_pose.position = target_object.location_;
  target_pose.position.z = pick_offset_z_;
  target_pose.orientation = target_object.orientation_;
  
  // move to setting place
  if(!MoveArm(target_pose))
    return false;

  RemoveCollisionObject(target_object.object_id_);

  if(!MoveGripper(gripper_closed_))
    return false;

  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool 
CW3::Place(Object target_object, Basket basket_object)
{
  geometry_msgs::Pose target_pose;
  target_pose.position = basket_object.location_;
  target_pose.position.x -= target_object.object_size_;
  if (target_object.object_type_ == nought_type_name_)
    target_pose.position.y += target_object.object_size_*2;
  target_pose.position.z = place_offset_z_;

  // quaternion to Euler
  tf2::Quaternion q;
  tf2::fromMsg(scanning_ori, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  // adjust orientation
  yaw = yaw + DEG2RAD(180);
  q.setRPY(roll, pitch, yaw);
  target_pose.orientation = tf2::toMsg(q);
  if(!MoveArm(target_pose))
    return false;
  if(!MoveGripper(gripper_open_))
    return false;
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool
CW3::PickAndPlace(Object target_object, Basket target_basket)
{
  if(!Pick(target_object))
    return false;
  
  if(!MoveBackToSafe())
    return false;

  if (!Place(target_object, target_basket))
    return false;

  RemoveCollisionObject("basket");

  if(!Home())
      ROS_ERROR("Home Failed");
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool 
CW3::AddCollisionObject(std::string object_type,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_type;
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
CW3::AddCollision(std::string object_type, std::string object_id,
  geometry_msgs::Point centre, double dimensions,
  geometry_msgs::Quaternion orientation)
{
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;

  
  collision_object.id = object_id;
  collision_object.header.frame_id = base_frame_;
  // input header information
  shapes::Mesh* m;
  shape_msgs::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  if (object_type == nought_type_name_)
  {
    if (abs(dimensions - 0.02)<0.003)
    {
      m = shapes::createMeshFromResource("package://cw3_team_12/models/nought_20mm_40H.STL"); // here defines the package
    } else if (abs(dimensions - 0.03)<0.003)
    {
      m = shapes::createMeshFromResource("package://cw3_team_12/models/nought_30mm_40H.STL");
    } else if (abs(dimensions - 0.04)<0.003)
    {
      m = shapes::createMeshFromResource("package://cw3_team_12/models/nought_40mm.STL");
    }
    
    shapes::constructMsgFromShape(m,object_mesh_msg);
    object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
  } else if (object_type == cross_type_name_)
  {
    if (abs(dimensions - 0.02)<0.003)
    {
      m = shapes::createMeshFromResource("package://cw3_team_12/models/cross_20mm_40H.STL");
    } else if (abs(dimensions - 0.03)<0.003)
    {
      m = shapes::createMeshFromResource("package://cw3_team_12/models/cross_30mm_40H.STL");
    } else if (abs(dimensions - 0.04)<0.003)
    {
      m = shapes::createMeshFromResource("package://cw3_team_12/models/cross_40mm.STL");
    }
    shapes::constructMsgFromShape(m,object_mesh_msg);
    object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
  }
  geometry_msgs::Pose object_pose;
	
  // change orientation to world 
  tf::Quaternion ori_buff(orientation.x, orientation.y, orientation.z, orientation.w);
  tf::Matrix3x3 mat(ori_buff);
  double r,p,y;
  mat.getRPY(r,p,y);
  y -= M_PI/4;
  r += M_PI/2;
  ori_buff.setRPY(r,p,y);
  ori_buff.normalize();
  object_pose.orientation.x = ori_buff.getX();
  object_pose.orientation.y = ori_buff.getY();
  object_pose.orientation.z = ori_buff.getZ();
  object_pose.orientation.w = ori_buff.getW();


	object_pose.position.x =  centre.x;
	object_pose.position.y =  centre.y;
	object_pose.position.z =  0.04;
 
	collision_object.meshes.push_back(object_mesh);
	collision_object.mesh_poses.push_back(object_pose);
	collision_object.operation = collision_object.ADD;

  object_vector.push_back(collision_object);
  bool success = planning_scene_interface_.applyCollisionObjects(object_vector);
  return success;
}

////////////////////////////////////////////////////////////////////////////////
bool 
CW3::RemoveCollisionObject(std::string object_type)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  collision_object.id = object_type;
  collision_object.operation = collision_object.REMOVE;
  object_vector.push_back(collision_object);
  
  bool success = planning_scene_interface_.applyCollisionObjects(object_vector);
  
  return success;
}