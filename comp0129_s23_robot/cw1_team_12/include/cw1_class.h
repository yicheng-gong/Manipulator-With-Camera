/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/conditional_removal.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <map>
// // include any services created in this package
// #include "cw1_team_x/example.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

// define parent class object
class SceneObject
{
public:
  SceneObject();
  /* ----- class member variables ----- */
  std::string object_name;
  std::string object_color;
  geometry_msgs::Vector3 collision_size_;
  geometry_msgs::Quaternion orientation ;
  geometry_msgs::Point location;
};
class Basket: public SceneObject
{
public:
  Basket(std::string object_name, std::string object_color,
    geometry_msgs::Point location);
};
class Cube: public SceneObject
{
public:
  Cube(std::string object_name, std::string object_color,
    geometry_msgs::Point location);
};

class CW1
{
public:

  /* ----- class member functions ----- */

  // constructor
  CW1(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  // execution function for tasks 1, 2, and 3
  bool
  T1Execution(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response);
  bool
  T2Execution(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response);
  bool
  T3Execution(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response);

  /** \brief MoveIt function for moving joints to predefined position 
    *
    * \input[in] none
    *
    * \return true if arm home ready
    */
  bool 
  Home();

  /** \brief MoveIt function for moving arm to new position 
    *
    * \input[in] arm's last joint position 
    *
    * \return true if arm moved to the new position
    */
  bool 
  MoveArm(geometry_msgs::Pose target_pose);

  /** \brief MoveIt function for moving the gripper fingers to a new position. 
    *
    * \input[in] width desired gripper finger width
    *
    * \return true if gripper fingers are moved to the new position
    */
  bool 
  MoveGripper(float width);

  /** \brief MoveIt function for moving arm to pick an object 
    *
    * \input[in] desired object picking position
    *
    * \return true if the object is picked
    */
  bool
  Pick(Cube cube_object);

  /** \brief MoveIt function for moving arm to place an object 
    *
    * \input[in] desired object placing position
    *
    * \return true if the object is picked
    */
  bool
  Place(Basket basket_object);

  /** \brief Add collision object for planning 
    *
    * \input[in] object_name, centre, dimensions, orientation
    *
    * \return none
    */
  bool
  AddCollisionObject(std::string object_name,
    geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
    geometry_msgs::Quaternion orientation);

  /** \brief Remove collision object by name 
    *
    * \input[in] name
    *
    * \return none
    */
  bool 
  RemoveCollisionObject(std::string object_name);

  /** \brief Add collision object by name 
    *
    * \input[in] name
    *
    * \return none
    */
  bool
  AddCollisionsInScene(std::vector<Cube> cube_vector_,
    std::vector<Basket> basket_vector_);

  /** \brief MoveIt function for moving arm to pick and  place an object 
    *
    * \input[in] desired object  position
    *
    * \return true if the action is correct
    */
  bool
  PickAndPlace(Cube object_cube, Basket object_basket);

  /** \brief Scane object by given location and store them to class varible
    *
    * \input[in] desired object  position
    *
    * \return true if the action is successs
    */
  bool
  ScanObjects(std::vector<geometry_msgs::PointStamped> &baskect_loc);

  /** \brief Apply Voxel Grid filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
  bool
  ApplyVX (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  
  /** \brief Subscriber call back for sensor
    *
    * \input[in] messages from sensor
    *
    * \return publisher for messages after filter
    */
  void
  SensorMsgsCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  /** \brief Judge point if in a certain range 
    *
    * \input[in] sensor_points PointT
    * 
    * \return true if point does not in the range 
    */
  bool
  IsNotInRange(PointT sensor_points);

  /** \brief Judge points color 
    *
    * \input[in] sensor_points PointT
    *
    * \return string for labelling points color
    */
  std::string
  JugColor(PointT sensor_points);

  /** \brief Judge PointC color
    *
    * \input[in] sensor_cloud the PointC pointor
    *
    * \return string for labelling PointC color
    */
  std::string
  SensorJugColor(PointCPtr sensor_cloud);

  /** \brief MoveIt function for moving arm to scan the whole screen
    *
    * \return true if the arm is moved
    */
  bool
  ScanPose();

  /** \brief Classify object color
    *
    * \input[in] sensor_cloud the PointC pointor
    *
    * \return true if object color classified
    */
  bool
  ColorClassify(PointCPtr sensor_cloud);

  /** \brief Classify objects into basket and cube
    *
    * \input[in] object_map PointC with color label 
    *
    * \return true if object type is classified
    */
  bool 
  StoreCubeAndBasket(std::map<std::string,PointC> object_map);

  /** \brief Add ground collision 
    *
    *
    * \return none
    */
  void
  AddGroundCollision();


  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  /** \brief Experimental values for arm to pick and place. */
  double pick_offset_z_ = 0.14;
  double holding_offset_z_ = 0.3;
  double place_offset_z_ = 0.3;

  /** \brief Define some useful constant values. */
  std::string base_frame_ = "panda_link0";
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  
  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /** \brief storing the information of basket and cude by vector */
  std::vector<Cube> cube_vector_;
  std::vector<Basket> basket_vector_;

  /** \brief Voxel Grid filter's leaf size. */
  double g_vg_leaf_sz_ = 0.01;
  
  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr_;

  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered_;
  
  /** \brief Sensor offset. */
  double sensor_zoffset_ = 0.5;
  /** \brief Store Object that detected*/
  std::map<std::string,PointC> object_map_;
};

#endif // end of include guard for CW1_CLASS_H_
