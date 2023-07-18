/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw3_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW3_CLASS_H_
#define CW3_CLASS_H_

// system includes
// system includes
#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/shape_operations.h"
// include services from the spawner package - we will be responding to these
#include "cw3_world_spawner/Task1Service.h"
#include "cw3_world_spawner/Task2Service.h"
#include "cw3_world_spawner/Task3Service.h"
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
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl/io/vtk_lib_io.h>
#include <map>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/filesystem.hpp>
#include <fstream>
// // include any services created in this package

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

// define parent class object for any object in scene
class SceneObject
{
public:
  /* ----- class member variables ----- */
  std::string object_id_;
  std::string object_type_;
  std::string object_color_;
  double object_size_;
  geometry_msgs::Vector3 collision_size_;
  geometry_msgs::Quaternion orientation_ ;
  geometry_msgs::Point location_;
  double x_len;
  double y_len;
  PointCPtr object_pc;
  geometry_msgs::Point collision_center_;
  SceneObject(){};
  SceneObject(geometry_msgs::Point location_, double x_len, double y_len, PointCPtr object_pc); // one of the constructure
};

// define specific object type basket
class Basket: public SceneObject
{
public:
  Basket(geometry_msgs::Point location);
};

// define object that stores cross and nought
class Object: public SceneObject
{
public:
  Object(std::string object_type, double object_size, std::string object_color,
    geometry_msgs::Point location, geometry_msgs::Quaternion orientation);
};

// coursework class 
class CW3
{
  public:

    /* ----- class member functions ----- */

    // constructor
    CW3(ros::NodeHandle nh);

    // service callbacks for tasks 1, 2, and 3
    bool 
    t1_callback(cw3_world_spawner::Task1Service::Request &request,
      cw3_world_spawner::Task1Service::Response &response);
    bool 
    t2_callback(cw3_world_spawner::Task2Service::Request &request,
      cw3_world_spawner::Task2Service::Response &response);
    bool 
    t3_callback(cw3_world_spawner::Task3Service::Request &request,
      cw3_world_spawner::Task3Service::Response &response);

    // execution function for tasks 1, 2, and 3
    bool
    T1Execution(cw3_world_spawner::Task1Service::Request &request,
      cw3_world_spawner::Task1Service::Response &response);
    bool
    T2Execution(cw3_world_spawner::Task2Service::Request &request,
      cw3_world_spawner::Task2Service::Response &response);
    bool
    T3Execution(cw3_world_spawner::Task3Service::Request &request,
      cw3_world_spawner::Task3Service::Response &response);

    /** \brief MoveIt function for moving joints to predefined position 
      *
      * \input[in] none
      *
      * \return true if arm home ready
      */
    bool 
    Home();

    /** \brief store object and basket to vector 
      *
      * \input[in] none
      *
      * \return none
      */
    void
    StoreBasketAndObject(std::vector<SceneObject> undefined_obj_vector);

    /** \brief MoveIt function for moving joints to predefined position 
    *
    * \input[in] none
    *
    * \return true if arm home ready
    */
    bool
    ScanPose(int num);

    /** \brief Add ground collision 
      *
      *
      * \return none
      */
    void
    AddGroundCollision();

    /** \brief Add collision object by name 
      *
      * \input[in] name
      *
      * \return none
      */
    bool
    AddCollisionsInScene(std::vector<Object> &object_vector_,
      std::vector<Basket> &basket_vector_);

    
    /** \brief Point Cloud CallBack function.
      * 
      * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
      */
    void
    PointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

    /** \brief Apply Voxel Grid filtering.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      * \input[out] out_cloud_ptr the output PointCloud2 pointer
      */
    bool
    ApplyVX(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

    /** \brief Normal estimation.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findNormals(PointCPtr &in_cloud_ptr);

    /** \brief Segment Plane from point cloud.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    segPlane(PointCPtr &in_cloud_ptr);

    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    Object
    generateObject(PointCPtr &in_cloud_ptr, geometry_msgs::Pose ori_pose, bool with_ground = true);

    /** \brief Go and config a game object
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    Object
    goConfigObject(geometry_msgs::Point center_location);

    /** \brief Go and config a game object
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    PointC
    getObjcluster(PointCPtr &in_cloud_ptr);

    /** \brief Find the cluster of point cloud, with ground.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    void
    findPCRange(PointC &in_cloud_points);

    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    std::vector<pcl::PointIndices> 
    findCluster(PointCPtr &in_cloud_ptr, int point_num);

      /** \brief Find the Pose of Cylinder.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
    std::vector<SceneObject>
    preProcessCluster(PointCPtr &in_cloud_ptr, std::vector<pcl::PointIndices> cluster_indices);

    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    PointC
    removeGroundPoint(PointCPtr &in_cloud_ptr);

    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    std::string
    jugObjType(PointC &in_cloud_points);

    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    double
    jugObjSize(std::string obj_type, PointC &in_cloud_points);

    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    double
    computeRotationAngle(std::string obj_type, double obj_size, PointC &in_cloud_points);

    /** \brief Find the Pose of Cylinder.
      * 
      * \input[in] in_cloud_ptr the input PointCloud2 pointer
      */
    geometry_msgs::Pose
    computeGraspPose(std::string obj_type, double obj_size, geometry_msgs::Pose ori_pose, double rotation_angle);

    /** \brief MoveIt function for moving arm to new position 
      *
      * \input[in] arm's last joint position 
      *
      * \return true if arm moved to the new position
      */
    bool 
    MoveArm(geometry_msgs::Pose target_pose);

    /** \brief MoveIt function for moving arm to safe position 
      *
      * \input[in] arm's last joint position 
      *
      * \return true if arm moved to the safe position
      */
    bool 
    MoveBackToSafe();

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
    Pick(Object target_object);

    /** \brief MoveIt function for moving arm to place an object 
      *
      * \input[in] desired object placing position
      *
      * \return true if the object is picked
      */
    bool
    Place(Object target_object, Basket basket_object);

    /** \brief MoveIt function for moving arm to pick and  place an object 
      *
      * \input[in] desired object  position
      *
      * \return true if the action is correct
      */
    bool
    PickAndPlace(Object target_object, Basket target_basket);


    /** \brief Add collision object for planning 
      *
      * \input[in] object_type, centre, dimensions, orientation
      *
      * \return none
      */
    bool
    AddCollisionObject(std::string object_type,
      geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
      geometry_msgs::Quaternion orientation);
    
    /** \brief Remove collision object by name 
      *
      * \input[in] name
      *
      * \return none
      */
    bool 
    RemoveCollisionObject(std::string object_type);

    /** \brief Add collision object by the object characteristic
      *
      * \input[in] 
      *
      * \return none
      */
    bool
    AddCollision(std::string object_type, std::string object_id,
      geometry_msgs::Point centre, double dimensions,
      geometry_msgs::Quaternion orientation);


  public:
    /* ----- class member variables ----- */
    ros::NodeHandle nh_;
    ros::ServiceServer t1_service_;
    ros::ServiceServer t2_service_;
    ros::ServiceServer t3_service_;

    /** \brief Experimental values for arm to pick and place. */
    double pick_offset_z_ = 0.14;
    double holding_offset_z_ = 0.3;
    double place_offset_z_ = 0.25;

    std::string base_frame_ = "panda_link0";
    double gripper_open_ = 80e-3;
    double gripper_closed_ = 0.0;

    geometry_msgs::Quaternion scanning_ori;
    double scanning_offset_z = 0.5;

    double camera_offset_x = -0.04;

    /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
      * these are defined in urdf. */
    moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
    moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

    /** \brief MoveIt interface to interact with the moveit planning scene 
      * (eg collision objects). */
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    /** \brief storing the information of basket, nought and cross by vector */
    std::vector<Object> object_vector_;
    std::vector<Basket> basket_vector_;
    std::vector<std::string> obstacle_vector_;

    /** \brief object type name of nought and cross */
    std::string nought_type_name_ = "nought";
    std::string cross_type_name_ = "cross";

    std::vector<PointCPtr> obj_template;

    /** \brief The input point cloud range. */
    std::vector<float> x_pc_range;
    std::vector<float> y_pc_range;
    std::vector<float> z_pc_range;
    geometry_msgs::Point center_point;

    /** \brief The input point cloud frame id. */
    std::string g_input_pc_frame_id_;

    /** \brief ROS geometry message point. */
    geometry_msgs::PointStamped g_obj_pt_msg;

    /** \brief Point Cloud (input). */
    pcl::PCLPointCloud2 g_pcl_pc;

    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr;
    
    /** \brief Voxel Grid filter's leaf size. */
    double g_vg_leaf_sz;

    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr_;

    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered, g_cloud_filtered2;

    /** \brief ROS publishers. */
    ros::Publisher g_pub_cloud;

    /** \brief KDTree for nearest neighborhood search. */
    pcl::search::KdTree<PointT>::Ptr g_tree_ptr;
    
    /** \brief Normal estimation. */
    pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
    
    /** \brief Cloud of normals. */
    pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals, g_cloud_normals2;
    
    /** \brief Nearest neighborhooh size for normal estimation. */
    double g_k_nn;

    /** \brief SAC segmentation. */
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg; 
    
    /** \brief Extract point cloud indices. */
    pcl::ExtractIndices<PointT> g_extract_pc;

    /** \brief Extract point cloud normal indices. */
    pcl::ExtractIndices<pcl::Normal> g_extract_normals;
    
    /** \brief Point indices for plane. */
    pcl::PointIndices::Ptr g_inliers_plane;
      
    /** \brief Model coefficients for the plane segmentation. */
    pcl::ModelCoefficients::Ptr g_coeff_plane;

    /** \brief Point cloud to hold plane and cylinder points. */
    PointCPtr g_cloud_plane ;

    /** \brief cw1Q1: TF listener definition. */
    tf::TransformListener g_listener_;

    /** \brief THe joints to safe plan */
    std::vector<double> safe_joints;

    /** \brief THe number of cross object*/
    int cross_num ;
    /** \brief THe number of nought object*/
    int nought_num ;

  protected:
    /** \brief Debug mode. */
    bool debug_;

};

#endif // end of include guard for cw3_CLASS_H_
