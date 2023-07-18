/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

int main(int argc, char **argv){
  
  ros::init(argc,argv, "cw1_solution_node");
  ros::NodeHandle nh;

  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate loop_rate(10);

  // create an instance of the cw1 class
  CW1 cw_class(nh);
  ros::Subscriber sub_cloud = nh.subscribe("/r200/camera/depth_registered/points", 
                                           1, 
                                           &CW1::SensorMsgsCallback, 
                                           &cw_class);

  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}