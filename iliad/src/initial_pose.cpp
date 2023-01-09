
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "initial_pose");

  ros::NodeHandle n;

  ros::Publisher ur10e_1 = n.advertise<std_msgs::Float64>("ur10e_shoulder_pan_joint_position_controller/command", 1000);
  
  ros::Publisher ur10e_2 = n.advertise<std_msgs::Float64>("ur10e_shoulder_lift_joint_position_controller/command", 1000);

  ros::Publisher ur10e_3 = n.advertise<std_msgs::Float64>("ur10e_elbow_joint_position_controller/command", 1000);
  
  ros::Publisher ur10e_4 = n.advertise<std_msgs::Float64>("ur10e_wrist_1_joint_position_controller/command", 1000);
  
  ros::Publisher ur10e_5 = n.advertise<std_msgs::Float64>("ur10e_wrist_2_joint_position_controller/command", 1000);
  
  ros::Publisher ur10e_6 = n.advertise<std_msgs::Float64>("ur10e_wrist_3_joint_position_controller/command", 1000);

  
  ros::Rate loop_rate(1);

  float sec = 0;
  while (ros::ok() && sec <= 5)
  {
    
    std_msgs::Float64 v_pos;
    
    
    v_pos.data = 0;
    ur10e_1.publish(v_pos);
    v_pos.data = -1.5;
    ur10e_2.publish(v_pos);
    v_pos.data = 2.8;
    ur10e_3.publish(v_pos);
    v_pos.data = -1.3;
    ur10e_4.publish(v_pos);
    v_pos.data = 1.57;
    ur10e_5.publish(v_pos);
    v_pos.data = 3.14;
    ur10e_6.publish(v_pos);
    
    ros::spinOnce();

    loop_rate.sleep();
    
    ++sec;
  }


  return 0;
}
