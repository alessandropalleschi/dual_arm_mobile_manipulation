#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
int main (int argc, char** argv)

{



  ros::init(argc,argv,"test_node");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
  gazebo_msgs::SetModelState setmodelstate;
  gazebo_msgs::ModelState modelstate;

  ros::Rate loop_rate(10);

  modelstate.model_name = "iliad";
  modelstate.pose.position.x = 0.0;
  
  setmodelstate.request.model_state = modelstate;
  
  while (ros::ok()){
    if (client.call(setmodelstate))
    {
      ROS_INFO("BRILLIANT!!!");
      ROS_INFO("%f",modelstate.pose.position.x);
    }
    else
    {
      ROS_ERROR("Failed to call service ");
      return 1;
    }
    

    ros::spinOnce();
    loop_rate.sleep();
  
  return 0;}
}