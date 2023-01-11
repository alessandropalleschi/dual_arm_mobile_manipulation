#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <gazebo_msgs/LinkStates.h>
#include "ros/time.h"

gazebo_msgs::LinkStates link_state;

geometry_msgs::Pose base_pose;

void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& state)
{
    ros::Rate loop_rate(10);

   // ROS_INFO_STREAM ("\n ------------------------------------------------------------------ \n "
     //                   "Name: " << state->name[7]);
    ROS_INFO_STREAM ("\n ----------------------------------------------------------------- \n "
                        "Pose: " << state->pose[7]);
   // ROS_INFO_STREAM ("\n ------------------------------------------------------------------ \n "
     //                   "Twist: \n" << state->twist[7]);

    base_pose.position.x = 0.0;
    base_pose.position.y = 0.0;
    base_pose.position.z = 0.0;
    base_pose.orientation.x= 0.0;
    base_pose.orientation.y= 0.0;
    base_pose.orientation.z= 0.0;
    base_pose.orientation.w= 1;

    base_pose = state->pose[7];
    
    ROS_INFO_STREAM ("\n ----------------------------------------------------------------- \n "
                        "Pose: " << base_pose);
    
    
    loop_rate.sleep();


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, chatterCallback);
    
    ros::spin();
    return 0;
}
