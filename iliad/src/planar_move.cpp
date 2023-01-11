#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Pose.h"
#include "ros/time.h"
#include <gazebo_msgs/LinkStates.h>

#include <sstream>

gazebo_msgs::LinkStates link_state;

geometry_msgs::Pose base_pose,base_goal;

void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& state)
{
    ros::Rate loop_rate(50);

   // ROS_INFO_STREAM ("\n ------------------------------------------------------------------ \n "
     //                   "Name: " << state->name[7]);
   // ROS_INFO_STREAM ("\n ----------------------------------------------------------------- \n "
     //                   "Pose: " << state->pose[7]);
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
    
   // ROS_INFO_STREAM ("\n ----------------------------------------------------------------- \n "
     //                   "Pose: " << base_pose);
    

    loop_rate.sleep();

}


int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "manager");
    ros::NodeHandle n;

    ros::Publisher summit_planar_control = n.advertise<geometry_msgs::Twist>("/robotnik_base_control/cmd_vel", 100);
    ros::Subscriber sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, chatterCallback);

    ros::Rate loop_rate(10);


    base_goal.position.x = 0;
    base_goal.position.y = 0;
    base_goal.position.z = 0.0;
    base_goal.orientation.x= 0.0;
    base_goal.orientation.y= 0.0;
    base_goal.orientation.z= 0.0;
    base_goal.orientation.w= 1;

    // Create a JointTrajectory with all positions set to zero, and command the arm.
    while(ros::ok())
    {
        // Create a message to send.
        geometry_msgs::Twist base_twist;

        double k = 0.5; 
        
        base_twist.linear.x = k*(base_goal.position.x-base_pose.position.x);
        base_twist.linear.y = k*(base_goal.position.y-base_pose.position.y);
        base_twist.linear.z = 0.0;
        base_twist.angular.x = 0.0;
        base_twist.angular.y = 0.0;
        base_twist.angular.z = k*(base_goal.orientation.z-base_pose.orientation.z);     //qui ci sta una bella trasformazioncina 
    
        ROS_INFO_STREAM ("Sending command to Summit_base: \n" << base_twist);

        summit_planar_control.publish(base_twist);

        ros::spinOnce();

        loop_rate.sleep();
 }
 
 ROS_INFO_STREAM("\n Ha funzionato?\n");
 return 0;
}
