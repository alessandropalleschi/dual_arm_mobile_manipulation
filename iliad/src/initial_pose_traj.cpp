
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ros/time.h"

#include <sstream>


int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "initial_pose_traj");
    ros::NodeHandle n;

    ros::Publisher traj_command = n.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 100);

    ros::Rate loop_rate(1);

    // Create a JointTrajectory with all positions set to zero, and command the arm.
    if(ros::ok())
    {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    // Fill the names of the joints to be controlled.
    msg.joint_names.clear();
    msg.joint_names.push_back("ur10e_shoulder_pan_joint");
    msg.joint_names.push_back("ur10e_shoulder_lift_joint");
    msg.joint_names.push_back("ur10e_elbow_joint");
    msg.joint_names.push_back("ur10e_wrist_1_joint");
    msg.joint_names.push_back("ur10e_wrist_2_joint");
    msg.joint_names.push_back("ur10e_wrist_3_joint");

    // Create one point in the trajectory.
    msg.points.resize(1);

    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    // msg.points[0].positions.resize(msg.joint_names.size(), 1.0);
    msg.points[0].positions.resize(6);
    msg.points[0].positions[0] = 0.0;
    msg.points[0].positions[1] = 0.0;
    msg.points[0].positions[2] = 0.0;
    msg.points[0].positions[3] = 0.0;
    msg.points[0].positions[4] = 0.0;
    msg.points[0].positions[5] = 0.0;

    msg.points[0].velocities.resize(6);
    msg.points[0].velocities[0] = 0.0;
    msg.points[0].velocities[1] = 0.0;
    msg.points[0].velocities[2] = 0.0;
    msg.points[0].velocities[3] = 0.0;
    msg.points[0].velocities[4] = 0.0;
    msg.points[0].velocities[5] = 0.0;

    msg.points[0].accelerations.resize(6);
    msg.points[0].effort.resize(6);

    ros::Time start_time = ros::Time::now();
    while( start_time == ros::Time::now()){
        start_time = ros::Time::now();
        ROS_INFO_STREAM ("il tempo è \n" << start_time);
    }
        
    msg.header.stamp = ros::Time::now() + ros::Duration(1.0);
    
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(5);

    

    ROS_INFO_STREAM ("Sending command:\n" << msg);


    traj_command.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
 }
 
 ROS_INFO_STREAM("Ha funzionato?\n");
 return 0;
}
