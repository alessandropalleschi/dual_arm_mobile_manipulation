

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ros/time.h"

#include <sstream>


int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "initial_pose_traj");
    ros::NodeHandle n;

    ros::Publisher traj_command_ur10e = n.advertise<trajectory_msgs::JointTrajectory>("/ur10e_eff_joint_traj_controller/command", 100);
    ros::Publisher traj_command_panda = n.advertise<trajectory_msgs::JointTrajectory>("/panda_eff_joint_traj_controller/command", 100);

    

    ros::Rate loop_rate(1);

    // Create a JointTrajectory with all positions set to zero, and command the arm.
    if(ros::ok())
    {
    // Create a message to send.
    trajectory_msgs::JointTrajectory pose_ur10e;

    // Fill the names of the joints to be controlled.
    pose_ur10e.joint_names.clear();
    pose_ur10e.joint_names.push_back("ur10e_shoulder_pan_joint");
    pose_ur10e.joint_names.push_back("ur10e_shoulder_lift_joint");
    pose_ur10e.joint_names.push_back("ur10e_elbow_joint");
    pose_ur10e.joint_names.push_back("ur10e_wrist_1_joint");
    pose_ur10e.joint_names.push_back("ur10e_wrist_2_joint");
    pose_ur10e.joint_names.push_back("ur10e_wrist_3_joint");

    // Create one point in the trajectory.
    pose_ur10e.points.resize(1);

    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    // pose_ur10e.points[0].positions.resize(pose_ur10e.joint_names.size(), 1.0);
    pose_ur10e.points[0].positions.resize(6);
    pose_ur10e.points[0].positions[0] = 0;
    pose_ur10e.points[0].positions[1] = -1.5;
    pose_ur10e.points[0].positions[2] = 2.8;
    pose_ur10e.points[0].positions[3] = -1.3;
    pose_ur10e.points[0].positions[4] = 1.57;
    pose_ur10e.points[0].positions[5] = 3.14;

    pose_ur10e.points[0].velocities.resize(6);
    pose_ur10e.points[0].velocities[0] = 0.0;
    pose_ur10e.points[0].velocities[1] = 0.0;
    pose_ur10e.points[0].velocities[2] = 0.0;
    pose_ur10e.points[0].velocities[3] = 0.0;
    pose_ur10e.points[0].velocities[4] = 0.0;
    pose_ur10e.points[0].velocities[5] = 0.0;

    pose_ur10e.points[0].accelerations.resize(6);
    pose_ur10e.points[0].effort.resize(6);

    // Create a message to send.
    trajectory_msgs::JointTrajectory pose_panda;

    // Fill the names of the joints to be controlled.
    pose_panda.joint_names.clear();
    pose_panda.joint_names.push_back("panda_joint1");
    pose_panda.joint_names.push_back("panda_joint2");
    pose_panda.joint_names.push_back("panda_joint3");
    pose_panda.joint_names.push_back("panda_joint4");
    pose_panda.joint_names.push_back("panda_joint5");
    pose_panda.joint_names.push_back("panda_joint6");
    pose_panda.joint_names.push_back("panda_joint7");

    // Create one point in the trajectory.
    pose_panda.points.resize(1);

    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    // pose_ur10e.points[0].positions.resize(pose_ur10e.joint_names.size(), 1.0);
    pose_panda.points[0].positions.resize(7);
    pose_panda.points[0].positions[0] = 0;
    pose_panda.points[0].positions[1] = -0.7853;
    pose_panda.points[0].positions[2] = 0;
    pose_panda.points[0].positions[3] = -2.35619;
    pose_panda.points[0].positions[4] = 0;
    pose_panda.points[0].positions[5] = 1.57079;
    pose_panda.points[0].positions[6] = 0.785398;

    pose_panda.points[0].velocities.resize(7);
    pose_panda.points[0].velocities[0] = 0.0;
    pose_panda.points[0].velocities[1] = 0.0;
    pose_panda.points[0].velocities[2] = 0.0;
    pose_panda.points[0].velocities[3] = 0.0;
    pose_panda.points[0].velocities[4] = 0.0;
    pose_panda.points[0].velocities[5] = 0.0;
    pose_panda.points[0].velocities[6] = 0.0;

    pose_panda.points[0].accelerations.resize(7);
    pose_panda.points[0].effort.resize(7);

    ros::Time start_time = ros::Time::now();
    while( start_time == ros::Time::now()){
        start_time = ros::Time::now();
        // ROS_INFO_STREAM ("tè \n" << start_time);
    }

    pose_ur10e.header.stamp = ros::Time::now() + ros::Duration(1.1);
    pose_panda.header.stamp = ros::Time::now() + ros::Duration(1.0);

    // How long to take getting to the point (floating point seconds).
    pose_ur10e.points[0].time_from_start = ros::Duration(3);
    pose_panda.points[0].time_from_start = ros::Duration(3);

    ROS_INFO_STREAM ("Sending command to UR10e: \n" << pose_ur10e);
    ROS_INFO_STREAM ("Sending command to Panda: \n" << pose_panda);


    traj_command_ur10e.publish(pose_ur10e);
    traj_command_panda.publish(pose_panda);

    ROS_INFO_STREAM("\n roba inviata\n");

    ros::spinOnce();

    loop_rate.sleep();
 }
 
 ROS_INFO_STREAM("\n Ha funzionato?\n");
 return 0;
}
