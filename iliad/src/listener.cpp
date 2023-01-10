
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <control_msgs/JointTrajectoryControllerState.h>
#include "ros/time.h"


control_msgs::JointTrajectoryControllerStateConstPtr controller_state;

void chatterCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& state)
{
    ros::Rate loop_rate(1);

    controller_state = state;
    ROS_INFO_STREAM ("\n ------------------------------------------------------------------ \n "
                        "Received ur10e state: \n" << state->error);

    loop_rate.sleep();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<control_msgs::JointTrajectoryControllerState>("/panda_eff_joint_traj_controller/state", 1, chatterCallback);

    ros::spin();
    return 0;
}
