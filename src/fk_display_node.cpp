#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <Eigen/Dense>

pinocchio::Model model;
pinocchio::Data* data_ptr;
pinocchio::FrameIndex l_EE, r_EE;

#define DoF 9
Eigen::VectorXd current_q = Eigen::VectorXd::Zero(DoF);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_q(0) = msg->position[8];
    current_q(1) = msg->position[1];
    current_q(2) = msg->position[2];
    current_q(3) = msg->position[3];
    current_q(4) = msg->position[0];
    current_q(5) = msg->position[5];
    current_q(6) = msg->position[6];
    current_q(7) = msg->position[7];
    current_q(8) = msg->position[4];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fk_display_node");
    ros::NodeHandle nh;

    std::string urdf_path =
        "/home/jungmin/catkin_ws/src/Dual_Arm_ROS_Simulation/urdf/dual_arm.urdf";
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);
    data_ptr = &data;

    l_EE = model.getFrameId("L_EE_joint");
    r_EE = model.getFrameId("R_EE_joint");

    ros::Subscriber sub = nh.subscribe("/dual_arm/joint_states", 10, jointStateCallback);
    ros::Rate rate(1); // 1초에 한 번 출력

    while (ros::ok())
    {
        ros::spinOnce();

        pinocchio::forwardKinematics(model, data, current_q);
        pinocchio::updateFramePlacements(model, data);

        auto l_pos = data.oMf[l_EE].translation();
        auto l_rot = data.oMf[l_EE].rotation();
        auto r_pos = data.oMf[r_EE].translation();
        auto r_rot = data.oMf[r_EE].rotation();

        ROS_INFO("=== Left End-Effector Pose ===");
        ROS_INFO("Position: %.4f %.4f %.4f", l_pos(0), l_pos(1), l_pos(2));
        ROS_INFO("Orientation:\n%.4f %.4f %.4f\n%.4f %.4f %.4f\n%.4f %.4f %.4f",
            l_rot(0,0), l_rot(0,1), l_rot(0,2),
            l_rot(1,0), l_rot(1,1), l_rot(1,2),
            l_rot(2,0), l_rot(2,1), l_rot(2,2));
        ROS_INFO("=== Right End-Effector Pose ===");
        ROS_INFO("Position: %.4f %.4f %.4f", r_pos(0), r_pos(1), r_pos(2));
        ROS_INFO("Orientation:\n%.4f %.4f %.4f\n%.4f %.4f %.4f\n%.4f %.4f %.4f",
            r_rot(0,0), r_rot(0,1), r_rot(0,2),
            r_rot(1,0), r_rot(1,1), r_rot(1,2),
            r_rot(2,0), r_rot(2,1), r_rot(2,2));

        rate.sleep();
    }
    return 0;
}