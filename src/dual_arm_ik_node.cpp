#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>

#define DoF 9
const double deg2rad = M_PI / 180.0;
const double SAMPLING_TIME = 0.001; // 1000Hz

Eigen::VectorXd current_q  = Eigen::VectorXd::Zero(DoF);
Eigen::VectorXd current_qv = Eigen::VectorXd::Zero(DoF);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_q(0) = msg->position[8]; // Waist
    current_q(1) = msg->position[1]; // L_shoulder_pitch
    current_q(2) = msg->position[2]; // L_shoulder_roll
    current_q(3) = msg->position[3]; // L_shoulder_yaw
    current_q(4) = msg->position[0]; // L_elbow
    current_q(5) = msg->position[5]; // R_shoulder_pitch
    current_q(6) = msg->position[6]; // R_shoulder_roll
    current_q(7) = msg->position[7]; // R_shoulder_yaw
    current_q(8) = msg->position[4]; // R_elbow

    current_qv(0) = msg->velocity[8];
    current_qv(1) = msg->velocity[1];
    current_qv(2) = msg->velocity[2];
    current_qv(3) = msg->velocity[3];
    current_qv(4) = msg->velocity[0];
    current_qv(5) = msg->velocity[5];
    current_qv(6) = msg->velocity[6];
    current_qv(7) = msg->velocity[7];
    current_qv(8) = msg->velocity[4];
}

// =====================================================================
// Quintic Trajectory 생성 함수 (선배 코드와 동일한 방식)
// 목표: 현재 자세 → 목표 자세까지 S자 곡선으로 부드럽게 이동
// 왜? 목표값을 바로 PD에 넣으면 급격한 힘이 발생해서 떨림이 생김
//     Quintic은 시작/끝에서 속도=0, 가속도=0이라 부드럽게 움직임
// =====================================================================
void computeQuinticTrajectory(
    const Eigen::VectorXd& q_ini,
    const Eigen::VectorXd& q_cmd,
    Eigen::MatrixXd& q_out,
    Eigen::MatrixXd& q_dot_out,
    Eigen::MatrixXd& q_acc_out)
{
    double q_dot_des = 0.5; // 원하는 각속도 (rad/s)

    // 최대 오차 기준으로 전체 궤적 시간 계산
    double max_error = 0;
    for (int i = 0; i < DoF; i++) {
        double err = fabs(q_cmd(i) - q_ini(i));
        if (err > max_error) max_error = err;
    }

    double Tf = max_error / q_dot_des;
    if (Tf < 0.5) Tf = 0.5; // 최소 0.5초 보장

    int step = round(Tf / SAMPLING_TIME);
    q_out.resize(step, DoF);
    q_dot_out.resize(step, DoF);
    q_acc_out.resize(step, DoF);

    for (int i = 0; i < DoF; i++) {
        double q0 = q_ini(i);
        double qf = q_cmd(i);
        double c0 = q0;
        double c1 = 0.0;
        double c2 = 0.0;
        double c3 =  10.0 * (qf - q0) / pow(Tf, 3);
        double c4 = -15.0 * (qf - q0) / pow(Tf, 4);
        double c5 =   6.0 * (qf - q0) / pow(Tf, 5);

        for (int j = 0; j < step; j++) {
            double t = j * SAMPLING_TIME;
            q_out(j, i)     = c0 + c1*t + c2*t*t + c3*pow(t,3) + c4*pow(t,4) + c5*pow(t,5);
            q_dot_out(j, i) = c1 + 2*c2*t + 3*c3*t*t + 4*c4*pow(t,3) + 5*c5*pow(t,4);
            q_acc_out(j, i) = 2*c2 + 6*c3*t + 12*c4*t*t + 20*c5*pow(t,3);
        }
    }
}

// =====================================================================
// DLS Cartesian IK 함수
// =====================================================================
Eigen::VectorXd cartesianIK(
    pinocchio::Model& model,
    pinocchio::Data& data,
    Eigen::VectorXd q,
    const Eigen::Vector3d& target_pos,
    pinocchio::FrameIndex ee_id,
    int max_iter = 100,
    double tol = 1e-4,
    double damping = 0.05)
{
    for (int iter = 0; iter < max_iter; iter++) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        Eigen::Vector3d current_pos = data.oMf[ee_id].translation();
        Eigen::Vector3d error = target_pos - current_pos;

        if (error.norm() < tol) {
            ROS_INFO("IK converged at iter %d, error: %.6f", iter, error.norm());
            break;
        }

        pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q, ee_id,
                                        pinocchio::LOCAL_WORLD_ALIGNED, J);
        Eigen::MatrixXd Jp = J.topRows(3);
        double lambda2 = damping * damping;
        Eigen::MatrixXd JJt = Jp * Jp.transpose();
        Eigen::MatrixXd dls_inv = Jp.transpose() * (JJt + lambda2 * Eigen::Matrix3d::Identity()).inverse();
        q += 0.5 * dls_inv * error;
    }
    return q;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_arm_ik_node");
    ros::NodeHandle nh;

    std::string urdf_path =
        "/home/jungmin/catkin_ws/src/Dual_Arm_ROS_Simulation/urdf/dual_arm.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    pinocchio::FrameIndex l_EE = model.getFrameId("L_EE_joint");
    pinocchio::FrameIndex r_EE = model.getFrameId("R_EE_joint");

    ros::Subscriber joint_sub = nh.subscribe(
        "/dual_arm/joint_states", 10, jointStateCallback);

    std::vector<ros::Publisher> pubs(DoF);
    ros::Publisher error_pub = nh.advertise<std_msgs::Float64MultiArray>(
        "/dual_arm_ik/joint_error", 10);

    std::vector<std::string> topics = {
        "/dual_arm/joint1_effort_controller/command",
        "/dual_arm/joint2_effort_controller/command",
        "/dual_arm/joint3_effort_controller/command",
        "/dual_arm/joint4_effort_controller/command",
        "/dual_arm/joint5_effort_controller/command",
        "/dual_arm/joint6_effort_controller/command",
        "/dual_arm/joint7_effort_controller/command",
        "/dual_arm/joint8_effort_controller/command",
        "/dual_arm/joint9_effort_controller/command"
    };
    for (int i = 0; i < DoF; i++)
        pubs[i] = nh.advertise<std_msgs::Float64>(topics[i], 10);

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    // 선배 코드와 동일한 게인 사용
    double Kp[DoF] = { 500, 300, 200, 50, 100, 300, 200, 50, 100 };
    double Kd[DoF] = { 10, 3, 1.5, 0.5, 1, 3, 1.5, 0.5, 1 };
    double torque_limit = 100.0;

    while (ros::ok())
    {
        std::cout << "\n=== Dual Arm IK Controller ===" << std::endl;
        std::cout << "Select mode (0=종료):" << std::endl;
        std::cout << "  1: Joint Space IK  (관절 각도 직접 입력, 단위: degree)" << std::endl;
        std::cout << "  2: Cartesian Space IK - Left Arm  (왼손끝 목표 위치, 단위: m)" << std::endl;
        std::cout << "  3: Cartesian Space IK - Right Arm (오른손끝 목표 위치, 단위: m)" << std::endl;
        std::cout << "  4: Cartesian Space IK - Both Arms (양팔 동시 제어, 단위: m)" << std::endl;
        int mode;
        std::cin >> mode;

        if (mode == 0) break;

        Eigen::VectorXd target_q = current_q;

        if (mode == 1)
        {
            // degree로 입력받아서 radian으로 변환
            std::cout << "Enter 9 joint angles (deg) [waist l_sp l_sr l_sy l_e r_sp r_sr r_sy r_e]:" << std::endl;
            for (int i = 0; i < DoF; i++) {
                double deg;
                std::cin >> deg;
                target_q(i) = deg * deg2rad;
            }
        }
        else if (mode == 2 || mode == 3)
        {
            double x, y, z;
            std::cout << "Enter target position (x y z) in meters:" << std::endl;
            std::cin >> x >> y >> z;
            Eigen::Vector3d target_pos(x, y, z);
            pinocchio::FrameIndex ee_id = (mode == 2) ? l_EE : r_EE;
            target_q = cartesianIK(model, data, current_q, target_pos, ee_id);
        }
        else if (mode == 4)
        {
            double lx, ly, lz;
            std::cout << "Enter LEFT arm target position (x y z) in meters:" << std::endl;
            std::cin >> lx >> ly >> lz;
            Eigen::Vector3d l_target(lx, ly, lz);

            double rx, ry, rz;
            std::cout << "Enter RIGHT arm target position (x y z) in meters:" << std::endl;
            std::cin >> rx >> ry >> rz;
            Eigen::Vector3d r_target(rx, ry, rz);

            Eigen::VectorXd l_result = cartesianIK(model, data, current_q, l_target, l_EE);
            target_q = current_q;
            target_q(0) = l_result(0);
            for (int i = 1; i <= 4; i++) target_q(i) = l_result(i);

            Eigen::VectorXd r_result = cartesianIK(model, data, target_q, r_target, r_EE);
            for (int i = 5; i <= 8; i++) target_q(i) = r_result(i);
        }
        else
        {
            std::cout << "잘못된 모드입니다. 다시 선택하세요." << std::endl;
            continue;
        }

        // Quintic Trajectory 계산
        Eigen::MatrixXd q_traj, q_dot_traj, q_acc_traj;
        computeQuinticTrajectory(current_q, target_q, q_traj, q_dot_traj, q_acc_traj);

        ROS_INFO("Sending IK result to Gazebo (Quintic Trajectory, %d steps)...", (int)q_traj.rows());

        ros::Rate control_rate(1000); // 1000Hz

        for (int i = 0; i < q_traj.rows(); i++)
        {
            ros::spinOnce();

            Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model, data, current_q);

            for (int j = 0; j < DoF; j++) {
                double q_error   = q_traj(i, j)     - current_q(j);
                double qv_error  = q_dot_traj(i, j) - current_qv(j);
                double torque = Kp[j] * q_error + Kd[j] * qv_error + gravity(j);

                if (torque >  torque_limit) torque =  torque_limit;
                if (torque < -torque_limit) torque = -torque_limit;

                std_msgs::Float64 msg;
                msg.data = torque;
                pubs[j].publish(msg);
            }

            // 오차 발행 (rqt_plot용)
            std_msgs::Float64MultiArray error_msg;
            error_msg.data.resize(DoF);
            for (int j = 0; j < DoF; j++)
                error_msg.data[j] = q_traj(i, j) - current_q(j);
            error_pub.publish(error_msg);

            control_rate.sleep();
        }

        // 자세 유지 루프 (다음 명령 입력 전까지 마지막 자세 홀드)
        // 왜? Done 후 토크가 안 가면 중력으로 팔이 떨어짐
        ROS_INFO("Done. 자세 유지 중... 다음 명령을 입력하세요.");
        

    

    } // while 루프 끝

    return 0;
}
