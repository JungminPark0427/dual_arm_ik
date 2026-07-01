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
#include <std_msgs/Float64MultiArray.h>

// =====================================================================
// 전역 변수: 현재 로봇의 관절 각도를 저장하는 곳
// (ROS 콜백이 실시간으로 업데이트해줌)
// =====================================================================
#define DoF 9
Eigen::VectorXd current_q = Eigen::VectorXd::Zero(DoF);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Gazebo에서 오는 관절 상태를 current_q에 저장
    // 선배 코드와 동일한 인덱스 순서 사용
    current_q(0) = msg->position[8]; // Waist
    current_q(1) = msg->position[1]; // L_shoulder_pitch
    current_q(2) = msg->position[2]; // L_shoulder_roll
    current_q(3) = msg->position[3]; // L_shoulder_yaw
    current_q(4) = msg->position[0]; // L_elbow
    current_q(5) = msg->position[5]; // R_shoulder_pitch
    current_q(6) = msg->position[6]; // R_shoulder_roll
    current_q(7) = msg->position[7]; // R_shoulder_yaw
    current_q(8) = msg->position[4]; // R_elbow
}

// =====================================================================
// DLS(Damped Least Squares) Cartesian IK 함수
// 목표: 손끝 목표 위치(target_pos)를 받아서
//       자코비안 역행렬로 필요한 관절 각도(q)를 반복 계산
// 왜 DLS? 특이점(관절이 일직선이 되는 위험한 자세)에서도 안정적으로 계산 가능
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
    for (int iter = 0; iter < max_iter; iter++)
    {
        // 1. 현재 관절 각도로 FK 계산 (손끝 현재 위치 구하기)
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        Eigen::Vector3d current_pos = data.oMf[ee_id].translation();

        // 2. 오차 계산 (목표 위치 - 현재 위치)
        Eigen::Vector3d error = target_pos - current_pos;

        // 3. 오차가 충분히 작으면 수렴 완료 → 종료
        if (error.norm() < tol) {
            ROS_INFO("IK converged at iter %d, error: %.6f", iter, error.norm());
            break;
        }

        // 4. 자코비안 계산 (관절 속도 → 손끝 속도 변환 행렬)
        pinocchio::Data::Matrix6x J = pinocchio::Data::Matrix6x::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q, ee_id,
                                         pinocchio::LOCAL_WORLD_ALIGNED, J);

        // 위치만 쓸 거라 자코비안 위 3행(x,y,z)만 사용
        Eigen::MatrixXd Jp = J.topRows(3);

        // 5. DLS 역행렬 계산
        // 일반 역행렬 대신 (J*J^T + λ²I)^-1 * J^T 사용 → 특이점에서도 안정적
        double lambda2 = damping * damping;
        Eigen::MatrixXd JJt = Jp * Jp.transpose();
        Eigen::MatrixXd dls_inv = Jp.transpose() * (JJt + lambda2 * Eigen::Matrix3d::Identity()).inverse();

        // 6. 관절 각도 업데이트 (작은 스텝으로 반복 수렴)
        double step = 0.5;
        q += step * dls_inv * error;
    }
    return q;
}

// =====================================================================
// Joint Space IK 함수
// 목표: 그냥 사용자가 입력한 관절 각도를 그대로 목표로 설정
// 왜 별도 함수? 나중에 궤적 생성(Quintic 등) 붙이기 쉽게 하려고
// =====================================================================
Eigen::VectorXd jointSpaceIK(const Eigen::VectorXd& target_q)
{
    return target_q; // Joint space는 목표 각도가 곧 IK 결과
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_arm_ik_node");
    ros::NodeHandle nh;

    // URDF 로드 (피노키오가 로봇 구조 파악하는 데 필요)
    std::string urdf_path =
        "/home/jungmin/catkin_ws/src/Dual_Arm_ROS_Simulation/urdf/dual_arm.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    // 엔드이펙터 프레임 ID 가져오기 (선배 코드와 동일한 이름 사용)
    pinocchio::FrameIndex l_EE = model.getFrameId("L_EE_joint");
    pinocchio::FrameIndex r_EE = model.getFrameId("R_EE_joint");

    // 현재 관절 상태 구독 (실시간으로 current_q 업데이트)
    ros::Subscriber joint_sub = nh.subscribe(
        "/dual_arm/joint_states", 10, jointStateCallback);

    // 각 관절에 명령 보내는 퍼블리셔 (effort 제어 모드)
    std::vector<ros::Publisher> pubs(DoF);
    // 오차 토픽 퍼블리셔 (rqt_plot으로 시각화용)
ros::Publisher error_pub = nh.advertise<std_msgs::Float64MultiArray>("/dual_arm_ik/joint_error", 10);
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

    ros::Rate loop_rate(100);
    ros::spinOnce();
    ros::Duration(1.0).sleep(); // 관절 상태 수신 대기

    // =====================================================================
    // 모드 선택: 사용자가 터미널에서 직접 입력
    // =====================================================================
    std::cout << "=== Dual Arm IK Controller ===" << std::endl;
    std::cout << "Select mode:" << std::endl;
    std::cout << "  1: Joint Space IK  (관절 각도 직접 입력)" << std::endl;
    std::cout << "  2: Cartesian Space IK - Left Arm  (왼손끝 목표 위치 입력)" << std::endl;
    std::cout << "  3: Cartesian Space IK - Right Arm (오른손끝 목표 위치 입력)" << std::endl;
    int mode;
    std::cin >> mode;

    Eigen::VectorXd target_q = current_q;

    if (mode == 1)
    {
        // Joint Space: 9개 관절 각도 직접 입력 (라디안)
        std::cout << "Enter 9 joint angles (rad) [waist l_sp l_sr l_sy l_e r_sp r_sr r_sy r_e]:" << std::endl;
        for (int i = 0; i < DoF; i++) std::cin >> target_q(i);
        target_q = jointSpaceIK(target_q);
    }
    else if (mode == 2 || mode == 3)
    {
        // Cartesian Space: 손끝 목표 위치 입력
        double x, y, z;
        std::cout << "Enter target position (x y z) in meters:" << std::endl;
        std::cin >> x >> y >> z;
        Eigen::Vector3d target_pos(x, y, z);

        pinocchio::FrameIndex ee_id = (mode == 2) ? l_EE : r_EE;
        target_q = cartesianIK(model, data, current_q, target_pos, ee_id);
    }

    // 계산된 관절 각도를 Gazebo에 전송 (PD제어 + 중력보상)
// 왜 중력보상? PD만 쓰면 중력 때문에 팔이 흔들리거나 처짐
// 피노키오로 중력 토크 계산 → PD 토크에 더해서 안정적으로 자세 유지
ROS_INFO("Sending IK result to Gazebo...");

double Kp[DoF] = { 500, 500, 500, 500, 500, 500, 500, 500, 500 };
double Kd[DoF] = { 10, 10, 10, 20, 20, 10, 10, 20, 20 };
double torque_limit = 100.0;
double target_q_dot[DoF] = {0,};
Eigen::VectorXd prev_q = current_q;



ros::Rate control_rate(100);
int print_cnt = 0;

for (int i = 0; i < 3000; i++)
{
    ros::spinOnce();

    // 현재 속도 추정
    Eigen::VectorXd current_q_dot_est = (current_q - prev_q) * 100.0;
    prev_q = current_q;

    // 중력 보상 토크 계산
    // 왜? 중력이 각 관절을 아래로 당기는 힘을 피노키오로 정확히 계산해서 상쇄
    Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model, data, current_q);

    // FK로 현재 손끝 위치 계산 (출력용)
    pinocchio::forwardKinematics(model, data, current_q);
    pinocchio::updateFramePlacements(model, data);

    int print_cnt = 0;

    for (int j = 0; j < DoF; j++) {
        double q_error = target_q(j) - current_q(j);
        double P_term = Kp[j] * q_error;
        double D_term = Kd[j] * (target_q_dot[j] - current_q_dot_est(j));
        double torque = P_term + D_term + gravity(j); // 중력 보상 추가

        if (torque >  torque_limit) torque =  torque_limit;
        if (torque < -torque_limit) torque = -torque_limit;

        std_msgs::Float64 msg;
        msg.data = torque;
        pubs[j].publish(msg);
    }

    // 오차값을 토픽으로 발행 (rqt_plot 시각화용)
    std_msgs::Float64MultiArray error_msg;
    error_msg.data.resize(DoF);
    for (int j = 0; j < DoF; j++) {
        error_msg.data[j] = target_q(j) - current_q(j);
    }
    error_pub.publish(error_msg);

    control_rate.sleep();
}
    ROS_INFO("Done.");
    return 0;
}