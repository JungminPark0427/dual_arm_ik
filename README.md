# Dual Arm IK — 실행 가이드

피노키오(Pinocchio) + DLS 알고리즘으로 구현한 듀얼암 로봇 역기구학(IK) 패키지.

## 환경

- Ubuntu 20.04
- ROS Noetic
- Pinocchio (robotpkg-py38-pinocchio)
- Gazebo 11

---

## 설치

```bash
cd ~/catkin_ws/src
git clone https://github.com/JungminPark0427/dual_arm_ik.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) -l$(nproc)
source ~/catkin_ws/devel/setup.bash
```

주의: dual_arm_ik_node.cpp와 fk_display_node.cpp 안의 URDF 경로를 본인 경로로 수정해야 합니다.

---

## 실행 방법

터미널 3개를 순서대로 실행합니다.

터미널 1 - ROS 마스터
```bash
roscore
```

터미널 2 - Gazebo + FK 실시간 출력
```bash
roslaunch dual_arm_ik gazebo_ik.launch
```

터미널 3 - IK 명령 입력
```bash
rosrun dual_arm_ik dual_arm_ik_node
```

---

## 입력 예시

모드 1 - Joint Space IK
1
0 1.0 0 0 1.0 1.0 0 0 1.0
모드 2 - Cartesian IK 왼팔
2
0.2 0.3 0.6
모드 3 - Cartesian IK 오른팔
3
-0.2 -0.3 0.6---

## 오차 데이터 녹화 및 그래프 분석

1. rosbag 녹화
```bash
rosbag record -O /tmp/ik_error.bag /dual_arm_ik/joint_error
```

2. CSV 변환
```bash
rostopic echo -b /tmp/ik_error.bag -p /dual_arm_ik/joint_error > /tmp/ik_error.csv
```

3. 그래프 출력
```bash
pip3 install matplotlib pandas
python3 plot_error.py
```

---

## 패키지 구조
dual_arm_ik/
├── src/
│   ├── dual_arm_ik_node.cpp
│   └── fk_display_node.cpp
├── launch/
│   └── gazebo_ik.launch
└── CMakeLists.txt---

## 현재 게인 설정

Kp = { 500, 500, 500, 500, 500, 500, 500, 500, 500 }
Kd = { 10,  10,  10,  10,  10,  10,  10,  10,  10  }
torque_limit = 100 Nm
