# Dual Arm IK — 실행 가이드

피노키오(Pinocchio) + DLS 알고리즘으로 구현한 듀얼암 로봇 역기구학(IK) 패키지.  
선배 코드(`Dual_Arm_ROS_Simulation`) 기반으로 Cartesian Space IK + Quintic Trajectory를 추가 구현.

## 환경

- Ubuntu 20.04 + ROS Noetic
- Pinocchio (`robotpkg-py38-pinocchio`)
- Gazebo 11

---

## 선배 코드와의 차이점

| 항목 | 선배 코드 | 우리 코드 |
|------|-----------|-----------|
| IK 방식 | Joint Space만 | Joint Space + **Cartesian DLS IK** |
| 궤적 생성 | Quintic Trajectory | Quintic Trajectory (동일 적용) |
| 입력 단위 | degree | degree (동일) |
| 양팔 동시 제어 | 없음 | **모드 4 추가** |
| FK 출력 터미널 | 같은 터미널 | **터미널 분리** |

---

## 설치

```bash
cd ~/catkin_ws/src
git clone https://github.com/JungminPark0427/dual_arm_ik.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) -l$(nproc)
source ~/catkin_ws/devel/setup.bash
```

> **주의:** `dual_arm_ik_node.cpp`와 `fk_display_node.cpp` 안의 URDF 경로를 본인 경로로 수정해야 합니다.
> ```cpp
> std::string urdf_path = "/home/jungmin/catkin_ws/.../dual_arm.urdf";
> ```

---

## 실행 방법

터미널 3개를 순서대로 실행합니다.

**터미널 1 — ROS 마스터**
```bash
roscore
```

**터미널 2 — Gazebo + FK 실시간 출력**
```bash
roslaunch dual_arm_ik gazebo_ik.launch
```
Gazebo 창이 뜨고 안정되면 터미널 3으로 진행합니다.  
이 터미널에서 현재 손끝 위치(End-Effector Pose)가 실시간으로 출력됩니다.

**터미널 3 — IK 명령 입력**
```bash
rosrun dual_arm_ik dual_arm_ik_node
```

---

## 모드별 입력 예시

### 모드 1 — Joint Space IK (degree 입력)

```
1
0 57 0 0 57 57 0 0 57
```
→ 좀비 자세 (양팔 앞으로 들고 팔꿈치 굽힘)

```
1
0 0 0 0 0 0 0 0 0
```
→ 기본 자세 복귀

### 모드 2 — Cartesian IK 왼팔 (미터 입력)

```
2
0.3 0.3 0.7
```
→ 왼손끝을 (0.3, 0.3, 0.7) 위치로 이동

### 모드 3 — Cartesian IK 오른팔 (미터 입력)

```
3
0.3 -0.3 0.7
```
→ 오른손끝을 (0.3, -0.3, 0.7) 위치로 이동

### 모드 4 — 양팔 동시 Cartesian IK

```
4
0.3 0.3 0.7
0.3 -0.3 0.7
```
→ 양팔 대칭으로 앞으로 뻗기

### 종료

```
0
```

---

## 오차 데이터 녹화 및 그래프 분석

**1. rosbag 녹화** (새 터미널)
```bash
rosbag record -O /tmp/ik_error.bag /dual_arm_ik/joint_error
```

**2. 명령 입력 후 Done. 뜨면 Ctrl+C로 저장**

**3. CSV 변환**
```bash
rostopic echo -b /tmp/ik_error.bag -p /dual_arm_ik/joint_error > /tmp/ik_error.csv
```

**4. 그래프 출력**
```bash
pip3 install matplotlib pandas
python3 plot_error.py
```

---

## 패키지 구조

```
dual_arm_ik/
├── src/
│   ├── dual_arm_ik_node.cpp   # Joint Space + Cartesian DLS IK + Quintic Trajectory
│   └── fk_display_node.cpp    # FK 실시간 출력 노드 (터미널 2)
├── launch/
│   └── gazebo_ik.launch       # Gazebo + FK 노드 통합 launch
└── CMakeLists.txt
```

---

## 게인 설정 (선배 코드 동일)

```cpp
double Kp[DoF] = { 500, 300, 200, 50, 100, 300, 200, 50, 100 };
double Kd[DoF] = { 10, 3, 1.5, 0.5, 1, 3, 1.5, 0.5, 1 };
double torque_limit = 100.0; // Nm
```

---

## 참고

- 피노키오 공식 문서: https://stack-of-tasks.github.io/pinocchio/
- 선배 레포: `SeungJun1999/Dual_Arm_ROS_Simulation`
- 