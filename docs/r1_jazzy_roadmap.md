# OMOROBOT R1 v2 · ROS 2 Jazzy 개발 방향

## 현재 장비와 우선순위

사용자가 확인한 구성은 **OMOROBOT R1 v2, YDLIDAR G6, 외부 IMU 없음, 추가 3D LiDAR 미설치, ROS 2 Jazzy**이다. 따라서 첫 목표는 G6의 2D `/scan`과 R1의 바퀴 오도메트리를 이용한 **SLAM Toolbox → 지도 저장 → AMCL + Nav2 자율주행**이다. 바퀴 오도메트리의 제공 방식, 온보드 컴퓨터, 모터 드라이버, 센서 장착 위치, 로봇 외형과 비상정지 방식은 장비 또는 제공 패키지에서 확인해야 한다.

기존 S1 bag은 3D 점군·IMU 메시지 처리용 합성 파일럿이다. R1 v2나 G6, 바퀴 구동, Nav2 주행을 재현하지 않는다. 이를 R1 실험 결과로 해석하지 않는다.

제조사 [R1 v2 제품 소개](https://www.omorobot.com/R1v2)는 센서와 PC를 옵션으로 설명한다. 공개 [OMOROBOT ROS 2 저장소](https://github.com/omorobot/omo_r1-ros2)에 bringup, Gazebo, Cartographer, Navigation2 관련 코드가 있으나 README는 R1mini와 ROS 2 Foxy를 설명한다. R1 v2용 Jazzy 호환이 검증됐다고 가정하지 않고 필요한 드라이버와 메시지 인터페이스를 먼저 확인한다. [YDLIDAR ROS 2 드라이버](https://github.com/YDLIDAR/ydlidar_ros2_driver)는 `/scan`의 `sensor_msgs/LaserScan` 출력을 제공하며 G6 지원을 명시하지만, 이 장비 조합의 Jazzy 빌드와 실제 스캔은 별도 검증이 필요하다.

## 실행 순서와 완료 기준

| 순서 | 작업 | 확인할 결과 |
| --- | --- | --- |
| 1. 인터페이스 조사 | R1 v2 모터/엔코더 드라이버와 G6 드라이버의 Jazzy 빌드, 포트·권한, 토픽, 타입, `frame_id`, 발행 주기 확인 | `/scan`, 바퀴 `/odom`, 제어용 `/cmd_vel`의 실제 계약과 누락 항목을 기록 |
| 2. R1 v2 시뮬레이션 | 실측 전에는 임시값임을 표시한 URDF/TF/footprint, 차동구동, G6와 같은 2D 스캔, 바퀴 오도메트리 구성 | Gazebo Harmonic에서 조종·스캔·오도메트리 동작; R1 전용 bag과 시각화 생성 |
| 3. 2D 지도와 위치추정 | SLAM Toolbox로 지도 작성·저장 후, 같은 지도에서 AMCL 실행 | `map → odom → base_link → laser` 변환 연결, RViz에서 지도·스캔·위치 확인 |
| 4. Nav2 주행 | R1 v2용 planner/controller/costmap, 실제 footprint, 속도·가속도 제한 설정 | 시뮬레이션에서 목표점 주행, 장애물 회피, 취소·정지, 로그 확인 |
| 5. 실기 이전 | 실제 장비의 저속 수동 조종, 정지 기능, 센서와 TF 확인 후 동일한 2D 절차 적용 | 시뮬레이션 설정과 실측값 차이를 기록하고 안전한 시험 공간에서 검증 |

시뮬레이션에서 [Nav2 Jazzy 설정 가이드](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/)의 Gazebo Harmonic 경로를 기준으로 삼는다. [Nav2 지도 작성·위치추정 가이드](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/)는 2D `LaserScan`을 쓰는 SLAM Toolbox와 AMCL 구성을 설명한다. TF 발행 주체를 한 곳씩 정한다. 지도 작성 시에는 SLAM Toolbox가, 저장 지도 주행 시에는 AMCL이 `map → odom`을 담당한다. 바퀴 드라이버 또는 오도메트리 노드 중 하나만 `odom → base_link`를 발행한다. 외부 IMU가 없으므로 IMU 융합을 전제로 설계하지 않는다. 내장 IMU가 실제로 제공되는 경우에만 [robot_localization 구성](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/)을 별도 평가한다.

## Nav2와 기존 3D SLAM 계획의 관계

| 구성 | 역할 | 현재 R1 v2에서 필요한 입력 | 지금 가능한가 |
| --- | --- | --- | --- |
| SLAM Toolbox + AMCL + Nav2 | 2D 지도 생성, 지도 내 위치추정, 경로 계획·제어 | G6 `/scan`, 바퀴 `/odom`, `cmd_vel`, TF | 드라이버·시뮬레이션 준비 후 가능 |
| KISS-ICP | 3D LiDAR 기반 국소 오도메트리 | 3D `PointCloud2` | 추가 3D LiDAR 장착 전에는 실기 적용 불가 |
| FAST-LIO2 / LIO-SAM | 3D LiDAR·IMU 기반 국소 오도메트리 | 3D `PointCloud2`와 IMU | 3D LiDAR와 IMU 장착 전에는 실기 적용 불가 |

Nav2는 목표점까지 주행하는 체계이고 KISS-ICP·FAST-LIO2·LIO-SAM은 그 체계에 연결할 수 있는 국소 움직임 추정 방법이다. AMCL의 지도 내 위치추정과 3D LiDAR 오도메트리는 출력의 역할도 다르다. 지금은 Nav2를 기준선으로 만들고, 센서가 추가되면 동일한 Nav2 설정에서 오도메트리 공급원만 바꿔 주행 성능을 비교한다. G6의 2D `LaserScan`을 3D LiDAR 데이터와 동등하게 취급하지 않는다.

향후 3D LiDAR만 추가하면 KISS-ICP를 먼저 검토할 수 있다. FAST-LIO2와 LIO-SAM 실기 실험은 IMU도 추가되고 시간 동기화·센서 외부 변환을 검증한 뒤 진행한다. 3D 점군을 Nav2 장애물 감지에 쓰는 [Voxel Layer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/)는 3D 오도메트리와 별개로 평가한다. 알고리즘 비교에는 궤적 오차·드리프트·중단·처리 지연을, 주행 비교에는 목표 도달률·시간·경로 길이·정지 및 재시도 횟수를 사용한다.

## 저장소의 다음 구현 단위

1. R1 v2의 실제 토픽·TF·장치 계약을 기록할 인터페이스 표와 Jazzy 패키지 뼈대를 만든다.
2. G6형 2D 스캔, 차동구동, 바퀴 오도메트리를 가진 R1 v2 시뮬레이션과 재생 가능한 bag을 만든다.
3. RViz에서 지도·스캔·TF·주행을 볼 수 있는 launch, SLAM Toolbox·AMCL·Nav2 설정을 추가한다.
4. 실측 footprint, 센서 위치, 모터 속도 제한, 드라이버 버전을 확인해 임시 설정을 교체한다.

실제 장비에서 얻지 않은 수치를 R1 v2의 확정 사양으로 표시하지 않는다.
