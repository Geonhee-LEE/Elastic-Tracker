# 06. 빌드 & 실행

## 6.1 환경 요구사항

원 저장소가 검증한 조합입니다.

| 항목 | 값 |
|---|---|
| OS | Ubuntu 18.04 / 20.04 |
| ROS | Melodic / Noetic |
| C++ | C++14 (`add_compile_options(-std=c++14)`) |
| 빌드 타입 | `Release` + `-O3 -Wall` (CMakeLists에 하드코딩) |
| CUDA | `local_sensing`의 깊이 렌더러용 (선택) |

빌드 타입이 각 패키지 CMakeLists에 고정되어 있어 `catkin_make -DCMAKE_BUILD_TYPE=Debug`로도
`traj_opt`, `planning`은 Release로 빌드됩니다.

## 6.2 의존 패키지

`package.xml`을 종합하면 다음이 필요합니다.

```bash
sudo apt install \
  ros-${ROS_DISTRO}-nodelet \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-pcl-ros \
  ros-${ROS_DISTRO}-pcl-conversions \
  ros-${ROS_DISTRO}-message-filters \
  ros-${ROS_DISTRO}-dynamic-reconfigure \
  ros-${ROS_DISTRO}-tf \
  ros-${ROS_DISTRO}-rviz \
  libeigen3-dev
```

`mapping/launch/run.launch`(실기)만 `realsense2_camera`를 추가로 요구합니다.
시뮬레이션에는 필요 없습니다.

> `local_sensing/package.xml`에 `svo_msgs`, `vikit_ros` 의존이 남아 있습니다.
> 실제 소스에서는 쓰이지 않으므로(SVO 시절 잔재) 빌드가 막히면 해당 줄을 지워도 됩니다.

## 6.3 CUDA 설정 — 빌드 전 필수 확인

[src/uav_simulator/local_sensing/CMakeLists.txt:19](../src/uav_simulator/local_sensing/CMakeLists.txt#L19)

```cmake
set(ENABLE_CUDA true)
...
set(CUDA_NVCC_FLAGS
     -gencode arch=compute_61,code=sm_61;
)
```

`compute_61`은 **GTX 10xx 세대(Pascal)** 기준입니다. 다른 GPU를 쓰면 커널이 로드되지 않거나
빌드가 실패합니다.

| GPU 세대 | 대표 모델 | 아키텍처 |
|---|---|---|
| Maxwell | GTX 9xx | `compute_52,code=sm_52` |
| Pascal | GTX 10xx | `compute_61,code=sm_61` (기본값) |
| Volta | Titan V | `compute_70,code=sm_70` |
| Turing | RTX 20xx, GTX 16xx | `compute_75,code=sm_75` |
| Ampere | RTX 30xx | `compute_86,code=sm_86` |
| Ada | RTX 40xx | `compute_89,code=sm_89` |

확인 방법:

```bash
nvidia-smi --query-gpu=name,compute_cap --format=csv
```

**GPU가 없거나 CUDA를 쓰고 싶지 않다면** `ENABLE_CUDA`를 끕니다.

```cmake
set(ENABLE_CUDA false)
```

이 경우 `pcl_render_node` 대신 CPU 기반 `pointcloud_render_node`가 빌드됩니다.
`pointcloud_render_node`는 깊이 이미지가 아니라 **점군**을 내보내므로,
`mapping` 노드렛이 `depth` 토픽을 받지 못합니다.
`uav_simulator.launch`의 `pcl_render_node` 블록을 그에 맞게 고쳐야 합니다.

## 6.4 빌드

```bash
git clone https://github.com/ZJU-FAST-Lab/Elastic-Tracker.git
cd Elastic-Tracker
catkin_make
source devel/setup.bash      # zsh이면 setup.zsh
chmod +x sh_utils/*.sh
```

`src/CMakeLists.txt`는 `.gitignore`에 있으므로 `catkin_make`가 처음 실행될 때
`catkin/cmake/toplevel.cmake` 심볼릭 링크를 자동 생성합니다.

빌드 순서 의존성:

```
quadrotor_msgs, object_detection_msgs, decomp_ros_msgs   (메시지 생성)
        ↓
mapping, traj_opt, decomp_ros_utils, pose_utils
        ↓
planning, target_ekf, so3_*, local_sensing, odom_visualization
```

catkin이 `package.xml`을 보고 자동으로 정렬합니다.

## 6.5 시나리오 1 — 기본 공중 추적

터미널 4개가 필요합니다.

```bash
# 터미널 1 — RViz
roslaunch mapping rviz_sim.launch

# 터미널 2 — 목표물 드론 (전역 맵도 여기서 생성)
roslaunch planning fake_target.launch

# 터미널 3 — Elastic Tracker
roslaunch planning simulation1.launch

# 터미널 4 — 트리거
./sh_utils/pub_triger.sh
```

**동작 순서**

1. `fake_target.launch`가 `mockamap_node`로 `/global_map`을 만들고, 목표물 드론을 (2, 0, 2)에 띄웁니다.
2. `simulation1.launch`가 추적 드론 스택을 (0, 0, 2)에 띄웁니다.
3. `pub_triger.sh`가 `/triger`를 발행하면 **두 드론이 동시에 반응**합니다.
   - 목표물 드론(fake 모드): `/move_base_simple/goal`을 골로 삼아 비행
   - 추적 드론: 목표물 추적 시작
4. 목표물을 움직이려면 RViz의 **2D Nav Goal**로 `/move_base_simple/goal`을 찍습니다.

> `pub_triger.sh`가 보내는 pose는 전부 0입니다. 추적 드론에게는 값이 무시되고 시작 신호로만
> 쓰이지만, 목표물 드론(fake 모드)은 이 값을 골(0, 0, 0.9)로 받습니다.
> 실제로 목표물을 원하는 곳으로 보내려면 RViz의 2D Nav Goal을 쓰세요.

**RViz에서 볼 것**

| 표시 | 토픽 | 의미 |
|---|---|---|
| 노란/빨간 화살표 | `/drone0/planning/ray` | 시선. 빨강이면 가려짐 |
| 초록 다면체 | `/drone0/planning/polyhedra` | 안전 회랑 |
| 부채꼴 메시 | `/drone0/planning/visible_region` | 가시 영역 |
| 곡선 | `/drone0/planning/traj` | 최적화된 궤적 |
| 점선 경로 | `/drone0/planning/car_predict` | 목표물 예측 |

## 6.6 시나리오 2 — 가시성 유무 비교

```bash
roslaunch mapping rviz_sim.launch
roslaunch planning fake_target.launch
roslaunch planning simulation2.launch
./sh_utils/pub_triger.sh
```

`/drone0`(`rhosVisibility_=10000`)과 `/drone1`(`rhosVisibility_=0`)이 같은 목표물을 쫓습니다.
두 드론의 궤적을 비교하면 가시성 항이 만드는 우회 궤적을 볼 수 있습니다.

> 두 드론 모두 **초기 위치가 `uav_simulator.launch` 기본값 (0, 0, 2)로 같습니다**
> (`simulation2.launch`가 `init_x_` 등을 넘기지 않음). 시작할 때 겹쳐 보이지만
> 시뮬레이터가 서로 다른 네임스페이스에서 독립적으로 돌기 때문에 충돌 판정은 없습니다.

## 6.7 시나리오 3 — 이동 차량 착륙

```bash
roslaunch mapping rviz_sim.launch
roslaunch planning fake_car_target.launch      # 차량 메시, init_z=0.8
roslaunch planning simulation_landing.launch
./sh_utils/pub_triger.sh                       # ① 추적 시작
# 차량이 움직이는 것을 확인한 뒤
./sh_utils/land_triger.sh                      # ② 착륙 전환
```

`land_triger.sh`가 보내는 pose는 **목표물 좌표계 기준 착륙점 오프셋**입니다.
전부 0이면 차량 원점에 착륙합니다. 차량 뒤쪽 착륙판에 내리려면:

```bash
rostopic pub -1 /land_triger geometry_msgs/PoseStamped \
  '{pose: {position: {x: -0.3, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

착륙 모드에서 달라지는 것:

- `envPtr_->short_astar()` 사용 (가시 경로 탐색 대신 최단 경로)
- 가시성 비용 없음, `grad_cost_p_landing`이 3축 모두 `tolerance_d_` 안으로 수렴시킴
- yaw가 목표물 yaw를 따라감: `yaw = 2·atan2(q.z, q.w)`
- 종료 조건: `|Δp| < 0.1 && |v_drone| < 0.1 && |v_target| < 0.2`

## 6.8 시나리오 4 — 디버그 재생

리플랜 한 사이클을 저장해 두고 반복 재생합니다.

```bash
# ① 기록: 정상 시나리오를 돌리면서
roslaunch planning log.launch
# play_bag_node가 replanState를 구독해 debug/replan_state.bin에 저장

# ② 재생
roslaunch planning debug.launch
```

`debug.launch`는 `debug=true`로 `debug_timer_callback`을 돌립니다. 이 콜백은 저장된
`iniState`, `target`, `occmap`으로 **매 주기 같은 입력에 대해 전체 파이프라인을 다시 실행**하므로,
최적화 발산이나 회랑 생성 실패를 재현하기 좋습니다.

파일 경로는 하드코딩되어 있습니다
([planning_nodelet.cpp:767](../src/planning/planning/src/planning_nodelet.cpp#L767)).

```cpp
ros::package::getPath("planning") + "/../../../debug/replan_state.bin"
```

즉 **워크스페이스 루트의 `debug/` 디렉터리**입니다(`src/planning/planning` 기준 3단계 위).
`.gitignore`에 `debug`가 들어 있습니다. 없으면 직접 만드세요.

```bash
mkdir -p debug
```

## 6.9 파라미터 튜닝 가이드

### 추적이 너무 굼뜰 때

```xml
<param name="vmax" value="4.0"/>       <!-- 3.0 → 4.0 -->
<param name="amax" value="8.0"/>       <!-- 6.0 → 8.0 -->
<param name="rhoT" value="200.0"/>     <!-- 시간 벌점 ↑ = 더 공격적 -->
<param name="prediction/vmax" value="5.0"/>   <!-- 예측이 목표물 속도를 못 따라가면 -->
```

`prediction/vmax`가 실제 목표물 속도보다 작으면 예측 탐색이 `isValid`에서 전부 걸려
`"[prediction] no way!"`가 뜨고 리플랜이 실패합니다.

### 목표물을 자꾸 놓칠 때

```xml
<param name="rhosVisibility" value="20000.0"/>
<param name="theta_clearance" value="1.0"/>     <!-- 가시 영역 중앙 고수 -->
<param name="tracking_dist" value="2.0"/>       <!-- 더 가까이 -->
```

### 장애물에 스칠 때

```xml
<param name="clearance_d" value="0.4"/>    <!-- 회랑 안쪽 여유 ↑ -->
<param name="rhoP" value="50000.0"/>
<param name="inflate_size" value="2"/>     <!-- mapping 쪽 팽창 반경 ↑ -->
```

`inflate_size`를 늘리면 맵 갱신 비용이 `(2n+1)³`로 커집니다(`free2occ`/`occ2free`가 3중 루프).
`1 → 2`면 셀당 27 → 125회 갱신입니다.

### 리플랜이 자꾸 실패할 때

`replanState`의 `state`를 보면 어디서 실패하는지 알 수 있습니다.

```bash
rostopic echo /drone0/planning/replanState/state
```

- `-2`가 계속 나오면 → 예측 또는 경로 탐색 실패. 콘솔의
  `[prediction] no way!` / `[env] no way!` / `extractVs fail!` 메시지 확인
- `2`(비상 정지)가 자주 나오면 → 맵 갱신이 궤적보다 느리거나 `clearance_d`가 작음
- `3`이 자주 나오면 → 최적화 결과가 충돌 검사에서 걸림. `rhoP` 상향

## 6.10 트러블슈팅

| 증상 | 원인 | 조치 |
|---|---|---|
| `nvcc fatal: Unsupported gpu architecture` | GPU 아키텍처 불일치 | §6.3의 `-gencode` 수정 |
| 빌드는 되는데 depth가 안 나옴 | CUDA 커널이 GPU에서 로드 실패 | 같음. 또는 `ENABLE_CUDA false` |
| `Could not find svo_msgs` | package.xml 잔재 | `local_sensing/package.xml`에서 해당 줄 삭제 |
| `[planner] REPLAN FAILED` 반복 | 위 §6.9 참조 | `replanState` 확인 |
| 드론이 안 움직임 | 트리거 미수신 | `rostopic echo /triger`, `chmod +x sh_utils/*.sh` |
| `too long time no update!` | EKF에 관측이 안 들어옴 | `/target/odom` 발행 여부, FOV 확인 |
| RViz에 아무것도 없음 | `.rviz` 설정 파일 부재 | `mapping/config/rviz_sim.rviz`가 저장소에 없음. 직접 구성 필요 |
| 시작 직후 수 초간 멈춤 | `Predict` 생성자의 대량 할당 | 정상. [07 문서](07-code-notes.md#713-메모리-특성) 참조 |

> ⚠️ `rviz_sim.launch`는 `$(find mapping)/config/rviz_sim.rviz`를 참조하지만
> **저장소에 그 파일이 없습니다**(`config/`에는 `camera.yaml`, `D435.yaml`만 존재).
> RViz가 기본 설정으로 뜨므로 디스플레이를 수동으로 추가해야 합니다.

### RViz 수동 설정

```
Global Options → Fixed Frame: world

Add →
  PointCloud2   /drone0/mapping_vis/gridmap_inflate     (장애물)
  Marker        /drone0/planning/traj                    (궤적)
  Marker        /drone0/planning/ray                     (시선)
  Marker        /drone0/planning/visible_region          (가시 영역)
  PolyhedronArray  /drone0/planning/polyhedra            (회랑, DecompROS 플러그인)
  Path          /drone0/planning/car_predict             (예측)
  Path          /drone0/planning/astar                   (경로)
  MarkerArray   /drone0/odom_visualization/robot         (드론 메시)
  MarkerArray   /target/odom_visualization/robot         (목표물 메시)
```

`PolyhedronArray`는 `decomp_ros_utils`가 제공하는 RViz 플러그인입니다.
`source devel/setup.bash` 후에 RViz를 띄워야 목록에 나타납니다.

## 6.11 실기 적용 시 체크리스트

`mapping/launch/run.launch`가 출발점입니다.

1. **오도메트리** — `~odom`을 VIO/EKF 출력으로 리맵. `run.launch`는 `/ekf/ekf_odom` 기본
2. **카메라 캘리브레이션** — `mapping/config/D435.yaml`의 `cam_fx/fy/cx/cy`,
   `cam2body_R`, `cam2body_p`를 실제 값으로
3. **검출기** — `object_detection_msgs/BoundingBoxes`를 내는 노드(YOLO 등)를
   `target_ekf_node`의 `~yolo`에 연결
4. **목표물 크기** — [target_ekf_node.cpp:153](../src/detection/target_ekf/src/target_ekf_node.cpp#L153)의
   `0.7`은 목표물 실제 높이[m] 가정값입니다. 대상에 맞게 수정
5. **목표물 마스킹** — `use_mask=true`, `~target`을 EKF 출력에 연결.
   마스크 박스 크기(1×1×2m)는 [mapping_nodelet.cpp:155](../src/mapping/src/mapping_nodelet.cpp#L155)에
   하드코딩되어 있음
6. **동역학 한계** — `vmax`, `amax`를 기체 성능에 맞게 하향
7. **제어기** — `so3_controller` 대신 PX4 등을 쓴다면 `PositionCommand`를 받는 어댑터 필요

## 6.12 다음 문서

- 코드 구조와 발견한 이슈 → [07. 코드 리딩 노트](07-code-notes.md)
