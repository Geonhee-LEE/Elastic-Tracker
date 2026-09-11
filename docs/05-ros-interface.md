# 05. ROS 인터페이스

## 5.1 커스텀 메시지

### `quadrotor_msgs/OccMap3d`

점유 격자 전체를 그대로 전송합니다.

```
Header  header
float32 resolution
int16   size_x, size_y, size_z        # 셀 개수 (2의 거듭제곱)
int16   offset_x, offset_y, offset_z  # 링버퍼 원점 인덱스
int8[]  data                          # infocc 버퍼: 1=점유, -1=자유, 0=미지
```

**크기 주의**: `data`는 `size_x·size_y·size_z` 바이트입니다.
로컬 맵 기본값(128×128×32)이면 **524,288바이트 = 512 KiB**. 이것이 깊이 콜백마다(≈30 Hz) 발행되므로
약 15 MB/s입니다. 같은 nodelet manager 안이라 프로세스 간 복사는 없지만 메시지 복사는 발생합니다.

`use_global_map=true`인 목표물 쪽 맵은 512×512×64 = **16 MiB**를 1 Hz로 발행합니다.

### `quadrotor_msgs/PolyTraj`

```
int16   drone_id
int32   traj_id            # 발행할 때마다 증가
time    start_time         # 이 시각을 t=0으로 샘플링
bool    hover              # true면 hover_p에 정지
float32 yaw                # 목표 yaw (절대값)
float32[] hover_p          # 3원소
uint8   order              # 항상 5
float32[] coef_x, coef_y, coef_z   # 6 × piece_num
float32[] duration                 # piece_num
```

계수는 `MinJerkOpt`의 `b` 행렬을 조각별로 펼친 것으로, **저차항부터** 들어갑니다
([planning_nodelet.cpp:95](../src/planning/planning/src/planning_nodelet.cpp#L95)).

`traj_server`가 이를 `CoefficientMat`으로 되돌릴 때 그대로 읽습니다
([traj_server.cpp:68](../src/planning/planning/src/traj_server.cpp#L68)).

> ⚠️ 계수가 `float32`입니다. 5차 다항식 계수는 크기 차이가 커서 `float32` 반올림이
> 궤적 끝단에서 수 mm 오차를 만들 수 있습니다. 실사용에서 문제된 적은 없지만 알아둘 만합니다.

### `quadrotor_msgs/ReplanState`

리플랜 한 사이클을 **완전히 재현**하기 위한 스냅샷입니다.

```
Header  header
int16   state              # 5.4절 표 참조
float64[] iniState         # 3×3 (p, v, a) column-major
nav_msgs/Odometry  target
quadrotor_msgs/OccMap3d occmap    # ← 맵 전체가 들어감
float64[] path
time    replan_stamp
```

`debug=true`로 띄우면 `wr_msg::readMsg`가 `debug/replan_state.bin`에서 이걸 읽어 반복 재생합니다
([planning_nodelet.cpp:767](../src/planning/planning/src/planning_nodelet.cpp#L767)).

### `object_detection_msgs/BoundingBoxes`

darknet_ros 호환 형식입니다. `target_ekf_node`가 `bounding_boxes.front()` 하나만 씁니다.

```
BoundingBox: float64 probability; int64 xmin,ymin,xmax,ymax; int16 id; string Class
```

---

## 5.2 노드별 토픽

### `planning/Nodelet`

이름은 모두 프라이빗(`~`)이며 런치에서 리맵합니다.

**구독**

| 토픽 (`~`) | 타입 | 리맵 예 | 용도 |
|---|---|---|---|
| `odom` | `nav_msgs/Odometry` | `odom` | 자기 상태 |
| `gridmap_inflate` | `quadrotor_msgs/OccMap3d` | `gridmap_inflate` | 점유 격자 |
| `target` | `nav_msgs/Odometry` | `/target_ekf_odom` | 목표물 상태 |
| `triger` | `geometry_msgs/PoseStamped` | `/triger` | 추적 시작 |
| `land_triger` | `geometry_msgs/PoseStamped` | `/land_triger` | 착륙 모드 전환 + 오프셋 |

`fake` 모드에서는 `triger`의 `position.x/y`가 골이 되고 z는 0.9로 고정됩니다
([planning_nodelet.cpp:107](../src/planning/planning/src/planning_nodelet.cpp#L107)).
추적 모드에서는 값이 무시되고 "시작 신호"로만 쓰입니다.

`land_triger`의 pose는 **목표물 좌표계 기준 착륙점 오프셋**입니다.
`sh_utils/land_triger.sh`는 전부 0(= 목표물 원점)을 보냅니다.

**발행**

| 토픽 (`~`) | 타입 | 주기 | 용도 |
|---|---|---|---|
| `heartbeat` | `std_msgs/Empty` | `plan_hz` | traj_server 워치독 |
| `trajectory` | `quadrotor_msgs/PolyTraj` | 리플랜 성공 시 | 실행 궤적 |
| `replanState` | `quadrotor_msgs/ReplanState` | 매 리플랜 | 디버그 로깅 |
| `polyhedra` | `decomp_ros_msgs/PolyhedronArray` | 매 리플랜 | RViz 회랑 |
| `gridmap_inflate` | `quadrotor_msgs/OccMap3d` | `debug` 모드만 | 저장된 맵 재생 |

**시각화 토픽** — `Visualization` 클래스가 토픽 이름을 처음 쓸 때 `advertise`를 지연 생성합니다
([visualization.hpp:121](../src/planning/planning/include/visualization/visualization.hpp#L121)).
따라서 아래 토픽은 코드에 문자열 리터럴로만 존재합니다.

| 토픽 | 타입 | 내용 |
|---|---|---|
| `~ray` | `Marker` (ARROW) | 드론→목표물 시선. 노랑=뚫림, 빨강=막힘 |
| `~car_predict` | `nav_msgs/Path` | 목표물 예측 궤적 |
| `~observable_margin` | `nav_msgs/Path` | 최종 예측점 주위 `tracking_dist` 원 |
| `~astar` | `nav_msgs/Path` | 가시 경로 탐색 결과 |
| `~corridor_path` | `nav_msgs/Path` | 회랑 생성 입력 경로 (debug 모드) |
| `~way_pts` | `PointCloud2` | 가시 웨이포인트 |
| `~visible_ps` | `PointCloud2` | 부채꼴 중심 방향점 |
| `~visible_region` | `Marker` (TRIANGLE_LIST) | 부채꼴 메시 |
| `~keyPts` | `Marker` (LINE_LIST) | 회랑 씨앗 선분 |
| `~traj` | `Marker` | 최적화된 궤적 |
| `~drone_vel`, `~target_vel` | `Marker` (ARROW) | 속도 (debug 모드) |
| `~check_pts`, `~invalid_pts` | `nav_msgs/Path` | 충돌 검사 샘플 (debug 모드) |

### `traj_server`

| 방향 | 토픽 (`~`) | 타입 |
|---|---|---|
| 구독 | `trajectory` | `quadrotor_msgs/PolyTraj` |
| 구독 | `heartbeat` | `std_msgs/Empty` |
| 발행 | `position_cmd` | `quadrotor_msgs/PositionCommand` |

100 Hz 타이머(`ros::Duration(0.01)`)로 궤적을 샘플링합니다.

### `mapping/Nodelet`

| 방향 | 토픽 (`~`) | 타입 | 조건 |
|---|---|---|---|
| 구독 | `depth` | `sensor_msgs/Image` | `use_global_map=false` |
| 구독 | `odom` | `nav_msgs/Odometry` | `use_global_map=false` |
| 구독 | `global_map` | `sensor_msgs/PointCloud2` | `use_global_map=true` |
| 구독 | `target` | `nav_msgs/Odometry` | `use_mask=true` |
| 발행 | `gridmap_inflate` | `quadrotor_msgs/OccMap3d` | 항상 |
| 발행 | `local_pointcloud` | `sensor_msgs/PointCloud2` | (코드상 주석 처리) |
| 발행 | `mask_cloud` | `sensor_msgs/PointCloud2` | (미사용) |

`depth`와 `odom`은 `ApproximateTime` 동기화(큐 100)입니다.

### `target_ekf_node` / `target_ekf_sim_node`

| 방향 | 토픽 (`~`) | 타입 (실기) | 타입 (시뮬) |
|---|---|---|---|
| 구독 | `yolo` | `object_detection_msgs/BoundingBoxes` | `nav_msgs/Odometry` |
| 구독 | `odom` | `nav_msgs/Odometry` | 동일 |
| 발행 | `target_odom` | `nav_msgs/Odometry` | 동일 |
| 발행 | `yolo_odom` | `nav_msgs/Odometry` | 동일 (원시 관측 시각화) |

### `so3_controller` / `so3_quadrotor`

```
so3_controller:  구독 ~position_cmd, ~odom, ~imu   →  발행 ~so3cmd
so3_quadrotor:   구독 ~so3cmd                       →  발행 ~odom, ~imu, ~vis
```

---

## 5.3 파라미터 전수 목록

### `planning/Nodelet` (플래너 + traj_opt + env + prediction이 공유)

읽는 주체가 여러 개인 파라미터가 있습니다. 아래 "읽는 곳" 열을 참고하세요.

| 파라미터 | 기본/예시 | 읽는 곳 | 의미 |
|---|---|---|---|
| `plan_hz` | 20 | nodelet | 리플랜 주파수 [Hz] |
| `debug` | false | nodelet | 저장된 상태 재생 모드 |
| `fake` | false | nodelet | 골 지향 내비게이션 모드 |
| `K` | 8 | traj_opt | 조각당 적분 샘플 수 |
| `vmax` | 3.0 | traj_opt | 최대 속도 [m/s] |
| `amax` | 6.0 | traj_opt | 최대 가속도 [m/s²] |
| `rhoT` | 100.0 | traj_opt | 시간 정규화 가중치 |
| `rhoP` | 10000.0 | traj_opt | 회랑 위반 가중치 |
| `rhoV` | 1000.0 | traj_opt | 속도 위반 가중치 |
| `rhoA` | 1000.0 | traj_opt | 가속도 위반 가중치 |
| `rhoTracking` | 1000.0 | traj_opt | 추적 거리 가중치 |
| `rhosVisibility` | 10000.0 | traj_opt | 가시성 가중치 (0이면 비활성) |
| `clearance_d` | 0.2 | traj_opt | 회랑 면에서의 여유 [m] |
| `theta_clearance` | 0.8 | traj_opt, **env** | 가시 영역 각도 여유 [rad] |
| `tracking_dist` | 2.5 | nodelet, traj_opt, **env** | 목표 추종 거리 [m] |
| `tolerance_d` | 0.3 | nodelet, traj_opt, **env** | 거리 허용 오차 [m] |
| `tracking_dur` | 3.0 | nodelet, traj_opt, **prediction** | 예측·궤적 시간 지평 [s] |
| `tracking_dt` | 0.2 | traj_opt, **prediction** | 예측 샘플 간격 [s] |
| `prediction/rho_a` | 1.0 | prediction | 예측 가속도 페널티 |
| `prediction/vmax` | 4.0 | prediction | 목표물 최대 속도 [m/s] |
| `prediction/car_z` | 1.0 | — | **선언만 되고 읽히지 않음** |
| `N` | — | — | **주석 처리됨** (`N_`은 회랑 수로 결정) |

> `theta_clearance`, `tracking_dist`, `tolerance_d`는 `TrajOpt`와 `Env`가 **각자** `getParam`으로
> 읽습니다. 두 값이 반드시 같아야 하는데 강제 장치가 없습니다.

### `mapping/Nodelet`

| 파라미터 | 기본/예시 | 의미 |
|---|---|---|
| `use_global_map` | false | true면 전역 점군을 그대로 격자화 |
| `resolution` | 0.15 | 셀 크기 [m] |
| `local_x/y/z` | 20 / 20 / 5 | 로컬 맵 크기 [m] (2의 거듭제곱으로 내림됨) |
| `x_length/y_length/z_length` | 42 / 40 / 5 | 전역 맵 크기 [m] (`use_global_map`일 때) |
| `inflate_size` | 1 | 팽창 반경 [셀] |
| `down_sample_factor` | 2 | 깊이 이미지 픽셀 스킵 |
| `depth_filter_tolerance` | 0.15 | 프레임 간 깊이 불일치 허용 [m] |
| `depth_filter_mindist` | 0.2 | 최소 유효 깊이 [m] |
| `depth_filter_margin` | 2 | 이미지 테두리 무시 폭 [px] |
| `p_min / p_max` | -199 / 220 | 로그 오즈 클램프 (≈0.12 / 0.90) |
| `p_hit / p_mis` | 62 / 62 | hit/miss 증감량 (≈0.65 / 0.35) |
| `p_occ` | 139 | 점유 판정 임계값 (≈0.80) |
| `p_def` | -199 | 초기값 |
| `use_mask` | false | 목표물 주변을 강제 free로 |
| `camera_rate` | 30.0 | (설정만, 미사용) |
| `camera_range` | 7.0 | 센서 최대 거리 [m] |
| `cam_width/height` | 640 / 480 | 이미지 크기 |
| `cam_fx/fy/cx/cy` | camera.yaml | 카메라 내부 파라미터 |
| `depth_scaling_factor` | 1000.0 | uint16 → m 변환 |
| `cam2body_R` | 9원소 행 우선 | 카메라→바디 회전 |
| `cam2body_p` | 3원소 | 카메라→바디 평행이동 |

### `target_ekf` 노드

| 파라미터 | 기본/예시 | 의미 |
|---|---|---|
| `ekf_rate` | 20 | 예측 스텝 주파수 [Hz] |
| `cam_fx/fy/cx/cy` | target_ekf/config/camera.yaml | 내부 파라미터 |
| `cam_width/height` | 640 / 480 | FOV 검사용 (sim 노드) |
| `cam2body_R`, `cam2body_p` | YAML | 외부 파라미터 |
| `pitch_thr` | 37 | 피치 임계값 [deg] (실기 노드에서는 주석 처리) |
| `check_fov` | true | FOV 밖 관측을 버릴지 (sim 노드) |

### 시뮬레이터

`so3_quadrotor.yaml`, `so3_controller.yaml`, `mockamap.yaml`은
[03. 패키지 레퍼런스 §3.6](03-packages.md#36-uav_simulator-계열)에 값을 정리해 두었습니다.

`pcl_render_node` 추가 파라미터:

| 파라미터 | 값 |
|---|---|
| `sensing_horizon` | 5.0 [m] |
| `sensing_rate` | 30.0 [Hz] |
| `estimation_rate` | 30.0 [Hz] |

> ⚠️ `pcl_render_node`의 `sensing_horizon`(5.0m)과 `mapping`의 `camera_range`(7.0m)가 다릅니다.
> 렌더러가 5m까지만 그리므로 실질 감지 거리는 5m입니다.

---

## 5.4 `ReplanState.state` 값

| 값 | 의미 |
|---|---|
| `0` | 리플랜 성공 |
| `1` | 리플랜 실패, force_hover 유지 |
| `2` | 비상 정지 |
| `3` | 리플랜 실패, 이전 궤적 계속 |
| `-1` | 호버링 (목표 상태 달성) |
| `-2` | 궤적 생성 실패 (예측/탐색/최적화) |

---

## 5.5 런치 파일 조합표

| 런치 | 띄우는 것 | 용도 |
|---|---|---|
| `mapping/rviz_sim.launch` | RViz만 | 시각화 |
| `planning/fake_target.launch` | mockamap + 목표물 드론(fake 모드) 일체 | 쫓기는 드론 |
| `planning/fake_car_target.launch` | 위와 동일 + 차량 메시, `init_z=0.8` | 쫓기는 차량 |
| `planning/simulation1.launch` | drone0 추적 스택 | 기본 추적 데모 |
| `planning/simulation2.launch` | drone0(가시성 ON) + drone1(OFF) | 비교 실험 |
| `planning/simulation_landing.launch` | drone0 + `check_fov=false` | 이동 차량 착륙 |
| `planning/planning.launch` | 플래너 + traj_server (인자 3개 필수) | 위 3개가 include |
| `planning/debug.launch` | 플래너 단독 (`debug=true`) | 저장 상태 재생 |
| `planning/log.launch` | `play_bag_node` | replanState 기록 |
| `mapping/mapping.launch` | uav_simulator + mapping | simulation*.launch가 include |
| `mapping/run.launch` | RealSense D435 실기 스택 | 실제 비행 |
| `uav_simulator/uav_simulator.launch` | 동역학 + 제어 + 깊이 렌더러 | mapping.launch가 include |

`planning.launch`는 인자 3개가 **필수**입니다.

```xml
<arg name="rhosVisibility_"/>
<arg name="rhoTracking_"/>
<arg name="target_name_"/>
```

## 5.6 다음 문서

- 빌드와 실행 → [06. 빌드 & 실행](06-build-run.md)
