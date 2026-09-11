# 02. 시스템 아키텍처

## 2.1 노드 구성

`simulation1.launch` 기준으로 실제로 뜨는 노드는 다음과 같습니다. 대부분이 하나의 nodelet manager에
로드되는 **nodelet**이라 프로세스 간 복사 없이 동작합니다.

```
[전역, 한 번만]
  mockamap_node          ── /global_map (PointCloud2)  ※ fake_target.launch에서 실행

[ns=/target]  (쫓기는 드론)
  manager (nodelet manager)
  so3_quadrotor/Nodelet  ── 강체 동역학 시뮬레이터
  so3_controller/Nodelet ── SE(3) 기하 제어기
  mapping/Nodelet        ── use_global_map=true (전역 맵을 그대로 격자화)
  planning/Nodelet       ── fake=true (골 지향 내비게이션)
  traj_server
  odom_visualization

[ns=/drone0]  (쫓는 드론 = Elastic Tracker)
  manager
  so3_quadrotor/Nodelet
  so3_controller/Nodelet
  pcl_render_node        ── CUDA 깊이 이미지 렌더러 (가상 카메라)
  mapping/Nodelet        ── 깊이 영상 → 로컬 점유 격자
  mapping_vis_node       ── 격자 → RViz 점군
  target_ekf_sim_node    ── 목표물 관측 → EKF
  planning/Nodelet       ── 본체 (추적 플래너)
  traj_server
  odom_visualization
```

`simulation2.launch`는 `/drone0`(가시성 ON, `rhosVisibility_=10000`)과
`/drone1`(가시성 OFF, `rhosVisibility_=0`) 두 벌을 동시에 띄워 비교합니다.

## 2.2 토픽 그래프 (drone0 기준)

```
                    /global_map (PointCloud2)
                          │
                          ▼
  odom ──────────▶ pcl_render_node ──▶ depth (Image)
    │                                     │
    │      ┌──────────────────────────────┘
    ▼      ▼
  ┌─────────────────┐
  │  mapping        │  ApproximateTime(depth, odom)
  │  Nodelet        │  → raycasting → 팽창 격자
  └────────┬────────┘
           │ gridmap_inflate (quadrotor_msgs/OccMap3d)
           ▼
  ┌─────────────────┐   ◀── odom (nav_msgs/Odometry)
  │  planning       │   ◀── /target_ekf_odom  (target)
  │  Nodelet        │   ◀── /triger, /land_triger (PoseStamped)
  └────┬───────┬────┘
       │       │ replanState (ReplanState)  ── 디버그 기록용
       │       │ polyhedra (PolyhedronArray) ── RViz 회랑
       │       └─ 각종 visualization_msgs/Marker
       │ trajectory (PolyTraj) + heartbeat (Empty)
       ▼
  ┌─────────────────┐
  │  traj_server    │  100 Hz 샘플링
  └────────┬────────┘
           │ position_cmd (PositionCommand)
           ▼
  ┌─────────────────┐  ◀── odom, imu
  │ so3_controller  │
  └────────┬────────┘
           │ so3cmd (SO3Command)
           ▼
  ┌─────────────────┐
  │ so3_quadrotor   │ ──▶ odom (400 Hz), imu
  └─────────────────┘


  /target/odom ──┐
                 ├──▶ target_ekf_sim_node ──▶ /target_ekf_odom
  odom (drone0) ─┘         (ApproximateTime 동기화)
```

시뮬레이션에서는 YOLO 대신 목표물의 참값 odom(`/target/odom`)을 "관측"으로 넣습니다
(`simulation1.launch`의 `<remap from="~yolo" to="/target/odom"/>`). 실제 비행에서는
`target_ekf_node`가 `object_detection_msgs/BoundingBoxes`를 받습니다.

## 2.3 데이터 플로우 상세

### 맵 파이프라인

1. `pcl_render_node`(local_sensing)가 `/global_map`과 드론 odom으로 **깊이 이미지를 GPU 렌더링**
   (`sensing_horizon=5.0`, `sensing_rate=30`).
2. `mapping/Nodelet`이 `depth`와 `odom`을 `ApproximateTime`으로 동기화
   ([mapping_nodelet.cpp:301](../src/mapping/src/mapping_nodelet.cpp#L301)).
3. 깊이 → 카메라 좌표 → 월드 좌표 역투영. 이때 **깊이 필터**가 직전 프레임으로 재투영해
   `depth_filter_tolerance`(0.15m) 이상 튀는 점을 버립니다
   ([mapping_nodelet.cpp:128](../src/mapping/src/mapping_nodelet.cpp#L128)).
4. `OccGridMap::updateMap()`이 링버퍼를 드론 중심으로 이동시키고, 각 점에 대해 hit/miss +
   레이캐스팅으로 로그 오즈를 갱신([mapping.cc:7](../src/mapping/src/mapping.cc#L7)).
5. 팽창(inflate)된 결과 `infocc`를 통째로 `OccMap3d`로 발행.

### 목표물 파이프라인

`target_ekf_sim_node`는 9상태 EKF입니다(위치 3 + 속도 3 + RPY 3,
[target_ekf.hpp](../src/detection/target_ekf/include/target_ekf/target_ekf.hpp)).

- `predict()`는 `ekf_rate`(기본 20 Hz) 타이머로 등속 모델 전파
- `update()`는 관측이 들어올 때. **속도가 `vmax=4`를 넘는 갱신은 기각**하고,
  각도는 ±π 랩어라운드를 보정
- 마지막 갱신 후 5초가 지나면 `reset()`, 2초가 지나면 예측조차 멈추고 발행 중단
  ([target_ekf_sim_node.cpp](../src/detection/target_ekf/src/target_ekf_sim_node.cpp))

실제 카메라용 `target_ekf_node`는 6상태(위치+속도)이며, 바운딩 박스 높이로부터 거리를 추정합니다.

```cpp
double depth = 0.7 / height * fy_;   // 목표물 실제 높이를 0.7m로 가정
```

([target_ekf_node.cpp:153](../src/detection/target_ekf/src/target_ekf_node.cpp#L153))

## 2.4 리플랜 상태 머신

`plan_timer_callback`은 매 주기 다음 순서로 게이트를 통과합니다.

```
① heartbeat 발행 (무조건)
②  odom_received_ && map_received_ ?           → no: return
③  triger_received_ ?                          → no: return
④  target_received_ ?                          → no: return
⑤  force_hover_ && |v| > 0.1 ?                 → yes: return  (감속 대기)
⑥  [착륙] 목표물과 0.1m 이내 & 양쪽 저속?      → yes: 호버 발행, return
   [추적] |Δp| - tracking_dist < tolerance_d
          && |v_drone|<0.1 && |v_target|<0.2
          && |Δyaw| < 0.5 ?                     → yes: 호버 발행, state=-1, return
⑦ 맵 갱신 → 예측 → 경로 → 회랑 → 최적화
⑧ validcheck
```

`replanStateMsg_.state` 값의 의미:

| state | 상황 | 코드 위치 |
|---|---|---|
| `0` | 리플랜 성공, 새 궤적 발행 | [planning_nodelet.cpp:392](../src/planning/planning/src/planning_nodelet.cpp#L392) |
| `1` | 리플랜 실패 + 이미 force_hover 상태 → 호버 유지 | [:412](../src/planning/planning/src/planning_nodelet.cpp#L412) |
| `2` | 비상 정지. 현재 위치에서 호버 명령 | [:417](../src/planning/planning/src/planning_nodelet.cpp#L417) |
| `3` | 리플랜 실패하지만 이전 궤적은 유효 → 이전 궤적 계속 | [:423](../src/planning/planning/src/planning_nodelet.cpp#L423) |
| `-1` | 호버링(목표 달성 상태) | [:225](../src/planning/planning/src/planning_nodelet.cpp#L225) |
| `-2` | 궤적 생성 실패(예측/탐색/최적화 중 하나가 false) | [:390](../src/planning/planning/src/planning_nodelet.cpp#L390) |

> ⚠️ 추적 경로의 state 2/3 분기 조건이 `fake` 경로와 반대입니다.
> 자세한 내용은 [07. 코드 리딩 노트](07-code-notes.md#71-추적-모드-emergency-stop-조건-반전)를 보세요.

### 리플랜 시작 상태 이어붙이기

새 궤적은 **현재 위치가 아니라 30ms 뒤 이전 궤적 위의 상태**에서 시작합니다
([planning_nodelet.cpp:262](../src/planning/planning/src/planning_nodelet.cpp#L262)).

```cpp
ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);
double replan_t = (replan_stamp - replan_stamp_).toSec();
if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
  iniState.col(0) = odom_p;  iniState.col(1) = odom_v;    // 호버에서 재시작
} else {
  iniState.col(0) = traj_poly_.getPos(replan_t);          // 이전 궤적에서 이어받기
  iniState.col(1) = traj_poly_.getVel(replan_t);
  iniState.col(2) = traj_poly_.getAcc(replan_t);
}
```

30ms는 계산 시간을 벌기 위한 마진입니다. `traj_server`는 `trajMsg.start_time`을 기준으로
샘플링하므로, 새 궤적이 늦게 도착해도 시간축이 어긋나지 않습니다.

## 2.5 안전 장치

| 장치 | 위치 | 동작 |
|---|---|---|
| heartbeat | traj_server [:118](../src/planning/planning/src/traj_server.cpp#L118) | 0.5초간 heartbeat 없으면 마지막 위치 정지 명령 |
| 궤적 이중화 | traj_server [:124](../src/planning/planning/src/traj_server.cpp#L124) | 새 궤적 실행 실패 시 `trajMsg_last_`로 폴백 |
| yaw 슬루 제한 | traj_server [:91](../src/planning/planning/src/traj_server.cpp#L91) | 한 스텝(10ms)당 yaw 변화 0.02 rad로 제한 |
| 궤적 충돌 검사 | planning [:731](../src/planning/planning/src/planning_nodelet.cpp#L731) | 앞으로 1초를 0.01s 간격으로 `isOccupied` 검사 |
| 경계 상태 클램프 | traj_opt [:275](../src/planning/traj_opt/src/traj_opt.cc#L275) | 시작/종료 v, a를 `vmax_`, `amax_`로 스케일 다운 |
| EKF 이상치 기각 | target_ekf.hpp | 갱신 결과 속도가 4 m/s 초과면 기각 |
| 탐색 시간/메모리 상한 | env.hpp `MAX_MEMORY`, `MAX_DURATION` | A* 노드 수 2^18, 0.2초 — 단, 시간 예산은 A* **내부에서 갱신되지 않아** 실제로는 호출 사이에서만 발동합니다 ([7.6](07-code-notes.md#76-a-시간-예산이-실제로는-발동하지-않음)) |

## 2.6 스레딩 모델

- nodelet manager는 `num_worker_threads=16`
- `planning::Nodelet::onInit()`은 **별도 스레드에서 `init()`을 실행**
  ([planning_nodelet.cpp:786](../src/planning/planning/src/planning_nodelet.cpp#L786)).
  `Env`와 `Predict` 생성자가 수십만~수백만 개의 `new Node`를 하기 때문에 onInit을 막지 않으려는 것.
- 콜백 간 공유 데이터는 `std::atomic_flag` 스핀락으로 보호합니다.

```cpp
while (odom_lock_.test_and_set());   // 스핀
odom_msg_ = *msgPtr;
odom_lock_.clear();
```

`odom_lock_`, `target_lock_`, `gridmap_lock_` 세 개가 있습니다. 콜백이 짧아서 스핀락으로 충분하다는
판단이지만, `gridmap_lock_`은 `from_msg()`(수십만 바이트 `std::vector` 복사)를 감싸고 있어
길게 잡힙니다.

## 2.7 좌표계

- 모든 플래닝은 `world` 프레임에서 이루어집니다.
- 카메라 → 바디 외부 파라미터는 YAML로 주입
  ([mapping/config/camera.yaml](../src/mapping/config/camera.yaml)).

```yaml
cam2body_R: [ 0.0,  0.0,  1.0,
             -1.0,  0.0,  0.0,
              0.0, -1.0,  0.0]
cam2body_p: [0.0, 0.0, 0.05]
```

즉 카메라 z축(광축)이 바디 x축(전방)을 향합니다.
- 격자 인덱스 ↔ 위치 변환은 원점 기준 floor
  ([mapping.h:117](../src/mapping/include/mapping/mapping.h#L117)).

```cpp
pos2idx(p) = floor(p / resolution)
idx2pos(id) = (id + 0.5) * resolution   // 셀 중심
```

## 2.8 다음 문서

- 각 패키지가 정확히 무엇을 하는지 → [03. 패키지 레퍼런스](03-packages.md)
- 토픽/파라미터 전수 목록 → [05. ROS 인터페이스](05-ros-interface.md)
