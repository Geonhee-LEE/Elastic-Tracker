# 03. 패키지 레퍼런스

워크스페이스에는 `package.xml` 기준 19개 패키지가 있습니다. 이 중 DecompROS의 테스트용 더미
패키지(`foo`, `bar`, `baz`, `catkin_simple`)를 빼면 실질적으로 15개입니다.

```
src/
├── detection/          목표물 검출·추정
│   ├── object_detection_msgs   메시지 정의만
│   └── target_ekf              EKF 노드 2종
├── mapping/            점유 격자 매핑
├── planning/           ★ 본체
│   ├── planning                플래너 노드렛 + 환경/예측/시각화 헤더
│   ├── traj_opt                MINCO 최적화기
│   └── DecompROS               외부 라이브러리(벤더링)
├── quadrotor_msgs/     공용 메시지
└── uav_simulator/      시뮬레이터 일체
    ├── so3_quadrotor   동역학
    ├── so3_controller  제어기
    ├── local_sensing   CUDA 깊이 렌더러
    ├── mockamap        랜덤 맵 생성
    ├── odom_vis        RViz 메시 시각화
    ├── uav_utils       헤더 전용 유틸
    └── uav_simulator   런치·설정 모음(메타)
```

---

## 3.1 `planning` — 플래너 본체

**주요 파일**

| 파일 | 줄 수 | 역할 |
|---|---|---|
| [src/planning_nodelet.cpp](../src/planning/planning/src/planning_nodelet.cpp) | 795 | 리플래닝 루프 3종(추적/fake/디버그) |
| [include/env/env.hpp](../src/planning/planning/include/env/env.hpp) | 844 | A* 3종, 가시 영역, SFC 생성, 레이캐스팅 |
| [include/prediction/prediction.hpp](../src/planning/planning/include/prediction/prediction.hpp) | 128 | 목표물 운동 예측 |
| [include/visualization/visualization.hpp](../src/planning/planning/include/visualization/visualization.hpp) | 465 | RViz 마커 헬퍼 12종 |
| [include/wr_msg/wr_msg.hpp](../src/planning/planning/include/wr_msg/wr_msg.hpp) | 39 | ROS 메시지 ↔ 바이너리 파일 |
| [src/traj_server.cpp](../src/planning/planning/src/traj_server.cpp) | 153 | PolyTraj → PositionCommand 샘플러 |
| [src/play_bag_node.cpp](../src/planning/planning/src/play_bag_node.cpp) | 26 | 디버그용 bag 기록 |
| [src/test_node.cpp](../src/planning/planning/src/test_node.cpp) | 72 | 단독 테스트 |

**`env::Env`가 제공하는 것** (모두 헤더 전용, 인라인)

| 함수 | 줄 | 설명 |
|---|---|---|
| `checkRayValid(p0,p1[,max_dist])` | [94](../src/planning/planning/include/env/env.hpp#L94), [128](../src/planning/planning/include/env/env.hpp#L128) | 3D DDA 레이캐스팅으로 시선 차단 검사 |
| `rayValid(idx0,idx1)` | [385](../src/planning/planning/include/env/env.hpp#L385) | 인덱스 버전 (A* 정지 조건에서 사용) |
| `getPointCloudAroundLine()` | [172](../src/planning/planning/include/env/env.hpp#L172) | 선분 주변 점유 셀만 추출(SFC 입력) |
| `generateSFC(path, w, hPolys, keyPts)` | [279](../src/planning/planning/include/env/env.hpp#L279) | 경로 → 볼록 다면체 열 |
| `generateOneCorridor(line, w, hPoly)` | [254](../src/planning/planning/include/env/env.hpp#L254) | 단일 다면체(fake 모드 초기 속도 방향용) |
| `filterCorridor(hPolys)` | [228](../src/planning/planning/include/env/env.hpp#L228) | 앞뒤 다면체에 완전히 포함되는 중간 것 제거 |
| `compressPoly(poly, dx)` | [159](../src/planning/planning/include/env/env.hpp#L159) | 모든 초평면을 `dx`만큼 안/밖으로 이동 |
| `findVisiblePath(...)` | [410](../src/planning/planning/include/env/env.hpp#L410), [513](../src/planning/planning/include/env/env.hpp#L513) | 가시성 정지 조건 A* |
| `astar_search(...)` | [536](../src/planning/planning/include/env/env.hpp#L536), [632](../src/planning/planning/include/env/env.hpp#L632) | 거리 정지 조건 A* (fake 모드) |
| `short_astar(...)` | [715](../src/planning/planning/include/env/env.hpp#L715) | 정확히 목표 셀까지 A* (웨이포인트 연결) |
| `visible_pair(center, seed, vis_p, theta)` | [646](../src/planning/planning/include/env/env.hpp#L646) | 부채꼴 가시 영역 1개 |
| `generate_visible_regions(...)` | [690](../src/planning/planning/include/env/env.hpp#L690) | 위를 예측점 전체에 적용 |
| `pts2path(wayPts, path)` | [817](../src/planning/planning/include/env/env.hpp#L817) | 웨이포인트 사이를 직선/A*로 메꿔 조밀 경로 생성 |

세 A*는 **정지 조건과 휴리스틱만 다르고 본체가 거의 동일**합니다(각 ~100줄 중복).

**메모리**: `Env`는 생성자에서 `MAX_MEMORY = 1<<18` (262,144)개의 `Node`를 미리 할당하고
소멸자에서 해제합니다. 탐색 후에는 `visited_nodes_.clear()`만 하고 노드는 재사용합니다.

---

## 3.2 `traj_opt` — MINCO 최적화기

| 파일 | 줄 수 | 역할 |
|---|---|---|
| [src/traj_opt.cc](../src/planning/traj_opt/src/traj_opt.cc) | 751 | 추적/착륙용 시공간 최적화 |
| [src/traj_opt_fake.cc](../src/planning/traj_opt/src/traj_opt_fake.cc) | 271 | 골 지향 전용(추적·가시성 비용 없음) |
| [include/traj_opt/minco.hpp](../src/planning/traj_opt/include/traj_opt/minco.hpp) | 382 | `MinJerkOpt` + 밴드 행렬 LU |
| [include/traj_opt/poly_traj_utils.hpp](../src/planning/traj_opt/include/traj_opt/poly_traj_utils.hpp) | 580 | `Piece`, `Trajectory` 클래스 |
| [include/traj_opt/lbfgs_raw.hpp](../src/planning/traj_opt/include/traj_opt/lbfgs_raw.hpp) | 1546 | L-BFGS (Lewis-Overton 라인서치) |
| [include/traj_opt/geoutils.hpp](../src/planning/traj_opt/include/traj_opt/geoutils.hpp) | 152 | 다면체 내부점, 정점 열거 |
| [include/traj_opt/quickhull.hpp](../src/planning/traj_opt/include/traj_opt/quickhull.hpp) | 1696 | 볼록 껍질 (정점 열거용) |
| [include/traj_opt/sdlp.hpp](../src/planning/traj_opt/include/traj_opt/sdlp.hpp) | 666 | Seidel 선형계획법 (내부점 찾기) |
| [include/traj_opt/root_finder.hpp](../src/planning/traj_opt/include/traj_opt/root_finder.hpp) | 874 | 다항식 근 찾기 (최대 속도/가속도 계산) |

두 `.cc` 파일은 **같은 라이브러리 `traj_opt`에 함께 컴파일**됩니다
([CMakeLists.txt:27](../src/planning/traj_opt/CMakeLists.txt#L27)). 심볼이 충돌하지 않는 이유는

- `traj_opt_fake.cc`가 정의하는 `TrajOpt` 멤버는 **`generate_traj(iniState, finState, hPolys, traj)`
  오버로드 하나뿐**입니다([traj_opt_fake.cc:192](../src/planning/traj_opt/src/traj_opt_fake.cc#L192))
- 나머지 헬퍼(`forwardT`, `objectiveFunc`, `earlyExit` 등)는 전부 `static`이라 파일 내부 링키지

즉 fake 모드는 **자기만의 목적 함수와 L-BFGS 호출을 인라인으로 갖되**, 생성자·`extractVs`·
`addTimeIntPenalty`는 `traj_opt.cc` 쪽 정의를 그대로 씁니다.

두 최적화의 실질적 차이:

| | `traj_opt.cc` (추적/착륙) | `traj_opt_fake.cc` (골 지향) |
|---|---|---|
| 결정 변수 | `t`(N-1개), `p`, **`deltaT`** | `t`(**N개**), `p` |
| `forwardT` | 단체(simplex) 정규화 → `ΣT = sumT` | 원소별 `expC2`만 → 각 `T_i` 독립 |
| 전체 시간 | `tracking_dur_ + deltaT²` (하한 있음) | 자유 |
| 시간 비용 | `rhoT_ · deltaT²` | `rhoT_ · ΣT` |
| 추적/가시성 비용 | `addTimeCost()` 있음 | 없음 |
| 초기 시간 배분 | `sum_T_ / N_` 균등 | `\|Δp\| / vmax / N_` |

---

## 3.3 `mapping` — 점유 격자

| 파일 | 역할 |
|---|---|
| [include/mapping/mapping.h](../src/mapping/include/mapping/mapping.h) | `RingBuffer<T>`, `OccGridMap` 정의 |
| [src/mapping.cc](../src/mapping/src/mapping.cc) | `updateMap`, `occ2pc`, `inflate*` |
| [src/mapping_nodelet.cpp](../src/mapping/src/mapping_nodelet.cpp) | 깊이/전역맵 콜백, 깊이 필터, 마스킹 |
| [src/mapping_vis_node.cpp](../src/mapping/src/mapping_vis_node.cpp) | 격자 → PointCloud2 |
| [src/visualize_history_path.cpp](../src/mapping/src/visualize_history_path.cpp) | 비행 궤적 누적 시각화 |

**`OccGridMap` 내부 4개 버퍼**

| 버퍼 | 타입 | 의미 |
|---|---|---|
| `pro` | `int16` | 로그 오즈 확률. `p_min`~`p_max` 클램프 |
| `occ` | `uint16` | 팽창 카운터. 주변 셀이 팽창시킨 횟수 |
| `infocc` | `int8` | **발행되는 값**. `1`=점유, `-1`=자유, `0`=미지 |
| `vis` | `int8` | 한 프레임 안에서 중복 갱신 방지 플래그 |

`use_global_map=true`면 `pro`를 아예 만들지 않고 `setOcc` + `inflate`만 씁니다
([mapping.h:95](../src/mapping/include/mapping/mapping.h#L95)).

**링버퍼 트릭**: 크기를 2의 거듭제곱으로 내림해서 모듈로를 비트 AND로 대체합니다.

```cpp
size_x = exp2(int(log2(map_size.x() / res)));
inline const int idx2add(int x, int N) const {
  return (x & N) >= 0 ? (x & N) : (x & N) + N;   // N = size-1
}
```

`local_x=20, resolution=0.15`면 `20/0.15 = 133.3` → `log2 = 7.06` → `int = 7` → **128셀 = 19.2m**.
설정값보다 실제 맵이 작아지는 점을 유의하세요. z축은 `5/0.15=33.3` → **32셀 = 4.8m**입니다.

**목표물 마스킹**: `use_mask=true`면 목표물 주변 1×1×2m 박스를 강제로 free로 만들어
목표물 자신이 장애물로 인식되는 것을 막습니다
([mapping_nodelet.cpp:152](../src/mapping/src/mapping_nodelet.cpp#L152)).

---

## 3.4 `target_ekf` — 목표물 상태 추정

두 개의 실행 파일이 있습니다.

| 노드 | 상태 | 관측 | 용도 |
|---|---|---|---|
| `target_ekf_node` | 6 (p, v) | `BoundingBoxes` + odom | 실제 비행 (YOLO 연동) |
| `target_ekf_sim_node` | 9 (p, v, rpy) | `Odometry` + odom | 시뮬레이션 |

`target_ekf_sim_node`는 목표물 참값을 받지만 **카메라 FOV 밖이면 갱신을 버립니다**
(`check_fov` 파라미터). 착륙 시나리오에서는 목표물이 화면 하단으로 벗어나므로
`simulation_landing.launch`에서 `check_fov=false`로 끕니다.

EKF 공통 구조:

```
x_{k+1} = A x_k + B u,   u ~ N(0, Qt)        A: 등속 모델
z_k     = C x_k + v,     v ~ N(0, Rt)        C: 위치(+자세)만 관측
```

`Qt` 대각이 `[4, 4, 1, ...]`로 xy 프로세스 노이즈가 큽니다 — 목표물이 수평으로 기동한다는 가정입니다.

---

## 3.5 `quadrotor_msgs` / `object_detection_msgs`

메시지 정의만 있는 패키지입니다. 이 프로젝트에서 실제로 쓰이는 것:

- `OccMap3d` — 격자 전체를 `int8[]`로 전송
- `PolyTraj` — 5차 다항식 계수 + 조각 시간 + yaw + hover 플래그
- `ReplanState` — 리플랜 한 사이클의 입력 전체(맵 포함). 디버그 재현용
- `PositionCommand`, `SO3Command`, `AuxCommand` — 제어 체인
- `BoundingBoxes` / `BoundingBox` — YOLO 인터페이스 (darknet_ros 호환 필드)

`Px4ctrlDebug`, `TakeoffLand`, `CarPosition`, `motorAngle`, `ObjectCount`는
이 워크스페이스 안에서는 참조되지 않습니다(실기 스택에서 넘어온 잔재).

---

## 3.6 `uav_simulator` 계열

### `so3_quadrotor`
[quadrotor_dynamics.hpp](../src/uav_simulator/so3_quadrotor/include/so3_quadrotor/quadrotor_dynamics.hpp)
가 4차 룽게-쿠타로 강체+모터 1차 지연을 적분합니다. `simulation_rate=1000`, `odom_rate=400`.

```yaml
mass: 0.98,  Ixx/Iyy: 2.64e-3,  Izz: 4.96e-3
kf: 8.98132e-9,  arm_length: 0.26,  motor_time_constant: 0.03333
max_rpm: 35000,  min_rpm: 1200
```

### `so3_controller`
SE(3) 기하 제어기. `PositionCommand`의 p/v/a와 yaw를 받아 `SO3Command`(추력 벡터 + 목표 자세)를
냅니다. 게인은 [so3_controller.yaml](../src/uav_simulator/uav_simulator/config/so3_controller.yaml).

### `local_sensing`
CUDA로 전역 점군을 깊이 이미지로 렌더링합니다.
**`CMakeLists.txt:19`의 `-gencode arch=compute_61,code=sm_61`을 자기 GPU에 맞게 바꿔야 합니다.**
`ENABLE_CUDA false`로 두면 `pointcloud_render_node`(CPU, 점군 출력)로 대체됩니다.

### `mockamap`
Perlin 노이즈 기반 랜덤 맵 생성기. 기본 설정은 `type: 2`(박스 장애물).

```yaml
x_length: 42, y_length: 40, z_length: 5, resolution: 0.1
width_min: 0.5, width_max: 1.5, height_min: 3.5, height_max: 4.5
obstacle_number: 120, seed: 510
```

`seed`를 바꾸면 다른 맵이 나옵니다.

### `odom_vis`
`odom_visualization`(드론 메시)과 `odom_visualization_car`(차량 메시) 두 실행 파일.
`pose_utils`에 의존합니다.

### `uav_utils`
헤더 전용 유틸(각도 정규화, 좌표 변환 등). `package.xml`이 없어 catkin 패키지로 인식되지 않고,
`CMakeLists.txt`만 있습니다.

---

## 3.7 `DecompROS` (외부 라이브러리)

[sikang/DecompROS](https://github.com/sikang/DecompROS)를 벤더링한 것입니다.
이 프로젝트에서 쓰는 것은 사실상 두 가지입니다.

- `EllipsoidDecomp3D` — 선분 + 주변 점군 → 볼록 다면체
  ([env.hpp:280](../src/planning/planning/include/env/env.hpp#L280))
- `decomp_ros_msgs/PolyhedronArray` + RViz 플러그인 — 회랑 시각화

`decomp_ros_utils/include/` 아래 `decomp_basis`, `decomp_geometry`, `decomp_util`은
헤더 전용 코어입니다.

## 3.8 다음 문서

- 알고리즘 수식 → [04. 알고리즘 심층 분석](04-algorithms.md)
- 토픽·파라미터 표 → [05. ROS 인터페이스](05-ros-interface.md)
