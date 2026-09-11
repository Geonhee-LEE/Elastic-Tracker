# 07. 코드 리딩 노트

코드를 읽으며 확인한 이슈, 성능 특성, 호출 체인을 정리합니다.
아래 이슈들은 **저장소 원본 코드를 읽어 확인한 것**이며, 이 문서 작업에서 코드를 수정하지는 않았습니다.

## 요약

| # | 이슈 | 위치 | 심각도 | 현재 영향 |
|---|---|---|---|---|
| [7.7](#77-노드-풀-경계-초과-접근) | 노드 풀 경계 초과 접근 | `env.hpp:66` | **높음** | 배열 밖 포인터 역참조. 탐색이 상한에 닿을 때만 발생 |
| [7.1](#71-추적-모드-emergency-stop-조건-반전) | EMERGENCY STOP 조건 반전 | `planning_nodelet.cpp:415` | **높음** | 이전 궤적이 충돌해도 계속 실행 |
| [7.8](#78-최적화-실패-시-x_-메모리-누수) | 최적화 실패 시 누수 | `traj_opt.cc:365, :416` | 중간 | 20 Hz 루프에서 실패가 이어지면 누적 |
| [7.6](#76-a-시간-예산이-실제로는-발동하지-않음) | A* 시간 예산 미작동 | `env.hpp:454-458` | 중간 | 단일 A*가 예산 없이 실행 → 실시간성 보장 없음 |
| [7.9](#79-링버퍼-이동-시-축-크기-혼동) | 링버퍼 축 크기 혼동 | `mapping.cc:26` | 중간 | z축 하강 시 확률 버퍼에 잔여값 |
| [7.2](#72-generatesfc의-current_poly-오참조) | `current_poly` 오참조 | `env.hpp:336` | 중간 | 수축 판정이 전 회랑에 일괄 적용. 후속 보정이 최악은 방지 |
| [7.3](#73-초기화되지-않은-반환-플래그) | 미초기화 `bool ret` | `traj_opt.cc:634, :675` | 낮음 | 형식상 미정의 동작. 비용·그래디언트는 0이라 실질 무해 |
| [7.5](#75-ekf-fov-검사의-widthheight-교차) | FOV 검사 width/height 교차 | `target_ekf_sim_node.cpp:82, :86` | 낮음 | `check_fov=false`가 기본이라 **비활성 경로** |
| [7.4](#74-착륙-호버-판정의-fabs-오용) | `fabs`가 bool을 감쌈 | `planning_nodelet.cpp:199` | 낮음 | 우연히 의도대로 동작. 잠재적 함정 |

7.6~7.9는 병렬 분석 세션(`elastic-tracker-main-45`)이 제보한 것을 이 세션에서 소스로 재확인한 항목입니다.

---

## 7.1 추적 모드 EMERGENCY STOP 조건 반전

**위치**: `src/planning/planning/src/planning_nodelet.cpp:415`

같은 로직이 두 콜백에 있는데 조건의 부호가 다릅니다.

```cpp
// plan_timer_callback (추적) — 415행
} else if (validcheck(traj_poly_, replan_stamp_)) {
  force_hover_ = true;
  ROS_FATAL("[planner] EMERGENCY STOP!!!");
  ...
} else {
  ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
  ...
}
```

```cpp
// fake_timer_callback (페이크) — 598행
} else if (!validcheck(traj_poly_, replan_stamp_)) {   // ← 부정 연산자
  force_hover_ = true;
  ROS_FATAL("[planner] EMERGENCY STOP!!!");
```

의미상 맞는 것은 페이크 쪽입니다. **이전 궤적이 무효일 때** 비상 정지해야 합니다.

추적 콜백의 현재 동작:

| 이전 궤적 상태 | 현재 동작 | 기대 동작 |
|---|---|---|
| 유효 (충돌 없음) | 비상 정지 | 이전 궤적 계속 실행 |
| 무효 (충돌) | 이전 궤적 계속 실행 | 비상 정지 |

`else` 분기의 로그 메시지("EXECUTE LAST TRAJ")와 실제 상황이 정반대로 뒤집혀 있습니다.
새 궤적 생성이 실패했는데 이전 궤적이 이미 장애물을 관통하는 경우, 그 궤적을 계속 실행하게 됩니다.

리플랜이 20 Hz로 돌고 대부분의 사이클이 성공하기 때문에 실사용에서 드러나기 어렵지만,
안전 로직의 최후 방어선이라는 점에서 영향이 큽니다.

---

## 7.2 generateSFC의 current_poly 오참조

**위치**: `src/planning/planning/include/env/env.hpp:336-343`

```cpp
Eigen::MatrixXd current_poly;                     // 위 루프에서 마지막 값이 남아 있음
...
std::vector<int> inflate(hPolys.size(), 0);
for (int i = 0; i < (int)hPolys.size(); i++) {
  if (geoutils::findInteriorDist(current_poly, interior) < 0.1) {   // ← hPolys[i]가 아님
    inflate[i] = 1;
  } else {
    compressPoly(hPolys[i], 0.1);
  }
}
```

판정 대상이 루프 변수 `hPolys[i]`가 아니라 직전 루프에서 남은 `current_poly`(마지막 다면체)입니다.
게다가 `filterCorridor(hPolys)`가 그 사이에 실행되어 `hPolys`의 내용이 바뀌었으므로,
`current_poly`는 `hPolys`의 어떤 원소와도 대응이 보장되지 않습니다.

결과적으로 조건이 **모든 `i`에 대해 동일하게** 평가됩니다.

- 마지막 다면체가 넉넉하면 → 전부 0.1 수축
- 마지막 다면체가 좁으면 → 전부 수축 없이 `inflate[i] = 1`

의도는 "좁은 다면체는 수축하지 않는다"였을 것입니다. 다행히 바로 아래 연결성 보정 루프(`:344-357`)가
인접 다면체 교집합이 비면 되부풀리므로, 최악의 경우(교집합 소멸)는 방지됩니다.

---

## 7.3 초기화되지 않은 반환 플래그

**위치**: `traj_opt.cc:634` (`grad_cost_p_tracking`), `traj_opt.cc:675` (`grad_cost_p_landing`)

```cpp
bool ret;              // ← 초기화 없음
gradp.setZero();
costp = 0;

double pen = dr2 - upper;
if (pen > 0) { ...; ret = true; }
else { pen = lower - dr2; if (pen > 0) { ...; ret = true; } }
pen = dz2 - tolerance_d_ * tolerance_d_;
if (pen > 0) { ...; ret = true; }
...
return ret;            // 아무 조건도 안 걸리면 미정의 값
```

세 조건이 모두 거짓이면 — **추적 거리와 고도가 모두 허용 범위 안일 때, 즉 정상 상태** —
초기화되지 않은 값을 반환합니다. 형식적으로 미정의 동작입니다.

실제 영향은 제한적입니다. `costp`와 `gradp`는 0으로 초기화되어 있어서,
`ret`이 우연히 `true`가 되어도 `addTimeCost`가 0을 더할 뿐입니다.

```cpp
// traj_opt.cc:536-542
if (grad_cost_p_tracking(pos, target_p, grad_tmp, cost_tmp)) {
  gradViolaPc = beta0 * grad_tmp.transpose();     // grad_tmp == 0
  cost += rho * step * cost_tmp;                  // cost_tmp == 0
  jerkOpt_.gdC.block<6,3>(piece*6, 0) += rho * step * gradViolaPc;   // += 0
```

비교 대상인 `grad_cost_v`(`:727`), `grad_cost_a`(`:739`), `grad_cost_visibility`(`:700`)는
모두 명시적으로 `return false`합니다. 이 두 함수만 패턴이 다릅니다.
`bool ret = false;`로 바꾸는 것이 맞습니다.

---

## 7.4 착륙 호버 판정의 fabs 오용

**위치**: `planning_nodelet.cpp:199`

```cpp
if (std::fabs((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 && target_v.norm() < 0.2)) {
```

괄호가 `fabs(...)` **안쪽**을 전부 감싸고 있어, `fabs`의 인자가 `bool`입니다.
`fabs(true) = 1.0`, `fabs(false) = 0.0`이고 `if`는 이를 다시 참/거짓으로 해석하므로
**결과적으로는 의도대로 동작합니다**.

의도했던 형태는 이랬을 것입니다.

```cpp
if (std::fabs((target_p - odom_p).norm()) < 0.1 && odom_v.norm() < 0.1 && target_v.norm() < 0.2) {
```

`norm()`은 항상 음이 아니므로 `fabs` 자체가 불필요합니다. 동작 버그는 아니고 가독성 문제입니다.
`-Wall`이 켜져 있으므로(`traj_opt`) 컴파일러 경고로 잡힐 수 있는 패턴입니다.

---

## 7.5 EKF FOV 검사의 width/height 교차

**위치**: `src/detection/target_ekf/src/target_ekf_sim_node.cpp:80-88`

```cpp
double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
if (x < 0 || x > height_) {      // ← x(가로 픽셀)를 height_(480)와 비교
  return;
}
double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
if (y < 0 || y > width_) {       // ← y(세로 픽셀)를 width_(640)와 비교
  return;
}
```

`camera.yaml`이 `cam_width: 640`, `cam_height: 480`이므로 두 검사가 서로 바뀌었습니다.
가로 좌표는 480까지만 허용되고(우측 160픽셀 손실), 세로 좌표는 640까지 허용됩니다(사실상 무제한).

영향은 `check_fov=true`일 때만 나타납니다. 저장소의 launch 파일들은
`simulation_landing.launch`에서 명시적으로 `false`를 주고, 나머지는 설정하지 않아
기본값 `false`(`target_ekf_sim_node.cpp:22`)를 씁니다. 즉 **기본 시나리오에서는 이 코드가 실행되지 않습니다.**

---

## 7.6 A* 시간 예산이 실제로는 발동하지 않음

**위치**: `src/planning/planning/include/env/env.hpp:454-458`

```cpp
double t_cost = (ros::Time::now() - t_start_).toSec();   // ← 루프 진입 전 1회만 계산
if (t_cost > MAX_DURATION) {
  std::cout << "[env] search costs more than " << MAX_DURATION << "s!" << std::endl;
}
while (visited_nodes_.size() < MAX_MEMORY && t_cost <= MAX_DURATION) {
  ...                                                     // 루프 안에서 t_cost 갱신 없음
}
```

`t_cost`는 while 루프의 조건에 들어가 있지만 **루프 본문 어디에서도 갱신되지 않습니다.**
따라서 한 번의 A* 탐색은 일단 시작하면 `MAX_DURATION`(0.2초)과 무관하게
`MAX_MEMORY`(262,144 노드)를 다 쓰거나 `open_set`이 빌 때까지 돕니다.

예산이 완전히 무의미한 것은 아닙니다. `t_start_`는 바깥 `findVisiblePath`(`:513`)에서 한 번만
찍히고, 체인의 각 A*가 진입 시점에 `t_cost`를 새로 계산하므로 **A* 호출 사이사이에는 검사가 걸립니다.**

| 검사 지점 | 동작 |
|---|---|
| 16회 A* 사이 | 누적 0.2초를 넘겼으면 다음 A*가 즉시 종료 |
| 한 A* 내부 | 검사 없음 — 메모리 소진까지 무제한 |

즉 최악의 경우는 "16 × 0.2초"가 아니라 "0.2초 + 단일 A*의 최대 소요 시간"입니다.
20 Hz(50 ms) 예산을 지키려면 루프 안에서 주기적으로(예: 1024 노드마다) `t_cost`를 다시 재야 합니다.

---

## 7.7 노드 풀 경계 초과 접근

**위치**: `src/planning/planning/include/env/env.hpp:63-66`

```cpp
inline NodePtr visit(const Eigen::Vector3i& idx) {
  auto iter = visited_nodes_.find(idx);
  if (iter == visited_nodes_.end()) {
    auto ptr = data_[visited_nodes_.size()];   // ← 경계 검사 없음
```

루프 가드는 `while (visited_nodes_.size() < MAX_MEMORY)`인데, **루프 본문 안에서 이웃 6개에
대해 `visit()`가 최대 6번 호출**됩니다. 각 호출이 새 인덱스면 크기가 1씩 늘어납니다.

```
size == MAX_MEMORY - 1 에서 루프 진입 (가드 통과)
  1번째 이웃 → data_[MAX_MEMORY - 1]   ✔ 유효, size = MAX_MEMORY
  2번째 이웃 → data_[MAX_MEMORY]       ✘ 배열 밖
  ...
  6번째 이웃 → data_[MAX_MEMORY + 4]   ✘
```

`data_`는 `NodePtr data_[MAX_MEMORY]`(`env.hpp:59`)로 고정 크기입니다.
최대 5개까지 범위 밖을 읽고, 그 쓰레기 포인터를 역참조해 `ptr->idx = idx` 등을 씁니다.

루프 하단의 `if (visited_nodes_.size() == MAX_MEMORY) { std::cout << "out of memory"; }`는
**출력만 하고 `break`하지 않습니다** — 다음 반복의 가드가 잡아 주지만 그때는 이미 초과 접근이 끝난 뒤입니다.

세 A* 함수(`:410`, `:536`, `:715`)가 모두 같은 `visit()`와 같은 패턴을 씁니다.
`visit()` 안에서 `if (visited_nodes_.size() >= MAX_MEMORY) return nullptr;`을 두고
호출부에서 확인하는 것이 최소 수정입니다.

---

## 7.8 최적화 실패 시 x_ 메모리 누수

**위치**: `traj_opt.cc:363-366`, `traj_opt.cc:414-417`

```cpp
x_ = new double[dim_p_ + dim_t_ + 1];      // :348 / :399
...
int opt_ret = optimize();
if (opt_ret < 0) {
  return false;                             // ← delete[] x_ 없이 반환
}
...
traj = jerkOpt_.getTraj();
delete[] x_;                                // 성공 경로에서만 해제
return true;
```

L-BFGS가 음수를 반환하면 `x_`가 해제되지 않습니다.
`generate_traj` 오버로드 두 개(추적 `:325`, 착륙 `:380`)에 같은 패턴이 있습니다.

한 번의 누수량은 `(dim_p_ + dim_t_ + 1) × 8`바이트로 수백 바이트 수준이지만,
**20 Hz로 도는 루프에서 최적화 실패가 지속되면 누적**됩니다.
`extractVs` 실패 경로(`:334`, `:385`)는 `x_` 할당 이전이라 문제없습니다.

`std::unique_ptr<double[]>`나 `std::vector<double>`로 바꾸면 근본적으로 해결됩니다.

---

## 7.9 링버퍼 이동 시 축 크기 혼동

**위치**: `src/mapping/src/mapping.cc:26`

```cpp
Eigen::Vector3i size_xyz(size_x, size_y, size_z);
for (int i = 0; i < 3; ++i) {
  if (move[i] >= 0) {
    from[i] = 0;                    from_small[i] = inflate_size;
    to[i]   = move[i];              to_small[i]   = move[i] + inflate_size;
  } else {
    from[i]       = move[i] + size_xyz[i];              // ✔ 축별 크기
    from_small[i] = move[i] + size_x - inflate_size;    // ✘ 항상 size_x
    to[i]         = size_xyz[i];                        // ✔
    to_small[i]   = size_xyz[i] - inflate_size;         // ✔
  }
}
```

`from_small`만 `size_xyz[i]` 대신 `size_x`를 씁니다. 같은 블록의 나머지 세 줄은 모두 축별 크기를 쓰므로
오타로 보입니다.

기본 설정 `local_x=20, local_y=20, local_z=5, resolution=0.15`에서 실제 격자는
`128 × 128 × 32`입니다. 따라서 **x·y축은 우연히 맞고, z축만 틀립니다.**

| 축 | `size_xyz[i]` | 쓰이는 값 | 결과 |
|---|---:|---:|---|
| x | 128 | 128 | 정상 |
| y | 128 | 128 | 우연히 정상 |
| z | 32 | 128 | `from_small.z()`가 96만큼 과대 |

영향받는 것은 `pro`(로그오드 확률) 버퍼를 `p_def`로 초기화하는 루프(`mapping.cc:77-85`)입니다.
드론이 **아래로 이동**(`move.z() < 0`)할 때 `from_small.z() > to_small.z()`가 되어
루프가 아예 돌지 않고, 새로 들어온 z 슬래브의 확률값이 이전 위치의 잔여값으로 남습니다.

`infocc`/`occ`를 지우는 루프(`:66-75`)는 `from`/`to`를 쓰므로 정상입니다.
즉 표시되는 점유 격자는 맞지만, 그 아래 확률 상태에 유령 값이 남아
이후 `hit`/`mis` 갱신이 잘못된 시작점에서 출발할 수 있습니다.

수정은 한 글자입니다 — `size_x` → `size_xyz[i]`.

---

## 7.10 페이크 모드 파라미터 미설정

`fake_target.launch` / `fake_car_target.launch`의 planning 노드렛은 다음을 설정하지 않습니다.

`rhoTracking`, `rhosVisibility`, `theta_clearance`, `tracking_dur`, `tracking_dt`,
`prediction/rho_a`, `prediction/vmax`.

`ros::NodeHandle::getParam`은 파라미터가 없으면 **인자를 건드리지 않고 false를 반환**하므로,
해당 멤버들이 초기화되지 않은 채 남습니다.

| 클래스 | 미초기화 멤버 | 사용 여부 |
|---|---|---|
| `TrajOpt` | `rhoTracking_`, `rhosVisibility_`, `theta_clearance_`, `tracking_dur_`, `tracking_dt_` | `traj_opt_fake.cc` 경로에서 미사용 |
| `Env` | `theta_clearance_` | `visible_pair()`에서만 사용 → 페이크 모드에서 미호출 |
| `Predict` | `pre_dur`, `dt`, `rho_a`, `vmax` | `predict()`에서 사용 |

세 번째 항목이 눈에 띕니다. `Predict` 생성자는 `tracking_dur`, `tracking_dt`,
`prediction/rho_a`, `prediction/vmax`를 읽는데(`prediction.hpp:44-48`), 페이크 모드 launch에
이들이 없습니다. 다만 `fake_timer_callback`은 `prePtr_->predict()`를 **호출하지 않으므로**
실제 문제는 발생하지 않습니다.

또 `Predict` 생성자는 이 시점에 이미 `MAX_MEMORY = 1 << 22`개의 `Node`를 `new`합니다.
페이크 모드에서 쓰지도 않을 메모리입니다 (7.13절 참고).

---

## 7.11 미사용 코드

| 항목 | 위치 | 비고 |
|---|---|---|
| `Predict::car_z` | `prediction.hpp:35` | 선언만 있고 읽히지 않음. `debug.launch`에만 파라미터 존재 |
| `pitch_thr_` (실기) | `target_ekf_node.cpp:127-134` | 피치 검사 블록 전체가 주석 처리 |
| `local_pc_pub_`, `pcl_pub_` | `mapping_nodelet.cpp:161-176` | advertise는 하지만 발행 코드가 주석 |
| `ReplanState.path`, `.replan_stamp` | `ReplanState.msg` | 메시지에 선언되어 있으나 채우는 코드 없음 |
| `TakeoffLand`, `Px4ctrlDebug` | `quadrotor_msgs/msg/` | 발행/구독하는 노드 없음 |
| `traj_opt.cc:583-596` | 이전 버전 `penF` | 주석 처리된 4차/선형 페널티 |
| `planning_nodelet.cpp:395-399` | 미지 영역 주시 로직 | 주석 처리 |
| 타이밍 계측 코드 | `planning_nodelet.cpp` 곳곳 | `t_path_`, `t_corridor_`, `t_optimization_` 전부 주석 |

마지막 항목이 유용합니다. 주석을 풀면 경로탐색 / 회랑생성 / 최적화의 평균 소요 시간이
콘솔에 출력됩니다. 성능 튜닝 시 첫 번째로 켜 볼 곳입니다.

---

## 7.12 중복 코드

### L-BFGS 두 벌

- `src/planning/traj_opt/include/traj_opt/lbfgs_raw.hpp` (1546행)
- `src/planning/planning/include/prediction/lbfgs.hpp` (1546행)

두 파일은 **바이트 단위로 동일**합니다 (`md5: 6ebd64df76a0a250a54d25f38373a4fa`).
그리고 `prediction/lbfgs.hpp`를 include하는 곳이 저장소 어디에도 없습니다 —
`prediction.hpp`는 격자 탐색만 하고 L-BFGS를 쓰지 않습니다. **완전히 죽은 파일**입니다.

### A* 세 벌

`env.hpp`의 `findVisiblePath`(`:410`), `astar_search`(`:536`), `short_astar`(`:715`)는
`stopCondition`과 `calulateHeuristic` 람다만 다르고 본체 60여 행이 동일합니다.
람다를 인자로 받는 하나의 템플릿 함수로 합칠 수 있습니다.

### checkRayValid 두 벌

`env.hpp:94`(거리 제한 있음)와 `:128`(없음)이 거의 같습니다.
전자를 `max_dist = infinity`로 호출하면 후자를 대체할 수 있습니다.

### EKF 두 벌

`target_ekf.hpp`의 9상태 `Ekf`와 `target_ekf_node.cpp:21`에 인라인으로 박힌 6상태 `Ekf`.
후자는 헤더를 include하지 않고 자체 정의를 씁니다.

---

## 7.13 메모리 특성

### 정적 노드 풀

| 클래스 | 상수 | 노드 수 | `Node` 크기 | 대략 총량 |
|---|---|---:|---:|---:|
| `env::Env` | `MAX_MEMORY = 1 << 18` | 262,144 | 48 B | ~12 MB |
| `prediction::Predict` | `MAX_MEMORY = 1 << 22` | 4,194,304 | 96 B | **~400 MB** |

`prediction::Node`는 `Eigen::Vector3d` 3개(72 B) + `double` 3개 + 포인터입니다.
생성자에서 개별 `new`로 419만 번 할당하므로, 할당자 오버헤드까지 더하면 실제 사용량은 더 큽니다.

```cpp
// prediction.hpp:45-47
for (int i = 0; i < MAX_MEMORY; ++i) {
  data[i] = new Node;     // 419만 번의 개별 할당
}
```

노드렛 초기화가 눈에 띄게 느린 주 원인입니다.
`Predict`에는 소멸자가 없어 이 메모리는 **프로세스 종료까지 해제되지 않습니다**
(`Env`는 소멸자에서 해제합니다, `env.hpp:85-89`).

실제 탐색이 소비하는 노드 수는 `pre_dur/dt = 15` 깊이에 브랜칭 9이므로 이론적으로 크지만,
장애물과 속도 제약으로 대부분 가지치기됩니다. 100 ms 시간 제한이 먼저 걸리는 것이 보통입니다.
`1 << 20` 정도로 줄여도 실용상 충분해 보입니다.

### 맵 메시지

`OccMap3d` 한 개 = `128 × 128 × 32 = 524,288` 바이트 (기본 설정).
20 Hz로 오가지만 nodelet manager 내부에서는 `boost::shared_ptr` 전달이라 복사가 없습니다.

단, `planning_nodelet.cpp:233`의 `gridmapPtr_->from_msg(map_msg_)`는
`infocc.data = msg.data`로 **벡터를 복사**합니다. 매 사이클 512 KB 복사가 발생합니다.
그리고 `prePtr_->setMap(*gridmapPtr_)`(`:237`)이 `OccGridMap`을 값으로 받아 **또 한 번 복사**합니다.

```cpp
// prediction.hpp:50-53
inline void setMap(const mapping::OccGridMap& _map) {
  map = _map;        // 링버퍼 4개 전체 복사
}
```

`Predict::map`은 `pro`, `occ`, `vis` 버퍼도 갖고 있지만 `from_msg`가 `infocc`만 채우므로
나머지는 빈 채로 복사됩니다.

---

## 7.14 호출 체인 요약

### 한 번의 추적 리플랜

```
plan_timer_callback                        planning_nodelet.cpp:149
├─ heartbeat_pub_.publish()                            :150
├─ [가드] odom/map/triger/target 수신 확인              :151-170
├─ [호버 판정] 거리·속도·yaw 4조건                      :218-231
├─ gridmapPtr_->from_msg(map_msg_)                     :233
├─ prePtr_->setMap(*gridmapPtr_)                       :237
│
├─ envPtr_->checkRayValid(odom_p, target_p)            :240   → env.hpp:128
├─ prePtr_->predict(target_p, target_v, predict)       :252   → prediction.hpp:56
│                                                              등가속도 격자 탐색, 3초
│
├─ [초기 상태] 이전 궤적에서 이어받기 or 정지 상태       :271-283
│
├─ envPtr_->findVisiblePath(p_start, predict, ...)     :303   → env.hpp:513
│    └─ 16 × findVisiblePath(idx, idx, path)                  → env.hpp:410
│         └─ rayValid()                                       → env.hpp:385
│
├─ envPtr_->generate_visible_regions(...)              :323   → env.hpp:690
│    └─ 16 × visible_pair()                                   → env.hpp:646
│         └─ 2 × O(π/dθ) checkRayValid()                      → env.hpp:128
│
├─ envPtr_->pts2path(way_pts, path)                    :339   → env.hpp:817
│    └─ short_astar() (가시선 끊긴 구간만)                     → env.hpp:715
│
├─ envPtr_->generateSFC(path, 2.0, hPolys, keyPts)     :347   → env.hpp:279
│    ├─ getPointCloudAroundLine()                             → env.hpp:172
│    ├─ EllipsoidDecomp3D::dilate()                           → DecompROS
│    ├─ filterCorridor()                                      → env.hpp:228
│    └─ compressPoly() / findInterior()                        → geoutils.hpp
│
├─ trajOptPtr_->generate_traj(...)                     :364   → traj_opt.cc:325
│    ├─ extractVs()                                           → traj_opt.cc:210
│    │    └─ geoutils::enumerateVs() → quickhull.hpp
│    ├─ setBoundConds()                                       → traj_opt.cc:275
│    │    ├─ backwardT() / backwardP()                        → :31 / :105
│    │    └─ jerkOpt_.reset()                                 → minco.hpp:184
│    ├─ optimize() → lbfgs_optimize()                         → traj_opt.cc:301
│    │    └─ objectiveFunc() (반복 최대 1000회)               → traj_opt.cc:167
│    │         ├─ forwardT() / forwardP()                     → :19 / :67
│    │         ├─ jerkOpt_.generate(P, T)                     → minco.hpp:199
│    │         ├─ getTrajJerkCost() / calGrads_CT()           → minco.hpp:360 / :279
│    │         ├─ addTimeIntPenalty()                         → traj_opt.cc:430
│    │         │    └─ grad_cost_p_corridor / _v / _a         → :560 / :727 / :739
│    │         ├─ addTimeCost()                               → traj_opt.cc:493
│    │         │    └─ grad_cost_p_tracking / _visibility     → :620 / :700
│    │         ├─ calGrads_PT()                               → minco.hpp:306
│    │         └─ addLayerTGrad() / addLayerPGrad()           → :38 / :141
│    └─ jerkOpt_.getTraj()                                    → minco.hpp:373
│
├─ validcheck(traj, replan_stamp)                      :385   → :731
│    └─ 0.01초 간격 × 1초 = 100회 isOccupied()
│
└─ pub_traj(traj, yaw, replan_stamp)                   :407   → :80
```

### 한 번의 맵 갱신

```
depth_odom_callback                        mapping_nodelet.cpp:78
├─ callback_lock_.test_and_set() 실패 시 프레임 드롭     :79
├─ cv_bridge::toCvCopy() (+ 32FC1 → 16UC1 변환)         :92-97
├─ [깊이 → 3D] down_sample_factor 간격 이중 루프         :110-142
│    └─ 이전 프레임 재투영 깊이 필터                      :126-138
├─ gridmap_.updateMap(cam_p, obs_pts)                   :143   → mapping.cc:7
│    ├─ vis.fillData(0)
│    ├─ [링버퍼 이동] x/y/z 슬래브 3회 초기화                  → mapping.cc:23-96
│    └─ for each point:
│         ├─ filter() → 범위/맵 클리핑                        → mapping.h:165
│         ├─ hit() or mis()                                   → mapping.h:218 / :226
│         │    └─ free2occ() / occ2free() (상태 전이 시만)     → mapping.h:196 / :207
│         └─ [레이캐스팅] 센서까지 mis() 표시                  → mapping.cc:109-140
├─ [use_mask] 목표물 주변 강제 자유화                    :146-160
└─ gridmap_.to_msg() → gridmap_inflate_pub_.publish()   :178-183
```

---

## 7.15 성능 관찰 포인트

측정 코드가 이미 있으므로(주석 처리) 튜닝 시 순서는 이렇습니다.

1. `planning_nodelet.cpp`의 `t_path_` / `t_corridor_` / `t_optimization_` 블록 주석 해제
2. 어느 단계가 20 Hz(50 ms) 예산을 먹는지 확인
3. 병목별 대응

| 병목 | 조정 대상 |
|---|---|
| 경로 탐색 | `env.hpp:52` `MAX_DURATION`, `resolution`(맵 해상도), `tracking_dur`(A* 횟수) |
| 가시 영역 | `visible_pair`의 `d_theta` — 현재 `resolution/desired_dist/2`. 2배로 늘리면 레이캐스팅 절반 |
| 회랑 생성 | `bbox_width`(현재 2.0) — 작을수록 다면체 수 증가, 클수록 점군 추출 비용 증가 |
| 최적화 | `K`(적분 샘플, 현재 8), `earlyExit`의 1000회 상한, `lbfgs_params.delta` |

`earlyExit`이 **시간이 아닌 반복 횟수** 기준이라는 점이 구조적 약점입니다
(`traj_opt.cc:202-212`). 실시간 보장이 필요하면 `k > 1e3` 대신 경과 시간 검사로 바꾸는 것이
직접적인 해법입니다.

---

## 7.16 확장 시 참고

### 목표물 예측을 바꾸고 싶다면

`prediction::Predict`를 교체하면 됩니다. 인터페이스는 두 개뿐입니다.

```cpp
void setMap(const mapping::OccGridMap&);
bool predict(const Vector3d& p, const Vector3d& v,
             std::vector<Vector3d>& out, const double& max_time = 0.1);
```

출력이 `tracking_dt` 등간격 점열이라는 것만 지키면 나머지 파이프라인은 그대로 동작합니다.
학습 기반 예측기를 끼우기 좋은 지점입니다.

### 가시성 모델을 바꾸고 싶다면

두 곳을 함께 손봐야 합니다.

1. `env.hpp:646` `visible_pair()` — 부채꼴 계산 (기하)
2. `traj_opt.cc:700` `grad_cost_visibility()` — 비용과 그래디언트 (해석)

둘이 같은 `(visible_p, theta)` 표현을 공유하므로, 표현을 바꾸면
`addTimeCost`(`:545`)의 호출부와 `TrajOpt` 멤버 `tracking_visible_ps_`, `tracking_thetas_`도 따라갑니다.

### 3D 가시성으로 확장하려면

현재 `visible_pair`는 z를 고정하고 xy 평면에서만 각도를 스캔합니다(`env.hpp:653-664`).
`prediction`도 z 가속을 0으로 두고 있습니다. 공중 목표물을 3D로 추적하려면
두 모듈 모두 고도 방향 탐색을 추가해야 합니다.

### 다중 드론

`simulation2.launch`가 이미 `drone0`/`drone1` 두 대를 띄우는 패턴을 보여 줍니다.
네임스페이스 분리 + `PolyTraj.drone_id` 필드(현재 사용되지 않음)를 활용하면
드론 간 궤적 공유로 확장할 수 있습니다. 다만 **드론 간 충돌 회피 비용은 구현되어 있지 않습니다.**
