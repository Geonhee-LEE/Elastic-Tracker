# 01. 개요

## 1.1 무엇을 하는 프로젝트인가

카메라를 장착한 쿼드로터가 **움직이는 목표물을 장애물 환경에서 추적**하도록 만드는 ROS1 워크스페이스입니다.
드론은 목표물을 일정 거리에서 따라다니되, 세 가지를 동시에 만족해야 합니다.

- 장애물에 부딪히지 않을 것 (**safety**)
- 목표물이 장애물에 가려 시야에서 사라지지 않을 것 (**visibility**)
- 목표물의 속도 변화에 궤적이 즉각 반응할 것 (**elasticity**)

기존 추적 플래너는 보통 "목표물 뒤 N미터 지점"을 골(goal)로 삼고 일반 내비게이션 플래너를 돌립니다.
그러면 목표물이 갑자기 가속하거나 기둥 뒤로 돌아갈 때 시선(line of sight)이 끊기고, 한 번 놓치면
추적이 실패합니다. Elastic Tracker는 골 하나가 아니라 **미래 3초간의 목표물 궤적 전체**를 받아서,
그 시계열에 대해 거리 제약과 가시성 제약을 동시에 거는 방식으로 이 문제를 풉니다.

## 1.2 핵심 아이디어 3가지

### (1) 시간까지 최적화 변수로 — "탄성(elastic)"

궤적은 MINCO(Minimum Jerk, 5차 다항식 조각)로 표현합니다. 일반적인 플래너는 조각별 시간 배분을
고정하고 공간(제어점)만 최적화하지만, 여기서는 **각 조각의 지속 시간 `T_i`와 전체 시간 `sum_T`를
같이 최적화**합니다.

`src/planning/traj_opt/src/traj_opt.cc:167` `objectiveFunc()`의 결정 변수는 세 덩어리입니다.

```
x_ = [ t (dim_t_ = N_-1개) | p (dim_p_개) | deltaT (1개) ]
        ↓ forwardT              ↓ forwardP
    조각별 시간 T_i          코리도 내 제어점 P
```

전체 시간은 [traj_opt.cc:177](../src/planning/traj_opt/src/traj_opt.cc#L177)에서

```cpp
double sumT = obj.sum_T_ + deltaT * deltaT;   // sum_T_ = tracking_dur_ (기본 3.0s)
```

로 계산됩니다. `deltaT²`이므로 전체 시간은 **항상 `tracking_dur_` 이상**이고, 목표물이 멀어지면
`deltaT`가 커지며 궤적이 시간축으로 늘어납니다. 이것이 "elastic"의 의미입니다.

`t`는 `expC2`([traj_opt.cc:12](../src/planning/traj_opt/src/traj_opt.cc#L12))를 통해 항상 양수로
매핑된 뒤 정규화되므로, 시간 배분에 부등식 제약을 걸 필요가 없습니다(제약 없는 최적화로 환원).

### (2) 부채꼴 가시 영역 — "가시성(visibility)"

목표물 예측 위치마다 **그 지점에서 반경 `tracking_dist_`의 원을 훑어서 시야가 막히지 않는 각도 구간**
을 찾습니다([env.hpp:646](../src/planning/planning/include/env/env.hpp#L646) `visible_pair()`).
결과는 중심 `center`, 이등분 방향점 `visible_p`, 반각 `theta`로 표현되는 부채꼴입니다.

최적화에서는 드론 위치 `p`가 이 부채꼴 안에 있도록 코사인 제약을 겁니다
([traj_opt.cc:700](../src/planning/traj_opt/src/traj_opt.cc#L700)).

```
a = p - center,  b = vis_p - center
penalty = cos(theta - theta_clearance) - (a·b)/(|a||b|)   > 0 이면 위반
```

즉 "드론에서 목표물을 바라보는 방향이 가려지지 않는 각도 부채꼴 안에 있어야 한다"를
미분 가능한 페널티로 바꾼 것입니다. `theta_clearance`만큼 여유를 두어 경계에 붙지 않게 합니다.

### (3) 시간 스탬프가 붙은 추적 비용

`addTimeCost()`([traj_opt.cc:493](../src/planning/traj_opt/src/traj_opt.cc#L493))는 예측된 목표물
위치 `tracking_ps_[i]`를 **시각 `i * tracking_dt`의 궤적 위치와 짝지어** 비용을 겁니다.
"목표물 근처에 있어라"가 아니라 "**t초 뒤에 t초 뒤의 목표물 근처에 있어라**"입니다.

가중치는 시간이 갈수록 감소합니다.

```cpp
double rho = exp2(-3.0 * i / M);   // i=0에서 1.0 → i=M에서 0.125
```

먼 미래의 예측은 덜 믿는다는 뜻입니다. 또 `M = tracking_ps_.size() * 4 / 5`로
예측 구간의 뒤쪽 20%는 아예 쓰지 않습니다.

## 1.3 전체 파이프라인 한눈에

`plan_timer_callback()`([planning_nodelet.cpp:149](../src/planning/planning/src/planning_nodelet.cpp#L149))
이 20 Hz로 아래를 전부 다시 돕니다.

```
목표물 odom (EKF 출력)
   │
   ├─▶ ① 예측        prediction.hpp  : 목표물의 3초 뒤까지 궤적을 A*로 탐색
   │                                   (장애물 회피 + 가속도 최소)
   ├─▶ ② 가시 경로   env::findVisiblePath : 예측점마다 "보이는 지점"까지 A*
   │                                        을 순차 연결
   ├─▶ ③ 가시 영역   env::generate_visible_regions : 예측점마다 부채꼴 생성
   │
   ├─▶ ④ 안전 회랑   env::generateSFC : 경로를 볼록 다면체 열로 감싸기
   │                                    (DecompROS EllipsoidDecomp3D)
   ├─▶ ⑤ 최적화      TrajOpt::generate_traj : MINCO + L-BFGS
   │                                          (jerk + 시간 + 회랑 + 동역학
   │                                           + 추적거리 + 가시성)
   └─▶ ⑥ 충돌 검사   validcheck : 앞으로 1초를 0.01s 간격으로 점검
              │
              ▼
        quadrotor_msgs/PolyTraj 발행 → traj_server → so3_controller
```

## 1.4 두 가지 동작 모드

같은 노드가 파라미터로 세 갈래 동작을 합니다([planning_nodelet.cpp:764](../src/planning/planning/src/planning_nodelet.cpp#L764)).

| 파라미터 | 콜백 | 용도 |
|---|---|---|
| `debug=true` | `debug_timer_callback` | 저장된 `replan_state.bin`을 반복 재생하며 디버깅 |
| `fake=true` | `fake_timer_callback` | 목표물 역할 드론이 **자기 자신이** 골까지 날아가는 모드 |
| 둘 다 false | `plan_timer_callback` | 실제 추적/착륙 플래너 |

`fake` 모드는 `fake_target.launch` / `fake_car_target.launch`에서 "쫓기는 쪽" 드론을 움직이는 데
쓰입니다. 이 모드는 추적/가시성 비용 없이 골 지향 내비게이션만 수행하고,
별도 구현인 [traj_opt_fake.cc](../src/planning/traj_opt/src/traj_opt_fake.cc)를 사용합니다.

추적 모드 안에서도 `/land_triger` 수신 여부로 갈립니다.

- **추적**: 목표물 위 1m(`target_p.z() += 1.0`), 수평 거리 `tracking_dist_` 유지, 가시성 비용 ON
- **착륙**: 목표물 좌표계의 `land_p_` 오프셋 지점으로 수렴, 가시성 비용 OFF,
  `grad_cost_p_landing`이 3축 모두 `tolerance_d_` 안으로 밀어넣음

## 1.5 논문과 코드의 대응

| 논문 개념 | 코드 |
|---|---|
| Target motion prediction | [prediction.hpp](../src/planning/planning/include/prediction/prediction.hpp) `Predict::predict()` |
| Visible region generation | [env.hpp:646](../src/planning/planning/include/env/env.hpp#L646) `visible_pair()` |
| Safe flight corridor | [env.hpp:279](../src/planning/planning/include/env/env.hpp#L279) `generateSFC()` (DecompROS) |
| Spatio-temporal optimization | [traj_opt.cc:167](../src/planning/traj_opt/src/traj_opt.cc#L167) `objectiveFunc()` |
| MINCO trajectory class | [minco.hpp:161](../src/planning/traj_opt/include/traj_opt/minco.hpp#L161) `MinJerkOpt` |

## 1.6 다음 문서

- 노드가 실제로 어떻게 연결되는지 → [02. 시스템 아키텍처](02-architecture.md)
- 패키지별 역할 → [03. 패키지 레퍼런스](03-packages.md)
- 수식 수준의 상세 → [04. 알고리즘 심층 분석](04-algorithms.md)
