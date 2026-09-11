# 04. 알고리즘 심층 분석

한 번의 리플랜(50ms 주기)에서 일어나는 다섯 단계를 순서대로 파헤칩니다.

```
① 목표물 예측 → ② 가시 경로 탐색 → ③ 가시 영역 생성 → ④ 안전 회랑 → ⑤ 시공간 최적화
```

---

## 4.1 목표물 운동 예측

**파일**: [prediction.hpp](../src/planning/planning/include/prediction/prediction.hpp)

목표물의 현재 위치·속도만으로 `tracking_dur_`(3초) 뒤까지의 궤적을 만듭니다.
단순 등속 외삽이 아니라 **장애물을 피하는 격자 탐색**입니다.

### 상태 격자와 입력

```cpp
for (input.x() = -3; input.x() <= 3; input.x() += 3)
  for (input.y() = -3; input.y() <= 3; input.y() += 3) {
    p = curPtr->p + curPtr->v * dt + input * dt²/2;
    v = curPtr->v + input * dt;
```

- 입력은 수평 가속도 `{-3, 0, 3} × {-3, 0, 3}` m/s²의 **9가지**
- z축 가속도는 항상 0 → 목표물의 수직 속도는 보존됩니다
- `dt = tracking_dt_` (0.2초)

### 비용 함수

```cpp
score(node) = rho_a * |a|                          // 가속도가 작을수록 좋음
h(node)     = 0.001 * |p - (target_p + target_v*T)|   // 등속 외삽 지점으로 유도
```

`h`의 계수가 0.001로 매우 작습니다. 즉 **"가능하면 등속으로 가되, 벽에 부딪히면 최소한의 가속으로
피한다"**는 정책입니다. 우선순위 큐 `score + h` 기준.

### 유효성

```cpp
bool isValid(p, v) { return (v.norm() < vmax) && (!map.isOccupied(p)); }
```

`prediction/vmax`(추적 시 4.0 m/s)를 넘거나 점유 셀에 들어가는 분기는 버립니다.

### 종료

`while (curPtr->t < pre_dur)` — 팝된 노드의 시각이 `pre_dur`에 도달하면 종료하고 부모 체인을
역추적합니다. `dt=0.2, pre_dur=3.0`이면 **16개 점**(t = 0.0 … 3.0)이 나옵니다.

실패 조건 두 가지:
- `stack_top == MAX_MEMORY` (2²² = 4,194,304 노드) → "out of memory"
- 경과 시간 > `max_time`(기본 0.1초) → "too slow"

두 경우 모두 `predict()`가 `false`를 반환하고, 플래너는 `state=-2`로 리플랜을 포기합니다.

---

## 4.2 가시 경로 탐색 — `findVisiblePath`

**파일**: [env.hpp:410](../src/planning/planning/include/env/env.hpp#L410)

예측된 목표물 위치 16개 각각에 대해 **"거기서 목표물이 보이는 드론 위치"**를 순차적으로 A*로 찾습니다.

```cpp
start_idx = 드론 현재 위치
for (target : target_predcit) {
    findVisiblePath(start_idx, pos2idx(target), idx_path);  // A* 1회
    start_idx = idx_path.back();          // 다음 탐색의 시작점 = 이번 탐색의 끝점
    way_pts.push_back(idx2pos(start_idx));  // 웨이포인트로 저장
}
```

**연쇄 구조가 핵심입니다.** i번째 웨이포인트에서 출발해 i+1번째 목표물 예측 위치를 볼 수 있는
곳까지 가므로, 결과 웨이포인트 열은 자연히 시간 순서를 따르는 연속 경로가 됩니다.

### 정지 조건 — 일반 A*와 다른 부분

```cpp
auto stopCondition = [&](const NodePtr& ptr) -> bool {
  return ptr->h < tolerance_d_ / mapPtr_->resolution && rayValid(ptr->idx, end_idx);
};
```

두 조건의 **AND**입니다.

1. `h`가 임계값 미만 — 뒤에서 보듯 `h`는 "목표 거리 링에서 얼마나 벗어났나"입니다
2. `rayValid` — 그 지점에서 목표물까지 시선이 뚫려 있어야 함

즉 목표물 셀에 도달하는 게 아니라 **"목표물 반경 `tracking_dist_` 근처이면서 목표물이 보이는 첫 셀"**
에서 멈춥니다.

### 휴리스틱 — 링 형태 목표

```cpp
Eigen::Vector3i dp = end_idx - ptr->idx;
double dr = dp.head(2).norm();            // 수평 거리 (격자 단위)
double lambda = 1 - stop_dist / dr;       // stop_dist = tracking_dist_ / resolution
double dx = lambda * dp.x();
double dy = lambda * dp.y();
ptr->h = fabs(dx) + fabs(dy) + abs(dz);
ptr->h += 0.001 * cross;                  // 타이브레이커
```

`lambda`는 "목표물까지의 벡터에서 `stop_dist`만큼을 뺀 비율"입니다.
`dr == stop_dist`면 `lambda = 0` → `h = |dz|` → 링 위에 정확히 있음.
`dr > stop_dist`면 링까지의 잔여 거리를, `dr < stop_dist`면 **음수 lambda로 바깥으로 밀어냅니다**
(너무 가까이 붙지 않게).

`cross`는 시작→끝 벡터와의 외적으로, 동점일 때 직선 경로를 선호하게 하는 관용적 타이브레이커입니다.

> ⚠️ `dr`이 0이면 `lambda`가 발산합니다. 목표물 바로 위 셀에서만 발생하며,
> 실제로는 `target_p.z() += 1.0` 때문에 발생하기 어렵습니다.

### 탐색 구조

- **6-연결** 격자(대각선 없음), 모든 간선 비용 1
- 노드 풀 재사용: `visit()`이 `data_[visited_nodes_.size()]`를 꺼내 씀
- 탐색 종료 시 `visited_nodes_.clear()`만 하고 노드 객체는 그대로 둠

---

## 4.3 가시 영역 생성 — `visible_pair`

**파일**: [env.hpp:646](../src/planning/planning/include/env/env.hpp#L646)

목표물 위치 `center`와 A*가 찾은 웨이포인트 `seed`를 받아, `center` 주변 반경 `desired_dist_`
원 위에서 **시야가 막히지 않는 연속 각도 구간**을 찾습니다.

```
        각도 스윕 (양방향)
              ↑ t_l
      ╭───────┼───────╮        반경 = tracking_dist_
     ╱   가시 영역     ╲
    │   ●center ───────┼── theta0 (seed 방향)
     ╲                 ╱
      ╰───────┼───────╯
              ↓ t_r
```

```cpp
double theta0 = atan2(dp.y(), dp.x());              // seed 방향
double d_theta = mapPtr_->resolution / desired_dist_ / 2;   // 각도 스텝(호 길이 ≈ 0.5셀)

for (t_l = theta0 - d_theta; t_l > theta0 - M_PI; t_l -= d_theta) {
  p = center + desired_dist_ * (cos t_l, sin t_l, 0);
  if (!checkRayValid(p, center)) { t_l += d_theta; break; }   // 막히면 직전 각도로 되돌림
}
// t_r도 대칭적으로 +방향 스윕
```

결과 세 값:

| 값 | 정의 | 용도 |
|---|---|---|
| `visible_p` | 중심각 `(t_l+t_r)/2` 방향의 원 위 점 | 최적화 제약의 기준 방향 `b` |
| `theta` | 반각 `(t_r-t_l)/2` | 허용 각도 반경 |
| `seed` (in/out) | 필요 시 안쪽으로 밀어넣은 웨이포인트 | 회랑 생성 경로 |

마지막에 `seed`를 보정합니다.

```cpp
double theta_c = min(theta, theta_clearance_);
if (theta0 - t_l < theta_c) { seed = 경계에서 theta_c만큼 안쪽; }
else if (t_r - theta0 < theta_c) { seed = 반대편 경계에서 theta_c만큼 안쪽; }
```

A*가 찾은 지점이 가시 영역 **경계에 붙어 있으면** 안쪽으로 밀어넣습니다. 경계에 붙은 채로
회랑을 만들면 최적화가 곧바로 가시성 제약을 위반하기 때문입니다.

> 이 함수는 **수평면(z 고정) 2D 스윕**입니다. 목표물보다 위에서 내려다보는 구성을 가정합니다.

### 각도 해상도 비용

`d_theta = resolution / desired_dist_ / 2` — `resolution=0.15, dist=2.5`면 `0.03 rad`.
최악의 경우 양방향 각각 π/0.03 ≈ 105회, 매회 `checkRayValid` 레이캐스팅.
예측점 15개면 최대 3,000회 레이캐스팅입니다. 대부분은 훨씬 일찍 막혀서 끝납니다.

---

## 4.4 안전 회랑 (SFC) 생성

**파일**: [env.hpp:279](../src/planning/planning/include/env/env.hpp#L279)

### 4.4.1 경로 조밀화 — `pts2path`

웨이포인트 열(드론 시작점 + 가시 웨이포인트들)을 연결된 경로로 바꿉니다.

```cpp
for (i : 0..M-2) {
  p0 = path.back();  p1 = wayPts[i+1];
  if (같은 셀) continue;
  if (!checkRayValid(p0, p1, 1.5)) {       // 1.5m 안에 직선으로 못 가면
    short_astar(p0, p1, short_path);       // A*로 우회
    path에 short_path 삽입;
  }
  path.push_back(p1);
}
```

`checkRayValid(p0, p1, 1.5)`의 세 번째 인자는 **최대 검사 거리**입니다. 1.5m를 넘어가면
그 이상은 검사하지 않고 유효로 판정합니다 — 가까운 장애물만 신경 씁니다.

### 4.4.2 회랑 씨앗 선분 고르기

```cpp
while (idx < path_len - 1) {
  next_idx = idx;
  // ① 직선으로 갈 수 있는 가장 먼 점까지 전진
  while (next_idx+1 < path_len && checkRayValid(path[idx], path[next_idx+1], bbox_width))
    next_idx++;

  // ② 그 선분으로 다면체 하나 생성
  line = {path[idx], path[next_idx]};
  keyPts.emplace_back(path[idx], path[next_idx]);
  getPointCloudAroundLine(line, maxWidth, obs_pc);
  decomp_util.set_obs(obs_pc);
  decomp_util.dilate(line);
  decompPolys.push_back(decomp_util.get_polyhedrons()[0]);

  // ③ 방금 만든 다면체 안에 들어오는 가장 먼 경로점으로 점프
  idx = next_idx;
  while (idx+1 < path_len && decompPolys.back().inside(path[idx+1])) idx++;
}
```

탐욕적으로 경로를 몇 개의 선분으로 쪼개고, 각 선분을 `EllipsoidDecomp3D`로 부풀려 볼록 다면체를
만듭니다. ③ 덕분에 다면체 개수가 필요 이상으로 늘지 않습니다.

`bbox_width = 2.0`으로 고정 호출되며(`generateSFC(path, 2.0, ...)`),
`maxWidth = 2.0 / 0.15 = 13`셀입니다.

### 4.4.3 장애물 점군 추출 — `getPointCloudAroundLine`

DecompROS에 넘길 장애물 점군을 **선분 주변 상자만** 훑어서 만듭니다
([env.hpp:172](../src/planning/planning/include/env/env.hpp#L172)).

1. 시작 셀 주위 `(2·maxWidth+1)³` 상자를 전부 검사
2. 선분을 따라 DDA로 한 셀씩 전진하면서, 진행 방향으로 `maxWidth`만큼 앞선 위치에
   **진행 방향에 수직인 (2·maxWidth+1)² 슬랩**만 추가 검사

즉 첫 상자는 전체를, 이후는 새로 들어오는 면만 스캔하는 슬라이딩 윈도우입니다.
같은 셀이 여러 번 들어갈 수 있지만(중복 제거 없음) DecompROS 쪽에서 문제되지 않습니다.

### 4.4.4 후처리

```cpp
filterCorridor(hPolys);          // ① 중복 다면체 제거
for each i: compressPoly(hPolys[i], 0.1);      // ② 0.1m 안쪽으로 수축(여유)
for i in 1..n-1:                 // ③ 인접 다면체 교집합 보장
  if (!findInterior([hPolys[i-1], hPolys[i]])) compressPoly(hPolys[i-1], -0.1);  // 팽창
```

**① `filterCorridor`** — `i-2`번째와 `i`번째 다면체의 교집합에 반경 1.0m 이상의 내부 공간이
있으면, 사이에 낀 `i-1`번째는 없어도 된다고 보고 건너뜁니다.

**③** 인접 다면체가 겹치지 않으면 MINCO 웨이포인트를 놓을 수 없으므로(`extractVs`가 실패),
한쪽을 0.1m 팽창시켜 강제로 겹치게 합니다. `inflate[]` 플래그로 같은 다면체를 두 번 팽창시키지
않습니다.

> ⚠️ ②의 조건문이 `hPolys[i]` 대신 루프 밖 변수 `current_poly`를 검사합니다.
> [07. 코드 리딩 노트](07-code-notes.md#72-generatesfc의-current_poly-오참조) 참고.

### 4.4.5 다면체 표현

`Eigen::MatrixXd hPoly`는 **6×(면 개수)** 행렬입니다.

```
col(j) = [ n_x, n_y, n_z, p_x, p_y, p_z ]ᵀ     n: 바깥 방향 법선, p: 면 위의 한 점
```

내부 조건은 `n · (x - p) ≤ 0`입니다.

---

## 4.5 시공간 최적화

**파일**: [traj_opt.cc](../src/planning/traj_opt/src/traj_opt.cc)

### 4.5.1 문제 설정

```cpp
N_ = 2 * cfgHs_.size();     // 회랑 하나당 궤적 조각 2개
sum_T_ = tracking_dur_;     // "NOTE wonderful trick"
dim_t_ = N_ - 1;
dim_p_ = Σ (cfgVs_[i].cols() - 1);
x_ = new double[dim_t_ + dim_p_ + 1];
```

결정 변수 벡터:

```
x_ = [  t₀ … t_{N-2}  |  p₀ … p_{dim_p-1}  |  deltaT  ]
       ── 시간 배분 ──   ── 웨이포인트 좌표 ──   전체 시간
```

**제약 없는(unconstrained) 문제**로 만든 것이 이 설계의 핵심입니다. L-BFGS는 제약을 다루지
못하므로, 제약을 변수 변환으로 흡수합니다.

### 4.5.2 시간 변수 변환 — `forwardT`

```cpp
static double expC2(double t) {
  return t > 0.0 ? ((0.5*t + 1.0)*t + 1.0)          // t>0:  0.5t² + t + 1
                 : 1.0 / ((0.5*t - 1.0)*t + 1.0);   // t≤0:  1/(0.5t² - t + 1)
}
```

`expC2`는 ℝ → (0, ∞)의 **C² 연속 전단사**입니다. `exp`보다 값이 완만하게 커져 수치적으로 안정적입니다
(t=0에서 값 1, 기울기 1로 두 조각이 매끄럽게 이어짐).

```cpp
static void forwardT(t, sT, vecT) {
  for (i) vecT(i) = expC2(t(i));       // 모두 양수
  vecT(M) = 0.0;
  vecT /= 1.0 + vecT.sum();            // 정규화
  vecT(M) = 1.0 - vecT.sum();          // 마지막 조각 = 나머지
  vecT *= sT;                          // 전체 시간으로 스케일
}
```

`N-1`개 자유 변수가 **합이 `sT`인 N개 양수**로 매핑됩니다(단체 simplex 파라미터화).
따라서 `T_i > 0`과 `ΣT_i = sT`가 자동으로 보장됩니다.

### 4.5.3 전체 시간의 탄성

```cpp
double sumT = obj.sum_T_ + deltaT * deltaT;    // ≥ tracking_dur_
...
grad[dim_t_+dim_p_] = obj.jerkOpt_.gdT.dot(T) / sumT + obj.rhoT_;
cost += obj.rhoT_ * deltaT * deltaT;
grad[dim_t_+dim_p_] *= 2 * deltaT;
```

- `T = sumT × (정규화된 배분)`이므로 `∂T_i/∂sumT = T_i/sumT` → `∂J/∂sumT = gdT·T/sumT`
- `rhoT_ * deltaT²`가 시간 정규화 항 (전체 시간이 길어지면 벌점)
- `∂sumT/∂deltaT = 2·deltaT`

**결과적으로 전체 시간의 하한은 `tracking_dur_`(3초)이고, 그 이상으로만 늘어납니다.**
추적 비용이 `[0, tracking_dur_]` 구간의 예측점에 걸려 있으므로 시간축이 이보다 짧아지면
비용을 평가할 수 없기 때문입니다.

`traj_opt_fake.cc`는 이 트릭 없이 `rhoT_ * ΣT`를 직접 쓰고 `deltaT`도 없습니다
(골 지향에서는 빨리 도착할수록 좋으므로 시간 하한이 필요 없음).

### 4.5.4 공간 변수 변환 — `forwardP`

각 웨이포인트가 대응하는 볼록 다면체 **안에** 있도록 만듭니다.

```cpp
k = cfgPolyVs[i].cols() - 1;                        // 정점 개수 - 1
q = 2.0 / (1.0 + p.segment(j,k).squaredNorm()) * p.segment(j,k);
inP.col(i) = cfgPolyVs[i].rightCols(k) * q.cwiseProduct(q) + cfgPolyVs[i].col(0);
```

`cfgPolyVs[i]`는 `[v₀ | v₁-v₀ | v₂-v₀ | …]` 형태(원점 + 오프셋)입니다.

`r = 2p/(1+|p|²)`이면 `|r|² = 4|p|²/(1+|p|²)² ≤ 1` (AM-GM). 따라서

```
inP = v₀ + Σᵢ rᵢ² (vᵢ - v₀)      단, Σ rᵢ² ≤ 1
```

는 정점들의 **볼록 결합**(v₀에 남은 가중치 `1-Σrᵢ²`)이 되어 다면체 안에 반드시 들어갑니다.
`p ∈ ℝᵏ`는 자유 변수이므로 제약이 완전히 사라집니다.

역변환 `backwardP`([traj_opt.cc:108](../src/planning/traj_opt/src/traj_opt.cc#L108))는 닫힌 형태가
없어서 **웨이포인트마다 작은 비선형 최소제곱을 L-BFGS로 128회 이내에 푸는** 방식입니다.
초기값 설정 시 한 번만 호출됩니다.

### 4.5.5 웨이포인트가 놓이는 위치 — `extractVs`

```cpp
for (i = 0 .. M-1) {            // M = hPs.size() - 1
  enumerateVs(hPs[i], curIV);              vPs.push_back(...);   // 다면체 i 내부
  curIH << hPs[i], hPs[i+1];
  enumerateVs(curIH, curIV);               vPs.push_back(...);   // 다면체 i ∩ i+1
}
enumerateVs(hPs.back(), curIV);            vPs.push_back(...);
```

결과는 `2M+1 = N_-1`개로, 웨이포인트가 번갈아 배치됩니다.

```
회랑:      ┌── P0 ──┐┌── P1 ──┐┌── P2 ──┐
웨이포인트: w0     w1        w2       w3      w4
           (P0)  (P0∩P1)   (P1)   (P1∩P2)  (P2)
조각:    ─ 0 ─┬─ 1 ─┬─ 2 ─┬─ 3 ─┬─ 4 ─┬─ 5 ─
```

**"회랑 하나당 조각 2개"**(`N_ = 2·#회랑`)가 여기서 나옵니다. 교집합 영역에 웨이포인트를 놓아
궤적이 다음 회랑으로 넘어가는 지점을 명시적으로 만든 것입니다.

`enumerateVs`는 `sdlp.hpp`(선형계획법)로 내부점을 찾고 `quickhull.hpp`(볼록 껍질)로 정점을
열거합니다. 실패하면 `generate_traj`가 `false`를 반환합니다.

### 4.5.6 목적 함수

```cpp
obj.jerkOpt_.generate(P, T);              // MINCO: 밴드 행렬 LU로 계수 b 계산
double cost = obj.jerkOpt_.getTrajJerkCost();   // ∫|jerk|² dt
obj.jerkOpt_.calGrads_CT();               // ∂J/∂c, ∂J/∂T
obj.addTimeIntPenalty(cost);              // 회랑 + 속도 + 가속도
obj.addTimeCost(cost);                    // 추적 거리 + 가시성
obj.jerkOpt_.calGrads_PT();               // ∂/∂c → ∂/∂P, ∂/∂T 로 전파
```

전체 비용:

```
J = ∫|jerk|²dt
  + ρT · ΔT²
  + Σ_pieces Σ_{k=0}^{K} ω_k · Δs · [ ρP·corridor³ + ρV·vpen³ + ρA·apen³ ]
  + Σ_{i=0}^{M} ρ(i) · dt · [ tracking(p(t_i), target_i) + visibility(p(t_i), …) ]
```

### 4.5.7 적분 페널티 — `addTimeIntPenalty`

각 조각을 `K_`(기본 8)등분해서 사다리꼴 적분합니다.

```cpp
step = jerkOpt_.T1(i) / K_;
omg = (j == 0 || j == K_) ? 0.5 : 1.0;      // 사다리꼴 가중치
const auto& hPoly = cfgHs_[i / 2];          // 조각 i는 회랑 i/2에 속함
```

시간 배분 `T_i`가 최적화 변수이므로, 샘플 지점도 시간에 따라 움직입니다.
그래서 `T_i`에 대한 그래디언트에 **두 항**이 들어갑니다.

```cpp
gradViolaPt = alpha * grad_tmp.transpose() * vel;      // alpha = j/K_
jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaPt);
                       //  ─────────────   ────────────────────
                       //  적분 구간 변화     샘플 위치 이동
```

**페널티 함수들**

| 항목 | 위반량 | 비용 | 파라미터 |
|---|---|---|---|
| 회랑 | `n·(p - q) + d_clear` | `ρP · pen³` | `clearance_d_ = 0.2` |
| 속도 | `\|v\|² - v_max²` | `ρV · pen³` | `vmax = 3.0` |
| 가속도 | `\|a\|² - a_max²` | `ρA · pen³` | `amax = 6.0` |

모두 **3차 페널티**입니다. 위반이 0일 때 값·1차·2차 도함수가 모두 0이라 C²로 매끄럽게 붙습니다.

회랑 페널티는 각 면에 대해 독립적으로 걸립니다.

```cpp
for (int i = 0; i < hPoly.cols(); ++i) {
  Eigen::Vector3d norm_vec = hPoly.col(i).head<3>();
  double pen = norm_vec.dot(p - hPoly.col(i).tail<3>() + clearance_d_ * norm_vec);
  if (pen > 0) { gradp += rhoP_ * 3*pen*pen * norm_vec;  costp += rhoP_ * pen³; }
}
```

`clearance_d_ * norm_vec`을 더해서 **면에서 0.2m 안쪽**을 실질 경계로 삼습니다.

### 4.5.8 시간 스탬프 비용 — `addTimeCost`

```cpp
int M = tracking_ps_.size() * 4 / 5;      // 예측 뒤쪽 20%는 버림
double step = tracking_dt_;               // 0.2s
for (int i = 0; i < M; ++i) {
  double rho = exp2(-3.0 * i / M);        // 1.0 → 0.125로 감쇠
  while (t - t_pre > T(piece)) { t_pre += T(piece); piece++; }   // t가 속한 조각 찾기
  s1 = t - t_pre;                         // 조각 내 국소 시간
  pos = c.transpose() * beta0;            // 궤적 위치 p(t)
  ...
  t += step;
}
```

`i`번째 예측점을 **시각 `i·0.2초`의 궤적 위치**와 짝짓습니다. 시간 배분 `T`가 바뀌면 어느 조각의
어느 지점인지가 바뀌므로, `T`에 대한 그래디언트가 붙습니다.

```cpp
if (piece > 0) {
  jerkOpt_.gdT.head(piece).array() += -rho * step * grad_tmp.dot(vel);
}
```

앞선 조각들의 시간이 늘어나면 같은 절대 시각 `t`에서 궤적을 더 이른 지점에서 평가하게 되므로
부호가 음수입니다.

#### 추적 거리 페널티 — `grad_cost_p_tracking`

수평/수직을 분리해서 다룹니다.

```
수평: (tracking_dist - tol)² ≤ dr² ≤ (tracking_dist + tol)²      ← 링(annulus)
수직:              dz² ≤ tol²
```

```cpp
double pen = dr2 - upper;
if (pen > 0) { costp += penF(pen, grad); gradp.head(2) += 2*grad*dp.head(2); }  // 너무 멀다
else {
  pen = lower - dr2;
  if (pen > 0) { costp += pen³; gradp.head(2) -= 6*pen²*dp.head(2); }           // 너무 가깝다
}
```

**바깥쪽 위반만 `penF`를 씁니다** ([traj_opt.cc:598](../src/planning/traj_opt/src/traj_opt.cc#L598)).

```cpp
static double penF(const double& x, double& grad) {
  static double eps = 0.05;
  if (x < 2*eps) { grad = 12/eps²·x² - 4/eps³·x³;  return 4/eps²·x³ - x⁴/eps³; }
  else           { grad = 16;                      return 16*(x - eps); }
}
```

`x = 2ε`에서 값 `16ε`, 기울기 `16`으로 **C¹ 연속**입니다.
작은 위반에는 부드럽게, 큰 위반에는 **선형으로만** 커집니다.

이게 중요한 이유: 목표물이 순간적으로 멀어졌을 때 3차 페널티를 쓰면 추적 항이 다른 모든 비용을
압도해서 궤적이 회랑을 뚫고 나가버립니다. 선형 포화가 이를 막습니다.
반대로 **너무 가까운 쪽은 3차**입니다 — 충돌 위험이 있으니 강하게 밀어냅니다.

#### 가시성 페널티 — `grad_cost_visibility`

```cpp
Eigen::Vector3d a = p - center;        // 목표물 → 드론
Eigen::Vector3d b = vis_p - center;    // 목표물 → 가시 영역 중심 방향
double theta_less = max(theta - theta_clearance_, 0);
double pen = cos(theta_less) - a·b/(|a||b|);
if (pen > 0) {
  costp = pen³;
  gradp = 3pen² · -( |a|·b - (a·b)/|a| · a ) / (|a|²|b|);
  gradp *= rhosVisibility_;
}
```

`a`와 `b` 사이 각이 `theta - theta_clearance` 이내여야 한다는 제약입니다.
`theta`는 `visible_pair`가 계산한 부채꼴 반각, `theta_clearance_`(0.8 rad)는 안전 여유.
부채꼴이 `theta_clearance_`보다 좁으면 `theta_less = 0` → **정확히 `visible_p` 방향에 있으라**는
가장 엄격한 제약이 됩니다.

그래디언트는 `∂/∂a [(a·b)/(|a||b|)]`를 부호 반전한 것입니다.

```
∂/∂a [(a·b)/(|a||b|)] = ( |a|b - (a·b)a/|a| ) / (|a|²|b|)
```

### 4.5.9 L-BFGS 설정

```cpp
lbfgs_params.mem_size = 16;      // 과거 16스텝 곡률 정보
lbfgs_params.past = 3;           // 3회 개선 없으면 종료
lbfgs_params.g_epsilon = 1e-10;
lbfgs_params.min_step = 1e-32;
lbfgs_params.delta = delta;      // 기본 1e-4
```

조기 종료는 반복 1000회 상한만 겁니다.

```cpp
static inline int earlyExit(...) { return k > 1e3; }
```

**시간 상한이 없습니다.** 20Hz 루프에서 최적화가 오래 걸리면 다음 리플랜 주기를 놓칠 수 있는데,
`plan_timer_`는 ROS 타이머라 콜백이 밀리면 그냥 다음 실행이 늦어집니다.

### 4.5.10 초기값 — `setBoundConds`

```cpp
T.setConstant(sum_T_ / N_);            // 시간을 균등 배분
backwardT(T, t_);

for (i = 0 .. N_-2)                    // 각 다면체 정점들의 무게중심
  P.col(i) = cfgVs_[i].rightCols(k).rowwise().sum() / (1.0 + k) + cfgVs_[i].col(0);
backwardP(P, cfgVs_, p_);

jerkOpt_.reset(initS, finalS, N_);
```

그리고 `x_[dim_p_+dim_t_] = 0.1` — `deltaT`의 초기값(전체 시간 = 3.0 + 0.01초).

경계 상태는 `vmax_`, `amax_`로 클램프됩니다. 이전 궤적에서 이어받은 상태가 제약을 이미 위반하고
있으면 최적화가 발산하기 때문입니다.

---

## 4.6 MINCO 궤적 표현

**파일**: [minco.hpp:161](../src/planning/traj_opt/include/traj_opt/minco.hpp#L161)

각 조각은 5차 다항식이고, 계수 `c ∈ ℝ^{6×3}`는 웨이포인트 `P`와 시간 `T`로부터
**선형 시스템의 해**로 결정됩니다.

```
A(T) · b = M(P, 경계조건)
```

`A`는 6N×6N **밴드 행렬**(상하 대역폭 6)이라 `factorizeLU()` + `solve()`가 O(N)입니다.

행 구성(조각 i 기준):

| 행 | 조건 |
|---|---|
| `6i+3` | jerk 연속 |
| `6i+4` | snap 연속 |
| `6i+5` | 웨이포인트 통과: `p_i(T_i) = P_i` |
| `6i+6` | 위치 연속 |
| `6i+7` | 속도 연속 |
| `6i+8` | 가속도 연속 |

첫 3행은 시작 PVA, 마지막 3행은 종료 PVA.

### 그래디언트 전파

```
∂J/∂c  ──calGrads_CT──▶ (∂J/∂c, ∂J/∂T의 직접 항)
       ──calGrads_PT──▶ A.solveAdj(gdC) → ∂J/∂P, ∂J/∂T
```

`calGrads_PT`가 **수반(adjoint) 시스템** `Aᵀ λ = ∂J/∂c`를 풀어 `∂J/∂P`, `∂J/∂T`를 얻습니다.
이 덕분에 `c`를 명시적으로 미분하지 않고도 O(N)에 전체 그래디언트가 나옵니다.
이것이 MINCO의 핵심 기여입니다 ([GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) 참조).

`getTrajJerkCost()`는 닫힌 형태입니다.

```cpp
36·|c₃|²T + 144·c₃·c₄·T² + 192·|c₄|²T³ + 240·c₃·c₅·T³ + 720·c₄·c₅·T⁴ + 720·|c₅|²T⁵
```

---

## 4.7 파라미터 감도 요약

| 파라미터 | 값 | 크게 하면 | 작게 하면 |
|---|---|---|---|
| `rhoT` | 100 | 궤적이 짧아짐(공격적) | 느긋해짐 |
| `rhoP` | 10000 | 회랑 중앙으로 붙음 | 벽에 붙거나 뚫음 |
| `rhoTracking` | 1000 | 거리 유지 우선 | 거리 흐트러짐 |
| `rhosVisibility` | 10000 | 가시성 우선(우회 증가) | 목표물 놓칠 위험 ↑ |
| `theta_clearance` | 0.8 | 가시 영역 중앙 고수 | 경계까지 허용 |
| `tracking_dur` | 3.0 | 예측·궤적이 길어짐(느린 반응) | 근시안적 |
| `tracking_dt` | 0.2 | (커지면) 비용 샘플 성김 | 계산량 ↑ |
| `K` | 8 | 적분 페널티 정확 / 계산량 ↑ | 회랑 위반 놓칠 수 있음 |
| `tolerance_d` | 0.3 | 거리 허용폭 넓음 | 빡빡함 |

`simulation2.launch`는 `rhosVisibility_`만 `10000` vs `0`으로 두고 나머지를 동일하게 유지해
가시성 항의 효과를 직접 비교합니다.

## 4.8 다음 문서

- 파라미터·토픽 전수 목록 → [05. ROS 인터페이스](05-ros-interface.md)
- 코드에서 발견한 문제 → [07. 코드 리딩 노트](07-code-notes.md)
