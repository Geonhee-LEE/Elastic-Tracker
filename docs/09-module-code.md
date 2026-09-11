# 09. 모듈별 구현 — 코드로 읽는 파이프라인

한 번의 리플랜에서 무엇이 어떤 순서로 실행되는지, **각 모듈이 실제로 어떻게 짜여 있는지**를
원본 코드로 따라갑니다. 개념 설명은 [04. 알고리즘](04-algorithms.md)과
[08. 계보](08-zju-fast-lab-family.md)에 있고, 여기서는 **코드가 왜 그 모양인지**만 봅니다.

행 번호는 이 저장소 기준입니다. HTML 판은 [`modules.html`](modules.html)에 있습니다.

## 파이프라인 한눈에

```
odom ─┐
      ├─► ① 점유 격자 ─► ② A* ─► ③ 안전 회랑 ─► ④ 회랑 다듬기
map ──┘                                              │
                                                     ▼
                                       ⑤ H→V 꼭짓점 열거
                                                     │
        ┌────────────────────────────────────────────┘
        ▼
  ⑥ 시간 변환 τ→T ─┐
                    ├─► ⑧ MINCO ─► ⑨ 비용·그래디언트 ─► ⑩ L-BFGS ─┐
  ⑦ 점 변환 p→q ───┘         ▲                                      │
                              └──────────────────────────────────────┘
                                         (수렴할 때까지)
        ▼
  ⑫ 검증 · 발행 ─► 제어기
```

| # | 모듈 | 원본 | 벤치마크 · JS 이식본 |
|---|---|---|---|
| ① | 점유 격자 · 광선 검사 | `env.hpp:94` | 해석적 원판 거리 |
| ② | A* 프런트엔드 | `env.hpp:536` | `bench.cpp:60` · `planner.js` |
| ③ | 안전 회랑 (DecompROS) | `env.hpp:279` | 자체 IRIS식 `bench.cpp:105` |
| ④ | 회랑 다듬기 | `env.hpp:159, 228` | 없음 |
| ⑤ | H→V 꼭짓점 열거 | `geoutils.hpp:108` | 2D 반평면 클리핑 |
| ⑥ | 시간 변환 τ→T | `traj_opt.cc:12` | 동일 |
| ⑦ | 점 변환 p→q | `traj_opt.cc:67` | 동일 |
| ⑧ | MINCO 코어 | `minco.hpp:161` | 동일 (2D) |
| ⑨ | 비용 · 그래디언트 | `traj_opt.cc:167, 430` | 동일 |
| ⑩ | L-BFGS | `lbfgs_raw.hpp:1268` | 약한 Wolfe 판 |
| ⑪ | B-스플라인 + 앵커 | 없음 (ego-planner 정식화) | `bench.cpp:238` |
| ⑫ | 리플랜 루프 | `planning_nodelet.cpp:149` | 없음 (한 번만) |

---

## ① 점유 격자와 광선 검사

**하는 일** — 두 점 사이가 비어 있는지 본다. A*의 유효성 판정과 회랑 시드 구간 선택에 쓰입니다.

`src/planning/planning/include/env/env.hpp:94`

```cpp
bool inline checkRayValid(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                          double max_dist) const {
  Eigen::Vector3d dp = p1 - p0;
  double dist = dp.norm();
  if (dist > max_dist) { return false; }
  Eigen::Vector3i idx0 = mapPtr_->pos2idx(p0);
  Eigen::Vector3i idx1 = mapPtr_->pos2idx(p1);
  Eigen::Vector3i d_idx = idx1 - idx0;
  Eigen::Vector3i step = d_idx.array().sign().cast<int>();
  ...
```

**설계 의도** — 거리장(ESDF)을 만들지 않습니다. 필요한 것은 "이 선분이 자유로운가" 하나뿐이라
**3D DDA(격자 순회)**로 셀을 따라가며 점유 여부만 봅니다. 맵 전체의 거리장을 매 프레임 굽는
Fast-Planner 계열과 갈라지는 지점이 여기입니다.

**벤치마크와의 차이** — 벤치마크는 장애물이 원판이라 거리를 **해석적으로** 계산합니다.

```js
// docs/bench/planner.js
function distToObs(S, x, y) {
  let d = 1e9;
  for (const o of S.obs) d = Math.min(d, Math.hypot(x - o.x, y - o.y) - o.r);
  d = Math.min(d, x - S.xmin, S.xmax - x, y - S.ymin, S.ymax - y);
  return d;
}
```

격자 해상도에서 오는 오차도, 미지 영역도 없습니다. **실제 시스템보다 훨씬 쉬운 조건**입니다.

---

## ② A* 프런트엔드

**하는 일** — 시작에서 목표까지 격자 위 최단 경로. 어떤 위상(어느 틈으로 지날지)을 고를지가
여기서 결정되고, 뒤쪽 최적화는 그 선택을 **바꾸지 않습니다.**

`env.hpp:536`

```cpp
auto calulateHeuristic = [&](const NodePtr& ptr) {
  Eigen::Vector3i dp = end_idx - ptr->idx;
  int dx = dp.x(), dy = dp.y(), dz = dp.z();
  ptr->h = abs(dx) + abs(dy) + abs(dz);          // 맨해튼
  double dx0 = (start_idx - end_idx).x();
  double dy0 = (start_idx - end_idx).y();
  double cross = fabs(dx * dy0 - dy * dx0) + abs(dz);
  ptr->h += 0.001 * cross;                        // 동점 깨기
};
// NOTE 6-connected graph
for (int i = 0; i < 3; ++i) {
  Eigen::Vector3i neighbor(0, 0, 0);
  neighbor[i] =  1; neighbors.emplace_back(neighbor, 1);
  neighbor[i] = -1; neighbors.emplace_back(neighbor, 1);
}
```

**세 가지 선택**

- **6-이웃**입니다. 대각선을 쓰지 않아 경로가 계단 모양이 되지만, 뒤에서 회랑이 그걸 다 삼키므로
  경로의 매끄러움은 중요하지 않습니다. **위상만 맞으면 됩니다.**
- 휴리스틱이 **맨해튼**입니다. 6-이웃 격자에서 맨해튼은 허용적(admissible)이고 일관적이라
  최적성이 보장됩니다.
- `0.001 * cross` 는 **동점 깨기**입니다. 같은 f 값이 널려 있을 때 시작-목표 직선에서 덜 벗어난
  쪽을 먼저 열어 탐색을 좁힙니다.

**노드는 지연 할당됩니다** (`env.hpp:63`)

```cpp
inline NodePtr visit(const Eigen::Vector3i& idx) {
  auto iter = visited_nodes_.find(idx);
  if (iter == visited_nodes_.end()) {
    auto ptr = data_[visited_nodes_.size()];   // 미리 잡아 둔 풀에서 꺼낸다
    ptr->idx = idx;
    ptr->valid = !mapPtr_->isOccupied(idx);
    ptr->state = UNVISITED;
    visited_nodes_[idx] = ptr;
    return ptr;
  }
  return iter->second;
}
```

`data_[MAX_MEMORY]` 를 생성자에서 통째로 잡아 두고 인덱스로 꺼내 씁니다. **탐색 중 `new` 가 없습니다.**
20 Hz 루프에서 할당 지연을 없애려는 것입니다. 대가로 `MAX_MEMORY` 를 넘으면 탐색이 그냥 끝납니다.

> **주의** — `visit()` 은 `idx` 의 경계 검사를 하지 않습니다. `mapPtr_->isOccupied(idx)` 가
> 범위 밖 인덱스를 어떻게 다루는지에 안전성이 의존합니다. [07. 코드 노트](07-code-notes.md) 참조.

---

## ③ 안전 회랑 (SFC)

**하는 일** — A* 경로를 **겹치는 볼록 다면체 사슬**로 감쌉니다. 이후 최적화는 이 안에서만 놉니다.

`env.hpp:279`

```cpp
while (idx < path_len - 1) {
  int next_idx = idx;
  // looking forward -> get a farest next_idx
  while (next_idx + 1 < path_len &&
         checkRayValid(path[idx], path[next_idx + 1], bbox_width)) {
    next_idx++;
  }
  vec_Vec3f line;
  line.push_back(path[idx]);
  line.push_back(path[next_idx]);
  keyPts.emplace_back(path[idx], path[next_idx]);
  getPointCloudAroundLine(line, maxWidth, obs_pc);   // 주변 점유 셀만 모은다
  decomp_util.set_obs(obs_pc);
  decomp_util.dilate(line);                          // ← DecompROS
  decompPolys.push_back(decomp_util.get_polyhedrons()[0]);

  // find a farest idx in current corridor
  idx = next_idx;
  while (idx + 1 < path_len && decompPolys.back().inside(path[idx + 1])) {
    idx++;
  }
}
```

**두 개의 탐욕 루프**가 핵심입니다.

1. 첫 `while` — **시드 선분을 최대한 길게** 잡습니다. 선분이 길수록 회랑이 커지고 개수가 줍니다.
2. 마지막 `while` — 만든 회랑이 **이미 삼킨 경로점을 건너뜁니다.** 회랑 개수가 궤적 조각 수
   (`N = 2 × 회랑 수`)를 정하므로, 이 두 줄이 곧 **최적화 문제의 크기**를 정합니다.

`dilate()` 는 **DecompROS**(Sikang Liu 외, UPenn, RA-L 2017)입니다. 선분 주위에 타원체를 키우고
닿는 점마다 접평면을 잘라 다면체를 만듭니다.

**벤치마크는 DecompROS를 못 씁니다** (ROS 의존). 그래서 IRIS식 생성기를 새로 썼습니다.

```js
// docs/bench/planner.js — corridor()
const used = new Array(S.obs.length).fill(false);
for (let it = 0; it < S.obs.length; it++) {
  let worst = -1e9, wi = -1;
  for (let i = 0; i < S.obs.length; i++) {
    if (used[i]) continue;
    ...
    const spx = o.x - nx2 * o.r, spy = o.y - ny2 * o.r;   // 원판의 접점
    if (!inside(spx, spy)) continue;      // 이미 다른 면에 의해 잘려나갔다
    if (pen > worst) { worst = pen; wi = i; ... }
  }
  if (wi < 0) break;
  used[wi] = true;                        // ← 이 표시가 없으면 같은 평면만 반복 추가된다
  H.push([wnx, wny, wpx, wpy]);
}
```

`used` 표시가 빠져 있던 초판에는 **회랑마다 장애물이 하나만 제거되는 결함**이 있었습니다.
접평면 위의 접점은 `n·(x−p) ≤ 0` 판정에서 값이 정확히 0이라 여전히 "안"으로 나오고,
그래서 같은 장애물이 계속 최악 후보로 뽑혔기 때문입니다. 대화형 도구를 만들며 발견해 고쳤습니다.

---

## ④ 회랑 다듬기

**하는 일** — 쓸모없는 회랑을 걷어내고, 남은 것을 안쪽으로 민 뒤, 그래도 이웃끼리 통하는지
다시 확인합니다. `generateSFC` 의 뒷부분(`env.hpp:332`) 전체가 이 작업입니다.

### ④-a 중복 회랑 제거 (`env.hpp:228`)

이웃하지 않은 두 회랑이 이미 충분히 겹치면 사이의 것을 버립니다.

```cpp
for (int i = 2; i < (int)hPolys.size(); i++) {
  curIH.resize(6, hPoly0.cols() + hPolys[i].cols());
  curIH << hPoly0, hPolys[i];
  if (geoutils::findInteriorDist(curIH, interior) < 1.0) {   // 교집합이 좁으면
    ret_polys.push_back(hPoly0);                             // i-1 을 살린다
    hPoly0 = hPolys[i - 1];
  } else {
    ret = true;                                              // i-1 을 버린다
  }
}
```

`findInteriorDist` 는 두 다면체 교집합의 **최대 내접 구 반지름**을 선형계획(SDLP)으로 구합니다.
그게 1 m 이상이면 중간 회랑 없이도 통하므로 건너뜁니다.
**조각 수 = 2 × 회랑 수**이니 이 한 줄이 최적화 변수를 직접 줄입니다.

### ④-b 수축 (`env.hpp:337`)

모든 면을 안쪽으로 `dx` 만큼 밉니다 — 기체 반경과 맵 오차를 위한 마진입니다.

```cpp
void compressPoly(Polyhedron3D& poly, double dx) {
  vec_E<Hyperplane3D> hyper_planes = poly.hyperplanes();
  for (uint j = 0; j < hyper_planes.size(); j++) {
    hyper_planes[j].p_ = hyper_planes[j].p_ - hyper_planes[j].n_ * dx;
  }
  poly = Polyhedron3D(hyper_planes);
}
```

호출은 `compressPoly(hPolys[i], 0.1)`. 즉 **10 cm**이고, 이후 최적화의 `clearance_d = 0.2 m` 와
**따로 누적**됩니다. 실효 마진은 **두 겹**입니다.

> **여기에 알려진 결함이 있습니다** — 수축 여부를 판정할 때 `hPolys[i]` 가 아니라
> 직전 루프에서 남은 `current_poly`(마지막 다면체)를 봅니다. 그래서 판정이 사실상
> 전 회랑에 일괄 적용됩니다. 자세한 내용은 [7.2 `current_poly` 오참조](07-code-notes.md#72-generatesfc의-current_poly-오참조).

### ④-c 되돌리기 (`env.hpp:344`)

수축 때문에 이웃 회랑끼리 **교집합이 사라졌으면** 다시 펴 줍니다.

```cpp
for (int i = 1; i < (int)hPolys.size(); i++) {
  curIH << hPolys[i - 1], hPolys[i];
  if (!geoutils::findInterior(curIH, interior)) {     // 교집합이 비었다
    if (!inflate[i - 1]) {
      compressPoly(hPolys[i - 1], -0.1);              // ← 음수 = 되펴기
      inflate[i - 1] = 1;
    }
  } else continue;
  curIH << hPolys[i - 1], hPolys[i];
  if (!geoutils::findInterior(curIH, interior)) {     // 그래도 비었으면
    if (!inflate[i]) { compressPoly(hPolys[i], -0.1); inflate[i] = 1; }
  }
}
```

**회랑 사슬이 끊기면 안 되기 때문**입니다. ⑦의 웨이포인트 변환은 이웃 회랑의 **교집합**
다면체를 쓰므로(⑤의 `extractVs` 가 `hPs[i]` 와 `hPs[i+1]` 을 합쳐 꼭짓점을 뽑습니다),
교집합이 비면 그 자리에서 최적화가 성립하지 않습니다.

`inflate[]` 플래그로 **한 번만** 되폅니다. 무한히 부풀어 장애물을 삼키는 것을 막습니다.

"안전을 위해 줄이되, 연결이 끊기면 안전보다 연결을 택한다" — 이 우선순위가 코드에 박혀 있습니다.

---

## ⑤ H→V 꼭짓점 열거

**하는 일** — 반평면 표현 `{x : nᵢ·(x−pᵢ) ≤ 0}` 을 **꼭짓점 목록**으로 바꿉니다.
⑦의 웨이포인트 변환이 꼭짓점의 볼록결합을 쓰기 때문에 반드시 필요합니다.

`geoutils.hpp:108` — **극쌍대(polar dual)** 를 씁니다.

```cpp
inline void enumerateVs(const Eigen::MatrixXd &hPoly, const Eigen::Vector3d &inner,
                        Eigen::MatrixXd &vPoly, const double epsilon = 1.0e-6) {
  // 1. 내부점 inner 를 원점으로 옮기고, 각 면을 점 n/b 로 사상한다
  Eigen::RowVectorXd b = hPoly.topRows<3>().cwiseProduct(hPoly.bottomRows<3>()).colwise().sum() -
                         inner.transpose() * hPoly.topRows<3>();
  Eigen::MatrixXd A = hPoly.topRows<3>().array().rowwise() / b.array();

  // 2. 그 점들의 볼록 껍질을 구한다
  quickhull::QuickHull<double> qh;
  const auto cvxHull = qh.getConvexHull(A.data(), A.cols(), false, true, qhullEps);

  // 3. 껍질의 각 면(삼각형)이 원래 다면체의 꼭짓점 하나에 대응한다
  for (int i = 0; i < hNum; i++) {
    point = A.col(idBuffer[3 * i + 1]);
    edge0 = point - A.col(idBuffer[3 * i]);
    edge1 = A.col(idBuffer[3 * i + 2]) - point;
    normal = edge0.cross(edge1);
    rV.col(i) = normal / normal.dot(point);
  }
  filterVs(rV, epsilon, vPoly);
  vPoly = (vPoly.array().colwise() + inner.array()).eval();   // 원위치로
}
```

**면 ↔ 점을 맞바꾸는 쌍대성**이 전부입니다. "다면체의 꼭짓점 열거"라는 어려운 문제를
"점집합의 볼록 껍질"이라는 잘 풀린 문제로 옮깁니다.

내부점은 선형계획으로 찾습니다 (`geoutils.hpp:37`). `max r  s.t.  nᵢ·x + r ≤ bᵢ` 형태의
**체비셰프 중심** 문제이고, `sdlp::linprog` (Seidel 1991, 저차원에서 기대 O(m))으로 풉니다.

**벤치마크는 2D라 훨씬 단순합니다.** 큰 상자를 반평면으로 차례차례 잘라 나가면 끝입니다
(Sutherland–Hodgman 클리핑). SDLP도 QuickHull도 필요 없습니다.

---

## ⑥ 시간 변환 τ → T

**하는 일** — `T > 0` 이라는 제약을 **변수 치환으로 없앱니다.**

`traj_opt.cc:12`

```cpp
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0)
                 : (1.0 - sqrt(2.0 / T - 1.0));
}
```

`exp` 대신 이 함수를 쓰는 이유는 세 가지입니다.

- **치역이 (0, ∞)** — `T > 0` 이 자동으로 만족됩니다. 제약 하나가 사라집니다.
- **C² 연속** — `t = 0` 에서 값·1차·2차 미분이 이어집니다. L-BFGS가 2차 정보를 근사하므로
  여기서 꺾이면 수렴이 망가집니다. 이름의 `C2` 가 그 뜻입니다.
- **다항식/유리식** — `exp` 보다 싸고, 큰 `t` 에서 오버플로가 없습니다.

### 그런데 추적판은 시간 파라미터화가 다릅니다

이것이 회랑판(A)과 추적판을 가르는 가장 큰 차이입니다.

**회랑판** (`traj_opt_fake.cc:17`) — 조각마다 독립. 총 시간도 자유.

```cpp
static void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t,
                     Eigen::Ref<Eigen::VectorXd> vecT) {
  for (int i = 0; i < M; ++i) vecT(i) = expC2(t(i));
}
```

**추적판** (`traj_opt.cc:19`) — 비율만 자유. 총 시간은 `sT` 에 **못 박힙니다.**

```cpp
static void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t,
                     const double& sT, Eigen::Ref<Eigen::VectorXd> vecT) {
  for (int i = 0; i < M; ++i) vecT(i) = expC2(t(i));
  vecT(M) = 0.0;
  vecT /= 1.0 + vecT.sum();      // ← 소프트맥스식 정규화
  vecT(M) = 1.0 - vecT.sum();
  vecT *= sT;                     // ← 총 시간은 sT
}
```

그리고 `sT` 는 **단 하나의 추가 변수** `deltaT` 로 만들어집니다 (`objectiveFunc`).

```cpp
double deltaT = x[obj.dim_t_ + obj.dim_p_];
// T_sigma = T_s + deltaT^2
double sumT = obj.sum_T_ + deltaT * deltaT;
```

`T_s` 는 목표물 예측 지평(`tracking_dur = 3.0 s`)에서 오고, `δ²` 로 **늘어나기만** 합니다.
이것이 논문 제목의 **“Elastic”** 입니다. 시간 배분(비율)과 지평 길이(총 시간)를 분리한 것이죠.

> **벤치마크의 A는 회랑판이므로 이 탄성 시간을 쓰지 않습니다.**
> 총 시간이 자유롭고 `rhoT · ΣT` 로만 벌점을 받습니다.

---

## ⑦ 점 변환 p → q

**하는 일** — "웨이포인트가 다면체 안에 있어야 한다"는 제약을 **변수 치환으로 없앱니다.**

`traj_opt.cc:67`

```cpp
static void forwardP(const Eigen::Ref<const Eigen::VectorXd>& p,
                     const std::vector<Eigen::MatrixXd>& cfgPolyVs,
                     Eigen::MatrixXd& inP) {
  int j = 0, k;
  for (int i = 0; i < M; ++i) {
    k = cfgPolyVs[i].cols() - 1;
    q = 2.0 / (1.0 + p.segment(j, k).squaredNorm()) * p.segment(j, k);
    inP.col(i) = cfgPolyVs[i].rightCols(k) * q.cwiseProduct(q) + cfgPolyVs[i].col(0);
    j += k;
  }
}
```

두 단계입니다.

1. `q = 2p / (1 + ‖p‖²)` — **역스테레오 투영.** ℝᵏ 전체를 단위구 위로 보냅니다. 따라서 `‖q‖ ≤ 1`.
2. `Σ qᵢ² = 1` 이 되므로 `qᵢ²` 가 곧 **볼록결합 계수**입니다. 꼭짓점들의 볼록결합은 항상 다면체 안입니다.

**제약 없는 ℝᵏ 의 어떤 값을 넣어도 결과가 회랑 안**입니다. 페널티가 아니라 **정확한 소거**입니다.

**대가는 과다 매개변수화**입니다. 3자유도 웨이포인트 하나에 `k = 꼭짓점수 − 1` 개의 변수를 씁니다.
면이 14개인 다면체면 오일러 공식으로 꼭짓점이 대략 24개, 즉 **점 하나에 23개 변수**입니다.
변수를 늘려서 제약을 없앤 거래입니다.

역방향은 닫힌 식이 없어 **작은 비선형 최소제곱**을 매번 풉니다 (`backwardP`, `traj_opt.cc:108`).
초기값 `p = 1/(√(k+1)+1)` 은 모든 꼭짓점에 균등한 무게를 주는 점입니다.

---

## ⑧ MINCO 코어

**하는 일** — 웨이포인트 `q` 와 시간 `T` 로부터 다항식 계수 `c` 를 **O(M)** 에 만들고,
`∂F/∂c` 를 `∂F/∂q`, `∂F/∂T` 로 **O(M)** 에 되돌립니다.

`minco.hpp:161` (`MinJerkOpt`, s = 3, 5차 다항식, 조각당 계수 6개)

**행렬 구성** — `A(0,0)=1, A(1,1)=1, A(2,2)=2` 로 시작·속도·가속을 박고,
조각 이음마다 6줄씩 채웁니다.

```cpp
for (int i = 0; i < N - 1; i++) {
  A(6*i+3, 6*i+3) = 6.0;                    // 저크 연속
  A(6*i+3, 6*i+4) = 24.0 * T1(i);
  A(6*i+3, 6*i+5) = 60.0 * T2(i);
  A(6*i+3, 6*i+9) = -6.0;
  A(6*i+4, 6*i+4) = 24.0;                   // 스냅 연속
  A(6*i+4, 6*i+5) = 120.0 * T1(i);
  A(6*i+4, 6*i+10) = -24.0;
  A(6*i+5, 6*i) = 1.0; ...                  // 웨이포인트 통과
  A(6*i+6, ...)                             // 위치 연속
  A(6*i+7, ...)                             // 속도 연속
  A(6*i+8, ...)                             // 가속 연속
  b.row(6*i+5) = inPs.col(i).transpose();   // ← q 가 들어가는 유일한 자리
}
A.factorizeLU();
A.solve(b);
```

**행렬 A는 시간 T에만 의존하고, 우변 b에만 웨이포인트가 들어갑니다.** 이 분리가 뒤의 모든 것을
가능하게 합니다.

**대역폭이 6인 밴디드 LU**입니다 (`A.create(6*N, 6, 6)`). 피벗을 하지 않습니다 —

```cpp
// This function conducts banded LU factorization in place
// Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
```

주석이 직접 말하듯 **속도를 위해 수치 안정성을 포기**한 것입니다. `T` 가 극단적으로 작아지면
대각원소가 0에 가까워질 수 있고, 그래서 ⑥의 `expC2` 가 `T > 0` 을 보장하는 것이 중요합니다.

**그래디언트 역전파** — `calGrads_CT()` 가 `∂J/∂c`, `∂J/∂T` 를 채우고,
`calGrads_PT()` 가 `Aᵀλ = ∂F/∂c` 를 풀어 `∂F/∂q` 로 옮깁니다.

```cpp
inline void calGrads_PT() {
  A.solveAdj(gdC);                                   // Aᵀ λ = ∂F/∂c
  gdP.setZero();
  for (int i = 0; i < N - 1; i++)
    gdP.col(i) += gdC.row(6 * i + 5).transpose();    // b 의 그 줄이 곧 q
  ...
}
```

`q` 가 `b` 의 한 줄에 그대로 들어가므로 `∂b/∂q` 가 단위벡터이고,
**λ 의 해당 성분이 곧 `∂F/∂q`** 입니다. 같은 LU 분해를 재사용하므로 비용은 다시 O(M)입니다.

`∂F/∂T` 는 `∂A/∂T` 가 필요합니다. `B1` 행렬에 `negVel, negAcc, negJer, negSnp, negCrk` 를
채워 넣는 부분이 그것입니다.

---

## ⑨ 비용과 그래디언트

**하는 일** — 부드러움 + 시간 + 제약 위반을 하나의 스칼라로 합칩니다.

`traj_opt.cc:167` (추적판)

```cpp
obj.jerkOpt_.generate(P, T);
double cost = obj.jerkOpt_.getTrajJerkCost();   // ∫‖jerk‖²
obj.jerkOpt_.calGrads_CT();
obj.addTimeIntPenalty(cost);                    // 회랑 · 속도 · 가속
obj.addTimeCost(cost);                          // 추적 거리 · 가시성
obj.jerkOpt_.calGrads_PT();
```

**순서가 중요합니다.** `calGrads_CT()` 로 `gdC` 를 저크 항으로 초기화하고, 페널티들이 거기에
**누적**한 다음, 마지막에 `calGrads_PT()` 가 한 번에 `q`, `T` 로 옮깁니다.
페널티마다 역전파를 돌리지 않습니다.

**시간 적분 페널티** (`traj_opt.cc:430`) — 조각마다 `K+1` 점을 찍어 사다리꼴로 적분합니다.

```cpp
step = jerkOpt_.T1(i) / K_;
const auto& hPoly = cfgHs_[i / 2];              // ← 조각 2개당 회랑 1개
for (int j = 0; j < innerLoop; ++j) {
  beta0 << 1.0, s1, s2, s3, s4, s5;             // 위치 기저
  beta1 << 0.0, 1.0, 2*s1, 3*s2, 4*s3, 5*s4;    // 속도
  ...
  omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;   // 사다리꼴 가중치

  if (grad_cost_p_corridor(pos, hPoly, grad_tmp, cost_tmp)) {
    gradViolaPc = beta0 * grad_tmp.transpose();
    gradViolaPt = alpha * grad_tmp.transpose() * vel;
    jerkOpt_.gdC.block<6,3>(i*6, 0) += omg * step * gradViolaPc;
    jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaPt);
    cost += omg * step * cost_tmp;
  }
  ...
}
```

`gdT` 에 **두 항**이 더해지는 것을 보세요. `cost_tmp / K_` 는 적분 구간이 늘어나서 생기는 몫이고,
`step * gradViolaPt` 는 샘플 위치가 시간에 따라 움직여서 생기는 몫입니다. 하나라도 빠지면
시간 그래디언트가 틀립니다.

**회랑 페널티 자체는 3차식**입니다 (`traj_opt.cc:560`).

```cpp
double pen = norm_vec.dot(p - hPoly.col(i).tail<3>() + clearance_d_ * norm_vec);
if (pen > 0) {
  double pen2 = pen * pen;
  gradp += rhoP_ * 3 * pen2 * norm_vec;
  costp += rhoP_ * pen2 * pen;                  // ρ·pen³
}
```

`pen³` 은 `pen = 0` 에서 **값·1차·2차 미분이 모두 0** 입니다. C² 매끄러움을 유지하려는 것이고,
같은 이유로 **제약을 정확히 만족시키지 못합니다** — 경계에서 그래디언트가 0이라 밀어내는 힘이
사라지기 때문입니다. [08절 ④](08-zju-fast-lab-family.md#8-8-직접-구현해-비교하기)의 잔여 위배가
이 성질의 실측값입니다.

`clearance_d_ * norm_vec` 이 면을 **안쪽으로 미는** 부분입니다. ④의 `compressPoly(0.1)` 과
합쳐 실효 마진이 두 겹이 됩니다.

---

## ⑩ L-BFGS

**하는 일** — 무제약 문제를 푼다. ⑥⑦이 제약을 다 없앴으므로 이것 하나로 끝납니다.

`lbfgs_raw.hpp:1268`

```cpp
ls = line_search_morethuente(n, x, &fx, g, &step, d, xp, gp,
                             &step_min, &step_max, &cd, &param);
if (ls < 0) {
    /* 실패하면 백트래킹으로 되돌린다 */
    lss = line_search_backtracking(n, x, &fx, g, &step, d, xp, gp, ...);
}
```

**More–Thuente 강한 Wolfe** 가 1차, 백트래킹이 대비책입니다.
GCOPTER의 현재 `lbfgs.hpp` 는 Lewis–Overton **약한 Wolfe** 로 바뀌었는데, 비평활 문제에서
더 견고하기 때문입니다. Elastic-Tracker는 갈라져 나온 시점의 판을 그대로 들고 있습니다.

호출부 설정 (`traj_opt.cc`)

```cpp
lbfgs_params.mem_size = 16;      // 저장할 (s,y) 쌍
lbfgs_params.past = 3;           // 3회 전과 비교해 개선이 없으면 종료
lbfgs_params.g_epsilon = 1e-10;  // 사실상 끄고 delta 로만 판정
lbfgs_params.delta = 1e-4;
```

`g_epsilon` 을 1e-10 으로 두어 **그래디언트 기준을 사실상 비활성화**하고, 상대 개선량(`delta`)으로만
끝냅니다. 20 Hz 예산 안에서 "충분히 좋은 해"를 빨리 얻으려는 선택입니다.

**JS 이식본은 라인서치가 다릅니다.** More–Thuente 대신 약한 Wolfe 브래킷을 씁니다.
같은 장면에서 길이·총시간·최대속도는 0.5% 안에서 맞지만 **∫jerk² 는 최대 22% 벌어집니다** —
다른 국소 최적해에 앉기 때문입니다.

---

## ⑪ B-스플라인 + {p,v} 앵커 (벤치마크 전용)

**하는 일** — ego-planner 정식화. **원본 저장소에는 없고** 비교를 위해 재구현한 것입니다.

`docs/bench/bench.cpp:238`

```cpp
void buildAnchors() {
  for (int i = 3; i + 3 < n; ++i) {
    Vec3 q = Q.col(i);
    if (distToObs(*S, q) >= safe) continue;      // 안 부딪히면 건너뛴다
    ...
    // q 에서 가이드 경로 쪽으로 걸어 나가 자유 공간에 처음 닿는 점 p
    for (double s2 = 0.05; s2 <= 6.0; s2 += 0.05) {
      Vec3 c2 = q + dir * s2;
      if (distToObs(*S, c2) >= safe) { pp = c2; found = true; break; }
    }
    if (!dup) anchors[i].push_back({pp, dir});   // ← 누적한다. 지우지 않는다
  }
}
```

비용은 앵커마다 반평면 하나입니다.

```cpp
double pen = B.safe - (Q.col(i) - a.first).dot(a.second);
if (pen > 0) { f += B.wC*pen*pen*pen; G.col(i) += -3*B.wC*pen*pen*a.second; }
```

**거리장을 만들지 않는 대신, 부딪힌 제어점에서만 국소적으로 “밀어내는 방향”을 만들어 쌓습니다.**
이것이 ego-planner의 "ESDF-free" 의 실체입니다.

**원본과 다른 점**을 분명히 해 둡니다.

- 가중치 `wS=1, wC=1e4, wF=1e2` 는 이 벤치마크에서 튜닝한 값입니다.
- 사후 시간 신축을 **정확한 다항식 최댓값**으로 합니다. ego-planner는 3배 보수적인
  볼록포 상한을 쓰므로, **이 구현이 시간 배분에서 원본보다 유리**합니다.
- 리플랜 루프, 초기 B-스플라인 피팅, feasibility 보정 반복이 없습니다.

따라서 **D의 절대 수치를 ego-planner의 성능으로 읽으면 안 됩니다.**

---

## ⑫ 리플랜 루프

**하는 일** — 위 전부를 20 Hz로 다시 돌리고, 결과를 검증해 발행합니다.

`planning_nodelet.cpp:149` (`plan_timer_callback`)

```
1. odom · map 스냅샷 (원자적 락)
2. 리플랜 필요 판단 — 남은 궤적이 아직 유효한가 (validcheck)
3. 초기 상태 = 현재 궤적 위의 미래 시각 (계산 지연 보상)
4. 목표물 예측 시퀀스 생성
5. A* → 회랑 → 최적화
6. 충돌 검사 (validcheck)
7. 통과하면 발행, 실패하면 비상 정지
```

**핵심은 3번**입니다. 계획을 시작하는 시점이 "지금"이 아니라 **"계획이 끝났을 시각"** 입니다.
그래야 새 궤적이 현재 궤적과 이어집니다.

**벤치마크에는 이 루프가 없습니다.** 한 번만 풉니다. 그래서 벤치마크가 재는 것은
**"한 번의 계획 품질"** 이지 **"움직이는 기체가 실제로 그리는 궤적"** 이 아닙니다.
실제 비행 궤적은 20 Hz로 갈아 끼워진 조각들의 이어붙임이고, 여기에 제어기 추종 오차와
모터 1차 지연(시뮬레이터 기준 τ = 0.0333 s, 4.8 Hz)이 더해집니다.

---

## 어디를 만지면 무엇이 바뀌는가

| 바꾸고 싶은 것 | 만질 곳 |
|---|---|
| 어느 틈으로 지날지 | ② A* 휴리스틱 · 이웃 정의 |
| 최적화 문제 크기 | ③ 시드 선분 탐욕 루프 · ④ `filterCorridor` |
| 안전 마진 | ④ `compressPoly(0.1)` · ⑨ `clearance_d_` (두 겹) |
| 부드러움 ↔ 속도 | ⑨ `rhoT_` |
| 제약 위반 허용량 | ⑨ `rhoP_` · 페널티 차수 |
| 수렴 속도 ↔ 품질 | ⑩ `delta` · `mem_size` |
| 궤적 표현 자체 | ⑧ `MinJerkOpt` → 다른 s (GCOPTER의 `MINCO_S4NU` 등) |

---

## 함께 읽을 것

- [04. 알고리즘](04-algorithms.md) — 각 단계가 **무엇을** 하는지 (개념)
- [07. 코드 리딩 노트](07-code-notes.md) — 실제로 읽다가 찾은 결함 9건
- [08. ZJU FAST Lab 계보](08-zju-fast-lab-family.md) — MINCO 배경과 실측 비교
- [`docs/bench/`](bench/README.md) — 벤치마크 · 대화형 도구의 코드
