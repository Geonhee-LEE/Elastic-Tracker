# 벤치마크 — 궤적 최적화 정식화 비교

`docs/08-zju-fast-lab-family.md` §8.8 과 `docs/lineage.html` 08절의 근거 코드입니다.

## 무엇을 비교하나

| | 구현 | 격리하는 것 |
|---|---|---|
| **A. MINCO + SFC** | **원본 `traj_opt_fake.cc` 를 그대로 링크** | 기준선 |
| **B. 시간만 균일** | A의 웨이포인트·총시간 동일, 배분만 균일 | **시간 배분** |
| **C. 웨이포인트 고정** | A* 경로점 통과, 회랑 없음 | **웨이포인트 자유도** |
| **D. B-스플라인 + 앵커** | ego-planner 정식화 자체 구현 | **궤적 표현** |

프런트엔드(A*)와 회랑(SFC)은 공유하므로 차이는 백엔드에서만 옵니다.
네 방법 모두 동역학 한계(`vmax=3`, `amax=6`)에 붙도록 시간을 스케일합니다 —
`∫jerk²` 는 총 시간에 매우 민감해 T가 다르면 비교가 무의미해집니다.

## 어떻게 원본 코드를 쓰나

`minco.hpp` · `lbfgs_raw.hpp` · `geoutils.hpp` · `sdlp.hpp` · `quickhull.hpp` ·
`root_finder.hpp` · `poly_traj_utils.hpp` 는 **ROS 의존이 0** 입니다.
`traj_opt.cc` / `traj_opt_fake.cc` 도 `nh.getParam` 호출과 `ROS_ERROR` 만 벗겨 내면
그대로 컴파일됩니다. 이 디렉터리의 `traj_opt.{h,cc}` · `traj_opt_fake.cc` 가 그 결과이며,
로직은 원본과 동일합니다 (파라미터를 `TrajOptCfg` 구조체로 받는 것만 다릅니다).

## 빌드 · 실행

```bash
cd docs/bench
INC=../../src/planning/traj_opt/include
g++ -O2 -std=c++14 -c traj_opt.cc      -I. -I$INC -I/usr/include/eigen3 -o traj_opt.o
g++ -O2 -std=c++14 -c traj_opt_fake.cc -I. -I$INC -I/usr/include/eigen3 -o traj_opt_fake.o
g++ -O2 -std=c++14 bench.cpp traj_opt.o traj_opt_fake.o -I. -I$INC -I/usr/include/eigen3 -o bench
./bench            # results.csv · traj.csv · scene.csv
DBG=1 ./bench      # B-스플라인 앵커 누적 과정 출력
```

ROS 는 필요 없습니다. Eigen3 와 C++14 컴파일러만 있으면 됩니다.

## 한계

- **A만 원본 코드입니다.** B·C는 원본 `minco.hpp` 를 쓰되 조합은 이 벤치마크의 것이고,
  **D는 ego-planner 정식화의 재구현**입니다 — 실제 ego-planner 저장소를 돌린 것이 아닙니다.
  **D의 절대 수치를 ego-planner 의 성능으로 읽으면 안 됩니다.**
- 2D 원판 장애물 · 장면 3개 — 경향을 보여 주는 사례이지 통계적 벤치마크가 아닙니다.
- 원본 저장소 전체(매핑·EKF·리플랜 루프)는 ROS 1 이 필요해 빌드하지 않았습니다.

## 그림 다시 그리기

```bash
python3 plot.py        # scene.csv · traj.csv → bench.png
```

CJK 글꼴이 필요합니다 (`Noto Sans CJK` 계열). 없으면 한글이 깨집니다.

## 대화형 도구 (`docs/lineage.html` 08절)

같은 알고리즘을 브라우저에서 돌리기 위한 **JavaScript 이식본**입니다.

| 파일 | 역할 |
|---|---|
| `planner.js` | `minco.hpp`(밴디드 LU · MinJerkOpt s=3) · L-BFGS · A* · SFC · 네 방법 · 지표 — **평면(2D)** |
| `lab.js` | 캔버스 렌더링 · 드래그 · 다시 풀기 |
| `lab.css` · `lab.html` | 위젯의 스타일과 마크업 |
| `verify.js` | 이식본을 `results.csv`(C++)와 대조 |

```bash
node verify.js         # JS 결과 / C++ 결과를 나란히 출력
```

세 장면 기준 편차는 **C 길이·T 0.00%**, D 길이 0.25% · T 1.9%,
A 길이 0.36% · T 0.51% · v_max 0.05% (∫jerk² 는 라인서치가 달라 최대 22%).
C 가 소수점까지 맞는 것이 이식한 밴디드 행렬·경계 조건이 옳다는 근거입니다.

## 알려진 결함 하나 (고침)

`corridor()` 는 잘라 낸 장애물을 `used` 로 표시하지 않아, 접평면 위의 지지점이
`inside()` 판정에서 계속 "안"으로 나오면서 **같은 평면만 24번 추가**하고
나머지 장애물은 회랑 안에 그대로 남겼습니다. 2024-09-10 에 고쳤고,
현재 `results.csv` · `bench.png` 는 고친 뒤 값입니다.
