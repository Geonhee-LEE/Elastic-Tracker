// ============================================================================
//  ZJU FAST Lab 플래너 정식화 벤치마크
//
//  A. MINCO + SFC      : Elastic-Tracker 의 traj_opt_fake.cc 원본을 그대로 링크
//  B. MINCO 시간 고정   : A 의 웨이포인트를 그대로 두고 T 만 균일 고정 (시간 변수 효과 격리)
//  C. MINCO 웨이포인트 고정 : A* 경로점을 그대로 통과 · 회랑 없음 (Mellinger 스타일)
//  D. B-스플라인 + 앵커 : ego-planner 정식화 (자체 구현, 같은 L-BFGS 사용)
//
//  프런트엔드(A*)와 회랑(SFC)은 A·B·D 가 공유한다 → 차이는 백엔드에서만 온다
// ============================================================================
#include "traj_opt.h"
#include <traj_opt/lbfgs_raw.hpp>
#include <traj_opt/geoutils.hpp>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

using Vec3 = Eigen::Vector3d;
static const double ZH = 0.40;   // 평면 운동: z 를 ±0.4 로 가둔다

// ------------------------------------------------------------------ 장면
struct Disc { double x, y, r; };
struct Scene {
  std::vector<Disc> obs;
  Vec3 start, goal;
  double xmin, xmax, ymin, ymax;
};

static double distToObs(const Scene& S, const Vec3& p) {   // 표면까지 거리(음수=관통)
  double d = 1e9;
  for (const auto& o : S.obs)
    d = std::min(d, std::hypot(p.x() - o.x, p.y() - o.y) - o.r);
  return d;
}

static Scene makeScene(int which) {
  Scene S; S.xmin = 0; S.xmax = 22; S.ymin = 0; S.ymax = 14;
  S.start = Vec3(1.5, 7.0, 0); S.goal = Vec3(20.5, 7.0, 0);
  if (which == 0) {            // 성긴 숲
    S.obs = {{5,7,1.1},{9,4.2,1.3},{9,10.0,1.3},{13,7,1.5},{17,4.5,1.2},{17,9.5,1.2}};
  } else if (which == 1) {     // 좁은 통로
    S.obs = {{7,3.0,2.6},{7,11.0,2.6},{13,3.2,2.6},{13,10.8,2.6},{18,7,1.0}};
  } else {                     // 조밀
    S.obs = {{4.5,5.0,.95},{4.5,9.0,.95},{7.5,7.0,.95},{7.5,3.0,.95},{7.5,11.0,.95},
             {10.5,5.0,.95},{10.5,9.0,.95},{13.5,7.0,.95},{13.5,3.2,.95},{13.5,10.8,.95},
             {16.5,5.0,.95},{16.5,9.0,.95}};
  }
  return S;
}

// ------------------------------------------------------------------ A* 프런트엔드
static bool astar(const Scene& S, double res, double infl, std::vector<Vec3>& path) {
  int nx = int((S.xmax - S.xmin) / res), ny = int((S.ymax - S.ymin) / res);
  auto occ = [&](int i, int j) {
    Vec3 p(S.xmin + (i + .5) * res, S.ymin + (j + .5) * res, 0);
    return distToObs(S, p) < infl;
  };
  auto id = [&](int i, int j) { return j * nx + i; };
  int si = int((S.start.x() - S.xmin) / res), sj = int((S.start.y() - S.ymin) / res);
  int gi = int((S.goal.x() - S.xmin) / res),  gj = int((S.goal.y() - S.ymin) / res);
  std::vector<double> g(nx * ny, 1e18);
  std::vector<int> par(nx * ny, -1);
  std::vector<char> closed(nx * ny, 0);
  auto h = [&](int i, int j) { return std::hypot(i - gi, j - gj) * res; };
  std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>,
                      std::greater<>> open;
  g[id(si,sj)] = 0; open.push({h(si,sj), id(si,sj)});
  const int dx[8]={1,-1,0,0,1,1,-1,-1}, dy[8]={0,0,1,-1,1,-1,1,-1};
  while (!open.empty()) {
    int c = open.top().second; open.pop();
    if (closed[c]) continue; closed[c] = 1;
    int ci = c % nx, cj = c / nx;
    if (ci == gi && cj == gj) {
      std::vector<Vec3> rev;
      for (int k = c; k != -1; k = par[k])
        rev.push_back(Vec3(S.xmin + (k % nx + .5) * res, S.ymin + (k / nx + .5) * res, 0));
      path.assign(rev.rbegin(), rev.rend());
      path.front() = S.start; path.back() = S.goal;
      return true;
    }
    for (int k = 0; k < 8; ++k) {
      int ni = ci + dx[k], nj = cj + dy[k];
      if (ni < 0 || nj < 0 || ni >= nx || nj >= ny || occ(ni, nj)) continue;
      double ng = g[c] + std::hypot(dx[k], dy[k]) * res;
      if (ng < g[id(ni,nj)]) {
        g[id(ni,nj)] = ng; par[id(ni,nj)] = c;
        open.push({ng + h(ni,nj), id(ni,nj)});
      }
    }
  }
  return false;
}

// ------------------------------------------------------------------ 안전 회랑
// 선분 주변 박스에서 시작해, 안에 남은 장애물마다 분리 평면을 잘라 넣는다 (IRIS/FIRI 계열)
static const int SFC_BASE_FACES = 6;   // 앞 6면은 박스(4)+z(2), 그 뒤가 장애물 접평면
static Eigen::MatrixXd corridor(const Scene& S, const Vec3& a, const Vec3& b, double bw) {
  std::vector<std::pair<Vec3,Vec3>> H;      // (n, p),  n·(x-p) <= 0
  Vec3 d = b - a; double L = d.norm();
  Vec3 u = L > 1e-9 ? (d / L).eval() : Vec3(1,0,0);
  Vec3 w(-u.y(), u.x(), 0);
  Vec3 c = 0.5 * (a + b);
  H.push_back({ u, b + u * bw}); H.push_back({-u, a - u * bw});
  H.push_back({ w, c + w * bw}); H.push_back({-w, c - w * bw});
  H.push_back({Vec3(0,0,1), Vec3(0,0,ZH)}); H.push_back({Vec3(0,0,-1), Vec3(0,0,-ZH)});
  auto inside = [&](const Vec3& p) {
    for (auto& hp : H) if (hp.first.dot(p - hp.second) > 1e-9) return false;
    return true;
  };
  // 잘라 낸 장애물은 used 로 표시한다.  표시하지 않으면 접평면 위의 지지점이
  // inside() 판정에서 여전히 "안"으로 나와 같은 평면만 반복 추가되고,
  // 나머지 장애물이 회랑 안에 그대로 남는다.
  std::vector<char> used(S.obs.size(), 0);
  for (size_t it = 0; it < S.obs.size(); ++it) {
    double worst = -1e9; int wi = -1; Vec3 wn, wp;
    for (size_t i = 0; i < S.obs.size(); ++i) {
      if (used[i]) continue;
      const auto& o = S.obs[i];
      Vec3 oc(o.x, o.y, 0);
      double t = std::max(0.0, std::min(L, (oc - a).dot(u)));
      Vec3 s = a + u * t;                       // 선분 위 최근접점
      Vec3 dv = oc - s; double dn = dv.norm();
      if (dn < 1e-9) continue;
      Vec3 n = dv / dn;
      Vec3 sp = oc - n * o.r;                   // 원판의 지지평면
      if (!inside(sp)) continue;                // 이미 다른 면에 의해 잘려나갔다
      double pen = -(dn - o.r);                 // 클수록 침범
      if (pen > worst) { worst = pen; wi = (int)i; wn = n; wp = sp; }
    }
    if (wi < 0) break;
    used[wi] = 1;
    H.push_back({wn, wp});
  }
  Eigen::MatrixXd P(6, H.size());
  for (size_t i = 0; i < H.size(); ++i) { P.col(i).head(3) = H[i].first; P.col(i).tail(3) = H[i].second; }
  return P;
}

static void buildSFC(const Scene& S, const std::vector<Vec3>& path, double bw,
                     std::vector<Eigen::MatrixXd>& hPolys, std::vector<Vec3>& keys) {
  hPolys.clear(); keys.clear();
  size_t i = 0;
  auto rayFree = [&](const Vec3& a, const Vec3& b) {
    int n = std::max(2, int((b - a).norm() / 0.1));
    for (int k = 0; k <= n; ++k)
      if (distToObs(S, a + (b - a) * (double(k) / n)) < 0.35) return false;
    return (b - a).norm() <= bw * 1.6;
  };
  while (i + 1 < path.size()) {
    size_t j = i;
    while (j + 1 < path.size() && rayFree(path[i], path[j + 1])) ++j;
    if (j == i) j = i + 1;
    hPolys.push_back(corridor(S, path[i], path[j], bw));
    keys.push_back(path[i]);
    i = j;
  }
  keys.push_back(path.back());
}

// ------------------------------------------------------------------ 지표
struct Metric {
  std::string name;
  bool ok = false;
  double len = 0, jerk2 = 0, snap2 = 0, minClr = 1e9, corrViol = 0, penViol = 0, penViolObs = 0;
  double vmax = 0, amax = 0, T = 0, ms = 0;
  int pieces = 0;
};

static void measure(Metric& m, const Trajectory& tr, const Scene& S,
                    const std::vector<Eigen::MatrixXd>* hPolys) {
  const double T = tr.getTotalDuration();
  m.T = T; m.pieces = tr.getPieceNum();
  const int NS = 4000; const double dt = T / NS;
  Vec3 prev = tr.getPos(0);
  for (int k = 0; k <= NS; ++k) {
    double t = std::min(T, k * dt);
    Vec3 p = tr.getPos(t);
    if (k) m.len += (p - prev).norm();
    prev = p;
    m.jerk2 += tr.getJer(t).squaredNorm() * dt;
    m.minClr = std::min(m.minClr, distToObs(S, p));
    if (hPolys && !hPolys->empty()) {
      // 이 시각이 속한 조각 → 회랑 (조각 2개당 회랑 1개)
      double acc = 0; int pc = 0;
      for (int q = 0; q < tr.getPieceNum(); ++q) {
        if (t <= acc + tr[q].getDuration() || q == tr.getPieceNum() - 1) { pc = q; break; }
        acc += tr[q].getDuration();
      }
      int hi = std::min((int)hPolys->size() - 1, pc / 2);
      const auto& H = (*hPolys)[hi];
      double v = -1e9, vo = -1e9;
      for (int c = 0; c < H.cols(); ++c) {
        double d = H.col(c).head(3).dot(p - H.col(c).tail(3));
        v = std::max(v, d);
        if (c >= SFC_BASE_FACES) vo = std::max(vo, d);   // 장애물에서 나온 면만
      }
      m.corrViol = std::max(m.corrViol, v);          // 실제 회랑 면 기준
      m.penViol  = std::max(m.penViol, v + 0.2);     // 페널티가 보는 경계 (clearance_d = 0.2)
      m.penViolObs = std::max(m.penViolObs, vo + 0.2);  // 그중 장애물 접평면 기준
    }
  }
  // 스냅은 수치 미분(저크의 도함수)
  for (int k = 1; k < NS; ++k) {
    double t = k * dt;
    Vec3 s = (tr.getJer(std::min(T, t + dt)) - tr.getJer(std::max(0.0, t - dt))) / (2 * dt);
    m.snap2 += s.squaredNorm() * dt;
  }
  m.vmax = tr.getMaxVelRate();      // RootFinder 기반 — 샘플링 아님
  m.amax = tr.getMaxAccRate();
  m.ok = true;
}

// ------------------------------------------------------------------ C. 웨이포인트 고정 MINCO
static bool minJerkWaypoint(const std::vector<Vec3>& wps, double vnom,
                            Trajectory& tr) {
  int N = (int)wps.size() - 1;
  if (N < 2) return false;
  Eigen::MatrixXd P(3, N - 1);
  for (int i = 0; i < N - 1; ++i) P.col(i) = wps[i + 1];
  Eigen::VectorXd T(N);
  for (int i = 0; i < N; ++i) T(i) = std::max(0.05, (wps[i + 1] - wps[i]).norm() / vnom);
  Eigen::Matrix3d h = Eigen::Matrix3d::Zero(), t = Eigen::Matrix3d::Zero();
  h.col(0) = wps.front(); t.col(0) = wps.back();
  minco::MinJerkOpt op; op.reset(h, t, N); op.generate(P, T);
  tr = op.getTraj();
  return true;
}

// ------------------------------------------------------------------ D. B-스플라인 + {p,v} 앵커
struct BsplineOpt {
  int n;                       // 제어점 개수
  double dt;                   // 균일 노트 간격
  Eigen::MatrixXd Q;           // 3 x n
  const Scene* S;
  double vmaxc, amaxc, safe;
  double wS = 1.0, wC = 1.0e4, wF = 1.0e2;
  std::vector<Vec3> guide;
  // 충돌 앵커 {p, v}
  std::vector<std::vector<std::pair<Vec3,Vec3>>> anchors;

  void buildAnchors() {
    // ego-planner: 새로 부딪힌 제어점에 대해서만 {p, v} 한 쌍을 만들어 **누적**한다.
    // p 는 충돌점에서 가이드 경로 쪽으로 걸어 나가 자유 공간에 처음 닿는 점,
    // v 는 그 방향. 거리장(ESDF)을 만들지 않는다.
    if ((int)anchors.size() != n) anchors.assign(n, {});
    for (int i = 3; i + 3 < n; ++i) {
      Vec3 q = Q.col(i);
      if (distToObs(*S, q) >= safe) continue;
      // 제어점 i 에 대응하는 가이드 점 (인덱스 비례)
      size_t gi = (size_t)std::llround(double(i - 3) / std::max(1, n - 7) * (guide.size() - 1));
      gi = std::min(gi, guide.size() - 1);
      Vec3 g = guide[gi];
      Vec3 dir = g - q; dir.z() = 0;
      if (dir.norm() < 1e-6) {                 // 가이드 위에 이미 있으면 장애물 반대 방향
        double bd = 1e18; Vec3 oc(0,0,0);
        for (const auto& o : S->obs) { double d = std::hypot(q.x()-o.x, q.y()-o.y) - o.r;
          if (d < bd) { bd = d; oc = Vec3(o.x, o.y, 0); } }
        dir = q - oc; dir.z() = 0;
        if (dir.norm() < 1e-6) dir = Vec3(0, 1, 0);
      }
      dir.normalize();
      // q 에서 dir 방향으로 걸어 나가 자유 공간에 처음 닿는 점 p
      Vec3 pp = q; bool found = false;
      for (double s2 = 0.05; s2 <= 6.0; s2 += 0.05) {
        Vec3 c2 = q + dir * s2;
        if (distToObs(*S, c2) >= safe) { pp = c2; found = true; break; }
      }
      if (!found) continue;
      // 중복 앵커 억제
      bool dup = false;
      for (auto& a : anchors[i]) if ((a.first - pp).norm() < 0.15 && a.second.dot(dir) > 0.95) dup = true;
      if (!dup) anchors[i].push_back({pp, dir});
    }
  }

  static double obj(void* inst, const double* x, double* grad, const int nv) {
    BsplineOpt& B = *(BsplineOpt*)inst;
    Eigen::Map<const Eigen::MatrixXd> Q(x, 3, B.n);
    Eigen::Map<Eigen::MatrixXd> G(grad, 3, B.n);
    G.setZero();
    double f = 0;
    // 평활도: 3차 차분 제곱합 (저크 대응)
    for (int i = 0; i + 3 < B.n; ++i) {
      Vec3 d = Q.col(i+3) - 3*Q.col(i+2) + 3*Q.col(i+1) - Q.col(i);
      f += B.wS * d.squaredNorm();
      G.col(i+3) += 2*B.wS*d;  G.col(i+2) += -6*B.wS*d;
      G.col(i+1) +=  6*B.wS*d; G.col(i)   += -2*B.wS*d;
    }
    // 충돌: {p, v} 앵커  pen = safe - (Q_i - p)·v
    for (int i = 0; i < B.n; ++i)
      for (auto& a : B.anchors[i]) {
        double pen = B.safe - (Q.col(i) - a.first).dot(a.second);
        if (pen > 0) { f += B.wC*pen*pen*pen; G.col(i) += -3*B.wC*pen*pen*a.second; }
      }
    // 동역학: 제어점 차분에 대한 볼록포 성질 이용
    const double p3 = 3.0;
    for (int i = 0; i + 1 < B.n; ++i) {
      Vec3 v = (Q.col(i+1) - Q.col(i)) * (p3 / B.dt);
      double e = v.squaredNorm() - B.vmaxc*B.vmaxc;
      if (e > 0) { f += B.wF*e*e*e;
        Vec3 g = B.wF*6*e*e*v*(p3/B.dt);
        G.col(i+1) += g; G.col(i) -= g; }
    }
    for (int i = 0; i + 2 < B.n; ++i) {
      Vec3 a = (Q.col(i+2) - 2*Q.col(i+1) + Q.col(i)) * (p3*(p3-1)/(B.dt*B.dt));
      double e = a.squaredNorm() - B.amaxc*B.amaxc;
      if (e > 0) { f += B.wF*e*e*e;
        Vec3 g = B.wF*6*e*e*a*(p3*(p3-1)/(B.dt*B.dt));
        G.col(i+2) += g; G.col(i+1) -= 2*g; G.col(i) += g; }
    }
    // 시작·끝 3점 고정 (경계 조건)
    for (int i = 0; i < 3; ++i) { G.col(i).setZero(); G.col(B.n-1-i).setZero(); }
    return f;
  }

  // 균일 3차 B-스플라인 → 5차 다항식 조각으로 변환 (Trajectory 로 지표 공유)
  Trajectory toTraj() const {
    std::vector<double> dur;
    std::vector<CoefficientMat> cs;
    // 구간 [i, i+1) : 제어점 Q_{i-3..i} (0<=i-3, i<n)
    for (int i = 3; i < n; ++i) {
      Eigen::Matrix<double,4,3> P;
      P.row(0) = Q.col(i-3).transpose(); P.row(1) = Q.col(i-2).transpose();
      P.row(2) = Q.col(i-1).transpose(); P.row(3) = Q.col(i).transpose();
      // 균일 3차 기저 행렬 / 6
      Eigen::Matrix4d M;
      M <<  1, -3,  3, -1,
            4,  0, -6,  3,
            1,  3,  3, -3,
            0,  0,  0,  1;
      M /= 6.0;
      Eigen::Matrix<double,4,3> C = M.transpose() * P;   // [c0;c1;c2;c3] (u 정규화)
      CoefficientMat cm; cm.setZero();
      // Trajectory 는 5차, 계수 순서 [t^5 ... t^0], 실시간 t 기준으로 스케일
      for (int d = 0; d < 3; ++d) {
        double c0=C(0,d), c1=C(1,d), c2=C(2,d), c3=C(3,d);
        cm(d,5)=c0; cm(d,4)=c1/dt; cm(d,3)=c2/(dt*dt); cm(d,2)=c3/(dt*dt*dt);
        cm(d,1)=0;  cm(d,0)=0;
      }
      cs.push_back(cm); dur.push_back(dt);
    }
    return Trajectory(dur, cs);
  }
};

// ------------------------------------------------------------------ 출력
static void row(const Metric& m) {
  printf("%-22s %6.2f %9.1f %10.1f %8.3f %9.4f %9.4f %9.4f %6.2f %6.2f %6.2f %7.1f %3d\n",
         m.name.c_str(), m.len, m.jerk2, m.snap2, m.minClr, m.corrViol, m.penViol, m.penViolObs,
         m.vmax, m.amax, m.T, m.ms, m.pieces);
}

int main(int argc, char** argv) {
  const char* names[3] = {"성긴 숲", "좁은 통로", "조밀"};
  FILE* csv = fopen("results.csv", "w");
  fprintf(csv, "scene,method,len,jerk2,snap2,minClr,corrViol,penViol,penViolObs,vmax,amax,T,ms,pieces\n");
  FILE* trj = fopen("traj.csv", "w");
  fprintf(trj, "scene,method,t,x,y\n");
  FILE* env = fopen("scene.csv", "w");
  fprintf(env, "scene,kind,a,b,c\n");

  TrajOptCfg cfg;
  cfg.vmax = 3.0; cfg.amax = 6.0; cfg.K = 8;
  cfg.rhoT = 100; cfg.rhoP = 10000; cfg.rhoV = 1000; cfg.rhoA = 1000;
  cfg.clearance_d = 0.2;

  for (int sc = 0; sc < 3; ++sc) {
    Scene S = makeScene(sc);
    for (auto& o : S.obs) fprintf(env, "%d,obs,%.3f,%.3f,%.3f\n", sc, o.x, o.y, o.r);
    fprintf(env, "%d,start,%.3f,%.3f,0\n", sc, S.start.x(), S.start.y());
    fprintf(env, "%d,goal,%.3f,%.3f,0\n", sc, S.goal.x(), S.goal.y());

    std::vector<Vec3> path;
    if (!astar(S, 0.20, 0.55, path)) { printf("[%d] A* 실패\n", sc); continue; }
    for (auto& p : path) fprintf(env, "%d,path,%.3f,%.3f,0\n", sc, p.x(), p.y());

    std::vector<Eigen::MatrixXd> hPolys; std::vector<Vec3> keys;
    buildSFC(S, path, 1.6, hPolys, keys);

    printf("\n================ 장면 %d : %s  (장애물 %zu · A* %zu점 · 회랑 %zu개) ================\n",
           sc, names[sc], S.obs.size(), path.size(), hPolys.size());
    printf("%-22s %6s %9s %10s %8s %9s %9s %9s %6s %6s %6s %7s %3s\n",
           "방법", "길이", "∫jerk²", "∫snap²", "최소여유", "회랑위배", "여유면위배", "장애물면", "vmax", "amax", "T", "ms", "N");

    Eigen::MatrixXd ini(3,3), fin(3,3); ini.setZero(); fin.setZero();
    ini.col(0) = S.start; fin.col(0) = S.goal;

    std::vector<Metric> ms;

    // ---------- A. MINCO + SFC (원본 코드) ----------
    Trajectory trA;
    {
      traj_opt::TrajOpt opt(cfg);
      auto t0 = std::chrono::high_resolution_clock::now();
      bool ok = opt.generate_traj(ini, fin, hPolys, trA);
      auto t1 = std::chrono::high_resolution_clock::now();
      Metric m; m.name = "A. MINCO+SFC (원본)";
      m.ms = std::chrono::duration<double,std::milli>(t1-t0).count();
      if (ok) { measure(m, trA, S, &hPolys); }
      ms.push_back(m); row(m);
      if (ok) for (int k=0;k<=600;++k){ double t=trA.getTotalDuration()*k/600.0;
        Vec3 p=trA.getPos(t); fprintf(trj,"%d,A,%.4f,%.4f,%.4f\n",sc,t,p.x(),p.y()); }
    }

    // ---------- B. A 의 웨이포인트 + 시간 균일 고정 ----------
    if (trA.getPieceNum() > 0) {
      std::vector<Vec3> wp; wp.push_back(S.start);
      double acc = 0;
      for (int q = 0; q + 1 < trA.getPieceNum(); ++q) { acc += trA[q].getDuration(); wp.push_back(trA.getPos(acc)); }
      wp.push_back(S.goal);
      int N = (int)wp.size() - 1;
      Eigen::MatrixXd P(3, N-1);
      for (int i=0;i<N-1;++i) P.col(i)=wp[i+1];
      Eigen::VectorXd T(N); T.setConstant(trA.getTotalDuration()/N);   // A 와 총 시간 동일
      Eigen::Matrix3d h=Eigen::Matrix3d::Zero(), t=Eigen::Matrix3d::Zero();
      h.col(0)=S.start; t.col(0)=S.goal;
      auto t0=std::chrono::high_resolution_clock::now();
      minco::MinJerkOpt op; op.reset(h,t,N); op.generate(P,T);
      auto t1=std::chrono::high_resolution_clock::now();
      Trajectory tr=op.getTraj();
      Metric m; m.name="B. 같은점·시간 균일고정";
      m.ms=std::chrono::duration<double,std::milli>(t1-t0).count();
      measure(m, tr, S, &hPolys); ms.push_back(m); row(m);
      for (int k=0;k<=600;++k){ double tt=tr.getTotalDuration()*k/600.0;
        Vec3 p=tr.getPos(tt); fprintf(trj,"%d,B,%.4f,%.4f,%.4f\n",sc,tt,p.x(),p.y()); }
    }

    // ---------- C. A* 경로점 고정 · 회랑 없음 ----------
    {
      std::vector<Vec3> wp; wp.push_back(S.start);
      int stride = std::max(1, (int)path.size()/12);
      for (size_t i=stride;i+stride<path.size();i+=stride) wp.push_back(path[i]);
      wp.push_back(S.goal);
      Trajectory tr;
      auto t0=std::chrono::high_resolution_clock::now();
      bool ok=minJerkWaypoint(wp, cfg.vmax, tr);
      auto t1=std::chrono::high_resolution_clock::now();
      if (ok) {   // C 도 동역학 한계에 붙여 공정 비교
        double v0=tr.getMaxVelRate(), a0=tr.getMaxAccRate();
        double k_=std::max(v0/cfg.vmax, std::sqrt(std::max(0.0,a0/cfg.amax)));
        if (k_>0) { std::vector<double> du; std::vector<CoefficientMat> cm;
          for (int q=0;q<tr.getPieceNum();++q){ du.push_back(tr[q].getDuration()*k_);
            CoefficientMat c=tr[q].getCoeffMat();
            for (int d=0;d<3;++d) for (int e=0;e<6;++e) c(d,e)/=std::pow(k_,5-e);
            cm.push_back(c); }
          tr=Trajectory(du,cm); }
      }
      Metric m; m.name="C. 웨이포인트 고정";
      m.ms=std::chrono::duration<double,std::milli>(t1-t0).count();
      if (ok) measure(m, tr, S, nullptr);
      ms.push_back(m); row(m);
      if (ok) for (int k=0;k<=600;++k){ double tt=tr.getTotalDuration()*k/600.0;
        Vec3 p=tr.getPos(tt); fprintf(trj,"%d,C,%.4f,%.4f,%.4f\n",sc,tt,p.x(),p.y()); }
    }

    // ---------- D. B-스플라인 + {p,v} 앵커 ----------
    {
      BsplineOpt B; B.S=&S; B.vmaxc=cfg.vmax; B.amaxc=cfg.amax; B.safe=0.55; B.guide=path;
      int stride = std::max(1,(int)path.size()/16);
      std::vector<Vec3> cp;
      for (int i=0;i<3;++i) cp.push_back(S.start);
      for (size_t i=stride;i+stride<path.size();i+=stride) cp.push_back(path[i]);
      for (int i=0;i<3;++i) cp.push_back(S.goal);
      B.n=(int)cp.size(); B.Q.resize(3,B.n);
      for (int i=0;i<B.n;++i) B.Q.col(i)=cp[i];
      double maxSeg = 0;
      for (int i=0;i+1<B.n;++i) maxSeg = std::max(maxSeg, (B.Q.col(i+1)-B.Q.col(i)).norm());
      B.dt = std::max(0.15, maxSeg * 3.0 / cfg.vmax);   // 초기 제어점 속도 ≤ vmax

      auto t0=std::chrono::high_resolution_clock::now();
      lbfgs::lbfgs_parameter_t pr; lbfgs::lbfgs_load_default_parameters(&pr);
      pr.mem_size=16; pr.past=3; pr.g_epsilon=1e-8; pr.delta=1e-5; pr.max_iterations=400;
      double fx;
      B.anchors.assign(B.n, {});
      for (int outer=0; outer<12; ++outer) {   // 앵커 누적 ↔ 최적화 반복
        B.buildAnchors();
        int na=0; for (auto& v : B.anchors) na += (int)v.size();
        int r = lbfgs::lbfgs_optimize(3*B.n, B.Q.data(), &fx, &BsplineOpt::obj, nullptr, nullptr, &B, &pr);
        double mc=1e9; for (int i=0;i<B.n;++i) mc=std::min(mc, distToObs(S, B.Q.col(i)));
        if (getenv("DBG")) fprintf(stderr,"   [D outer %2d] 앵커 %3d  lbfgs=%3d  f=%.3e  minCP여유=%.3f\n",
                                   outer, na, r, fx, mc);
      }
      // 동역학 실현가능성 확보 위한 시간 신축 (ego-planner 의 사후 연장).
      // 다항식은 dt 배율에 대해 속도 ∝ 1/s, 가속도 ∝ 1/s² 로 정확히 스케일되므로
      // 필요한 배율을 닫힌 형태로 구한다 (보수적 볼록포 상한이 아니라 실제 최댓값 기준).
      {
        Trajectory t0_ = B.toTraj();
        double v0 = t0_.getMaxVelRate(), a0 = t0_.getMaxAccRate();
        double s1 = v0 / cfg.vmax, s2 = std::sqrt(std::max(0.0, a0 / cfg.amax));
        double sc_ = std::max(s1, s2);   // 1 보다 작으면 dt 를 줄여 한계에 붙인다
        B.dt *= sc_;
        for (int it = 0; it < 30; ++it) {          // 수치 오차 보정
          Trajectory t_ = B.toTraj();
          if (t_.getMaxVelRate() <= cfg.vmax * 1.001 &&
              t_.getMaxAccRate() <= cfg.amax * 1.001) break;
          B.dt *= 1.01;
        }
      }
      auto t1=std::chrono::high_resolution_clock::now();
      Trajectory tr=B.toTraj();
      Metric m; m.name="D. B-스플라인+앵커";
      m.ms=std::chrono::duration<double,std::milli>(t1-t0).count();
      measure(m, tr, S, nullptr);
      ms.push_back(m); row(m);
      for (int k=0;k<=600;++k){ double tt=tr.getTotalDuration()*k/600.0;
        Vec3 p=tr.getPos(tt); fprintf(trj,"%d,D,%.4f,%.4f,%.4f\n",sc,tt,p.x(),p.y()); }
    }

    for (auto& m : ms)
      fprintf(csv, "%d,%s,%.4f,%.2f,%.2f,%.4f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.2f,%d\n",
              sc, m.name.c_str(), m.len, m.jerk2, m.snap2, m.minClr, m.corrViol, m.penViol, m.penViolObs,
              m.vmax, m.amax, m.T, m.ms, m.pieces);
  }
  fclose(csv); fclose(trj); fclose(env);
  printf("\n결과: results.csv · traj.csv · scene.csv\n");
  return 0;
}
