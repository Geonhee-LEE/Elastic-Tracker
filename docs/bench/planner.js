/* ============================================================================
   MINCO 평면(2D) 이식 — Elastic-Tracker 의 minco.hpp / traj_opt.cc 를 옮긴 것
   원본: MIT (Zhepei Wang) / GPL-3.0 (Elastic-Tracker)
   z 축을 제거한 것 말고는 행렬 구성·비용·그래디언트가 원본과 같습니다.
   ========================================================================== */
'use strict';

/* ---------------------------------------------------------------- 밴디드 LU */
// minco.hpp 의 BandedSystem — N×N, 하한/상한 대역폭 p/q, 피벗 없음
class Banded {
  constructor(n, p, q) {
    this.N = n; this.lo = p; this.up = q;
    this.d = new Float64Array(n * (p + q + 1));
  }
  reset() { this.d.fill(0); }
  idx(i, j) { return (i - j + this.up) * this.N + j; }
  get(i, j) { return this.d[this.idx(i, j)]; }
  set(i, j, v) { this.d[this.idx(i, j)] = v; }
  add(i, j, v) { this.d[this.idx(i, j)] += v; }
  factorizeLU() {
    const N = this.N, lo = this.lo, up = this.up;
    for (let k = 0; k <= N - 2; k++) {
      const iM = Math.min(k + lo, N - 1);
      const cv = this.get(k, k);
      for (let i = k + 1; i <= iM; i++) {
        if (this.get(i, k) !== 0) this.set(i, k, this.get(i, k) / cv);
      }
      const jM = Math.min(k + up, N - 1);
      for (let j = k + 1; j <= jM; j++) {
        const c = this.get(k, j);
        if (c !== 0) {
          for (let i = k + 1; i <= iM; i++) {
            const l = this.get(i, k);
            if (l !== 0) this.set(i, j, this.get(i, j) - l * c);
          }
        }
      }
    }
  }
  // b: Float64Array(N*m) 행 우선. Ax=b 를 풀어 b 에 덮어쓴다
  solve(b, m) {
    const N = this.N, lo = this.lo, up = this.up;
    for (let j = 0; j <= N - 1; j++) {
      const iM = Math.min(j + lo, N - 1);
      for (let i = j + 1; i <= iM; i++) {
        const f = this.get(i, j);
        if (f !== 0) for (let k = 0; k < m; k++) b[i * m + k] -= f * b[j * m + k];
      }
    }
    for (let j = N - 1; j >= 0; j--) {
      const dg = this.get(j, j);
      for (let k = 0; k < m; k++) b[j * m + k] /= dg;
      const iM = Math.max(0, j - up);
      for (let i = iM; i <= j - 1; i++) {
        const f = this.get(i, j);
        if (f !== 0) for (let k = 0; k < m; k++) b[i * m + k] -= f * b[j * m + k];
      }
    }
  }
  // AᵀX=b
  solveAdj(b, m) {
    const N = this.N, lo = this.lo, up = this.up;
    for (let j = 0; j <= N - 1; j++) {
      const dg = this.get(j, j);
      for (let k = 0; k < m; k++) b[j * m + k] /= dg;
      const iM = Math.min(j + up, N - 1);
      for (let i = j + 1; i <= iM; i++) {
        const f = this.get(j, i);
        if (f !== 0) for (let k = 0; k < m; k++) b[i * m + k] -= f * b[j * m + k];
      }
    }
    for (let j = N - 1; j >= 0; j--) {
      const iM = Math.max(0, j - lo);
      for (let i = iM; i <= j - 1; i++) {
        const f = this.get(j, i);
        if (f !== 0) for (let k = 0; k < m; k++) b[i * m + k] -= f * b[j * m + k];
      }
    }
  }
}

/* ------------------------------------------------------- MinJerkOpt (s=3, 2D) */
const DIM = 2;
class MinJerkOpt {
  reset(head, tail, N) {          // head/tail: [[px,py],[vx,vy],[ax,ay]]
    this.N = N; this.head = head; this.tail = tail;
    this.A = new Banded(6 * N, 6, 6);
    this.b = new Float64Array(6 * N * DIM);
    this.gdC = new Float64Array(6 * N * DIM);
    this.gdP = new Float64Array((N - 1) * DIM);
    this.gdT = new Float64Array(N);
    this.T1 = new Float64Array(N); this.T2 = new Float64Array(N);
    this.T3 = new Float64Array(N); this.T4 = new Float64Array(N);
    this.T5 = new Float64Array(N);
  }
  // P: Float64Array((N-1)*DIM), ts: Float64Array(N)
  generate(P, ts) {
    const N = this.N, A = this.A, b = this.b;
    const T1 = this.T1, T2 = this.T2, T3 = this.T3, T4 = this.T4, T5 = this.T5;
    for (let i = 0; i < N; i++) {
      T1[i] = ts[i]; T2[i] = T1[i] * T1[i]; T3[i] = T2[i] * T1[i];
      T4[i] = T2[i] * T2[i]; T5[i] = T4[i] * T1[i];
    }
    A.reset(); b.fill(0);
    A.set(0, 0, 1.0); A.set(1, 1, 1.0); A.set(2, 2, 2.0);
    for (let k = 0; k < DIM; k++) {
      b[0 * DIM + k] = this.head[0][k];
      b[1 * DIM + k] = this.head[1][k];
      b[2 * DIM + k] = this.head[2][k];
    }
    for (let i = 0; i < N - 1; i++) {
      const o = 6 * i;
      A.set(o + 3, o + 3, 6.0);
      A.set(o + 3, o + 4, 24.0 * T1[i]);
      A.set(o + 3, o + 5, 60.0 * T2[i]);
      A.set(o + 3, o + 9, -6.0);
      A.set(o + 4, o + 4, 24.0);
      A.set(o + 4, o + 5, 120.0 * T1[i]);
      A.set(o + 4, o + 10, -24.0);
      A.set(o + 5, o, 1.0); A.set(o + 5, o + 1, T1[i]); A.set(o + 5, o + 2, T2[i]);
      A.set(o + 5, o + 3, T3[i]); A.set(o + 5, o + 4, T4[i]); A.set(o + 5, o + 5, T5[i]);
      A.set(o + 6, o, 1.0); A.set(o + 6, o + 1, T1[i]); A.set(o + 6, o + 2, T2[i]);
      A.set(o + 6, o + 3, T3[i]); A.set(o + 6, o + 4, T4[i]); A.set(o + 6, o + 5, T5[i]);
      A.set(o + 6, o + 6, -1.0);
      A.set(o + 7, o + 1, 1.0); A.set(o + 7, o + 2, 2 * T1[i]); A.set(o + 7, o + 3, 3 * T2[i]);
      A.set(o + 7, o + 4, 4 * T3[i]); A.set(o + 7, o + 5, 5 * T4[i]); A.set(o + 7, o + 7, -1.0);
      A.set(o + 8, o + 2, 2.0); A.set(o + 8, o + 3, 6 * T1[i]); A.set(o + 8, o + 4, 12 * T2[i]);
      A.set(o + 8, o + 5, 20 * T3[i]); A.set(o + 8, o + 8, -2.0);
      for (let k = 0; k < DIM; k++) b[(o + 5) * DIM + k] = P[i * DIM + k];
    }
    const n = N - 1, o = 6 * N;
    A.set(o - 3, o - 6, 1.0); A.set(o - 3, o - 5, T1[n]); A.set(o - 3, o - 4, T2[n]);
    A.set(o - 3, o - 3, T3[n]); A.set(o - 3, o - 2, T4[n]); A.set(o - 3, o - 1, T5[n]);
    A.set(o - 2, o - 5, 1.0); A.set(o - 2, o - 4, 2 * T1[n]); A.set(o - 2, o - 3, 3 * T2[n]);
    A.set(o - 2, o - 2, 4 * T3[n]); A.set(o - 2, o - 1, 5 * T4[n]);
    A.set(o - 1, o - 4, 2); A.set(o - 1, o - 3, 6 * T1[n]);
    A.set(o - 1, o - 2, 12 * T2[n]); A.set(o - 1, o - 1, 20 * T3[n]);
    for (let k = 0; k < DIM; k++) {
      b[(o - 3) * DIM + k] = this.tail[0][k];
      b[(o - 2) * DIM + k] = this.tail[1][k];
      b[(o - 1) * DIM + k] = this.tail[2][k];
    }
    A.factorizeLU();
    A.solve(b, DIM);
  }
  // ∫||jerk||² 비용
  jerkCost() {
    const b = this.b, N = this.N;
    const T1 = this.T1, T2 = this.T2, T3 = this.T3, T4 = this.T4, T5 = this.T5;
    let J = 0;
    for (let i = 0; i < N; i++) {
      const o = 6 * i;
      let c33 = 0, c34 = 0, c44 = 0, c35 = 0, c45 = 0, c55 = 0;
      for (let k = 0; k < DIM; k++) {
        const b3 = b[(o + 3) * DIM + k], b4 = b[(o + 4) * DIM + k], b5 = b[(o + 5) * DIM + k];
        c33 += b3 * b3; c34 += b4 * b3; c44 += b4 * b4;
        c35 += b5 * b3; c45 += b5 * b4; c55 += b5 * b5;
      }
      J += 36 * c33 * T1[i] + 144 * c34 * T2[i] + 192 * c44 * T3[i] +
           240 * c35 * T3[i] + 720 * c45 * T4[i] + 720 * c55 * T5[i];
    }
    return J;
  }
  // gdC, gdT 를 ∫jerk² 기여로 초기화 (calGrads_CT)
  calGradsCT() {
    const b = this.b, N = this.N, gdC = this.gdC, gdT = this.gdT;
    const T1 = this.T1, T2 = this.T2, T3 = this.T3, T4 = this.T4, T5 = this.T5;
    gdC.fill(0); gdT.fill(0);
    for (let i = 0; i < N; i++) {
      const o = 6 * i;
      for (let k = 0; k < DIM; k++) {
        const b3 = b[(o + 3) * DIM + k], b4 = b[(o + 4) * DIM + k], b5 = b[(o + 5) * DIM + k];
        gdC[(o + 5) * DIM + k] += 240 * b3 * T3[i] + 720 * b4 * T4[i] + 1440 * b5 * T5[i];
        gdC[(o + 4) * DIM + k] += 144 * b3 * T2[i] + 384 * b4 * T3[i] + 720 * b5 * T4[i];
        gdC[(o + 3) * DIM + k] += 72 * b3 * T1[i] + 144 * b4 * T2[i] + 240 * b5 * T3[i];
      }
      let c33 = 0, c34 = 0, c44 = 0, c35 = 0, c45 = 0, c55 = 0;
      for (let k = 0; k < DIM; k++) {
        const b3 = b[(o + 3) * DIM + k], b4 = b[(o + 4) * DIM + k], b5 = b[(o + 5) * DIM + k];
        c33 += b3 * b3; c34 += b4 * b3; c44 += b4 * b4;
        c35 += b5 * b3; c45 += b5 * b4; c55 += b5 * b5;
      }
      gdT[i] += 36 * c33 + 288 * c34 * T1[i] + 576 * c44 * T2[i] +
                720 * c35 * T2[i] + 2880 * c45 * T3[i] + 3600 * c55 * T4[i];
    }
  }
  // gdC → gdP, gdT (calGrads_PT).  gdC 는 여기서 파괴적으로 변경된다
  calGradsPT() {
    const N = this.N, b = this.b, gdC = this.gdC, gdT = this.gdT, gdP = this.gdP;
    const T1 = this.T1, T2 = this.T2, T3 = this.T3, T4 = this.T4;
    this.A.solveAdj(gdC, DIM);
    gdP.fill(0);
    for (let i = 0; i < N - 1; i++)
      for (let k = 0; k < DIM; k++) gdP[i * DIM + k] += gdC[(6 * i + 5) * DIM + k];

    for (let i = 0; i < N - 1; i++) {
      const o = 6 * i;
      let s = 0;
      for (let k = 0; k < DIM; k++) {
        const b1 = b[(o + 1) * DIM + k], b2 = b[(o + 2) * DIM + k], b3 = b[(o + 3) * DIM + k],
              b4 = b[(o + 4) * DIM + k], b5 = b[(o + 5) * DIM + k];
        const negVel = -(b1 + 2 * T1[i] * b2 + 3 * T2[i] * b3 + 4 * T3[i] * b4 + 5 * T4[i] * b5);
        const negAcc = -(2 * b2 + 6 * T1[i] * b3 + 12 * T2[i] * b4 + 20 * T3[i] * b5);
        const negJer = -(6 * b3 + 24 * T1[i] * b4 + 60 * T2[i] * b5);
        const negSnp = -(24 * b4 + 120 * T1[i] * b5);
        const negCrk = -120 * b5;
        // B1 행 = [negSnp, negCrk, negVel, negVel, negAcc, negJer] 를 gdC[6i+3 .. 6i+8] 과 내적
        s += negSnp * gdC[(o + 3) * DIM + k];
        s += negCrk * gdC[(o + 4) * DIM + k];
        s += negVel * gdC[(o + 5) * DIM + k];
        s += negVel * gdC[(o + 6) * DIM + k];
        s += negAcc * gdC[(o + 7) * DIM + k];
        s += negJer * gdC[(o + 8) * DIM + k];
      }
      gdT[i] += s;
    }
    const n = N - 1, o6 = 6 * N;
    let s = 0;
    for (let k = 0; k < DIM; k++) {
      const b1 = b[(o6 - 5) * DIM + k], b2 = b[(o6 - 4) * DIM + k],
            b3 = b[(o6 - 3) * DIM + k], b4 = b[(o6 - 2) * DIM + k], b5 = b[(o6 - 1) * DIM + k];
      const negVel = -(b1 + 2 * T1[n] * b2 + 3 * T2[n] * b3 + 4 * T3[n] * b4 + 5 * T4[n] * b5);
      const negAcc = -(2 * b2 + 6 * T1[n] * b3 + 12 * T2[n] * b4 + 20 * T3[n] * b5);
      const negJer = -(6 * b3 + 24 * T1[n] * b4 + 60 * T2[n] * b5);
      s += negVel * gdC[(o6 - 3) * DIM + k];
      s += negAcc * gdC[(o6 - 2) * DIM + k];
      s += negJer * gdC[(o6 - 1) * DIM + k];
    }
    gdT[n] += s;
  }
  totalDuration() { let s = 0; for (let i = 0; i < this.N; i++) s += this.T1[i]; return s; }
  // 조각 i 의 국소 시각 s 에서 d 차 미분 (d=0 위치, 1 속도, 2 가속, 3 저크, 4 스냅)
  evalPiece(i, s, d, out) {
    const o = 6 * i, b = this.b;
    for (let k = 0; k < DIM; k++) {
      let v = 0, sp = 1;
      for (let j = d; j < 6; j++) {
        let c = 1;
        for (let q = 0; q < d; q++) c *= (j - q);
        v += c * b[(o + j) * DIM + k] * sp;
        sp *= s;
      }
      out[k] = v;
    }
    // 위 루프는 sp 가 s^(j-d) 여야 하는데 j=d 에서 1 로 시작하므로 맞다
    return out;
  }
  eval(t, d, out) {
    let i = 0, s = t;
    while (i < this.N - 1 && s > this.T1[i]) { s -= this.T1[i]; i++; }
    if (s < 0) s = 0;
    if (s > this.T1[i]) s = this.T1[i];
    return this.evalPiece(i, s, d, out);
  }
}

/* ------------------------------------------------------------------- L-BFGS */
// 무제약 L-BFGS + 약한 Wolfe 백트래킹.  lbfgs_raw.hpp 와 알고리즘 골격이 같고
// 라인서치만 More-Thuente 대신 bracketing 방식이다.
function lbfgs(n, x, fg, opt) {
  opt = opt || {};
  const m = opt.mem || 16, maxIter = opt.maxIter || 400;
  const gEps = opt.gEps || 1e-8, delta = opt.delta || 1e-6, past = opt.past || 3;
  const S = [], Y = [], RHO = [];
  const g = new Float64Array(n), gp = new Float64Array(n), xp = new Float64Array(n);
  const d = new Float64Array(n), q = new Float64Array(n), al = new Float64Array(m);
  let f = fg(x, g);
  const fhist = [f];
  let iter = 0;
  for (; iter < maxIter; iter++) {
    let gn = 0; for (let i = 0; i < n; i++) gn = Math.max(gn, Math.abs(g[i]));
    if (gn < gEps) break;
    q.set(g);
    const k = S.length;
    for (let j = k - 1; j >= 0; j--) {
      let s = 0; for (let i = 0; i < n; i++) s += S[j][i] * q[i];
      al[j] = RHO[j] * s;
      for (let i = 0; i < n; i++) q[i] -= al[j] * Y[j][i];
    }
    let scale = 1;
    if (k > 0) {
      let yy = 0, sy = 0;
      for (let i = 0; i < n; i++) { yy += Y[k - 1][i] * Y[k - 1][i]; sy += S[k - 1][i] * Y[k - 1][i]; }
      if (yy > 0) scale = sy / yy;
    }
    for (let i = 0; i < n; i++) q[i] *= scale;
    for (let j = 0; j < k; j++) {
      let s = 0; for (let i = 0; i < n; i++) s += Y[j][i] * q[i];
      const be = RHO[j] * s;
      for (let i = 0; i < n; i++) q[i] += S[j][i] * (al[j] - be);
    }
    for (let i = 0; i < n; i++) d[i] = -q[i];

    let dg = 0; for (let i = 0; i < n; i++) dg += d[i] * g[i];
    if (dg >= 0) { for (let i = 0; i < n; i++) d[i] = -g[i]; dg = 0; for (let i = 0; i < n; i++) dg -= g[i] * g[i]; S.length = Y.length = RHO.length = 0; }

    xp.set(x); gp.set(g);
    const f0 = f;
    // 약한 Wolfe 조건 (c1=1e-4, c2=0.9) 브래킷 라인서치
    let lo = 0, hi = Infinity, step = (k === 0 ? Math.min(1, 1 / Math.max(1e-12, Math.sqrt(-dg))) : 1);
    let ok = false;
    for (let ls = 0; ls < 48; ls++) {
      for (let i = 0; i < n; i++) x[i] = xp[i] + step * d[i];
      f = fg(x, g);
      if (!isFinite(f)) { hi = step; step = 0.5 * (lo + hi); continue; }
      if (f > f0 + 1e-4 * step * dg) { hi = step; step = 0.5 * (lo + hi); continue; }
      let dg2 = 0; for (let i = 0; i < n; i++) dg2 += d[i] * g[i];
      if (dg2 < 0.9 * dg) { lo = step; step = (hi === Infinity) ? 2 * step : 0.5 * (lo + hi); continue; }
      ok = true; break;
    }
    if (!ok) { x.set(xp); g.set(gp); f = f0; break; }

    const s = new Float64Array(n), y = new Float64Array(n);
    let sy = 0;
    for (let i = 0; i < n; i++) { s[i] = x[i] - xp[i]; y[i] = g[i] - gp[i]; sy += s[i] * y[i]; }
    if (sy > 1e-16) {
      S.push(s); Y.push(y); RHO.push(1 / sy);
      if (S.length > m) { S.shift(); Y.shift(); RHO.shift(); }
    }
    fhist.push(f);
    if (fhist.length > past) {
      const old = fhist[fhist.length - 1 - past];
      if (old - f <= delta * Math.max(1, Math.abs(old))) break;
    }
  }
  return { f, iter };
}

/* ------------------------------------------------------------------ 장면 */
function distToObs(S, x, y) {                     // 표면까지 거리(음수=관통)
  let d = 1e9;
  for (const o of S.obs) d = Math.min(d, Math.hypot(x - o.x, y - o.y) - o.r);
  // 경계벽도 장애물로 본다
  d = Math.min(d, x - S.xmin, S.xmax - x, y - S.ymin, S.ymax - y);
  return d;
}

/* ------------------------------------------------------------- A* 프런트엔드 */
function astar(S, res, infl) {
  const nx = Math.floor((S.xmax - S.xmin) / res), ny = Math.floor((S.ymax - S.ymin) / res);
  const occ = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++)
    occ[j * nx + i] = distToObs(S, S.xmin + (i + .5) * res, S.ymin + (j + .5) * res) < infl ? 1 : 0;
  const si = Math.floor((S.start[0] - S.xmin) / res), sj = Math.floor((S.start[1] - S.ymin) / res);
  const gi = Math.floor((S.goal[0] - S.xmin) / res), gj = Math.floor((S.goal[1] - S.ymin) / res);
  if (si < 0 || sj < 0 || si >= nx || sj >= ny || gi < 0 || gj < 0 || gi >= nx || gj >= ny) return null;
  const g = new Float64Array(nx * ny).fill(1e18);
  const par = new Int32Array(nx * ny).fill(-1);
  const closed = new Uint8Array(nx * ny);
  const h = (i, j) => Math.hypot(i - gi, j - gj) * res;
  // 이진 힙
  const heap = [];
  const push = (k, v) => { heap.push([k, v]); let c = heap.length - 1;
    while (c > 0) { const p = (c - 1) >> 1; if (heap[p][0] <= heap[c][0]) break; const t = heap[p]; heap[p] = heap[c]; heap[c] = t; c = p; } };
  const pop = () => { const top = heap[0], last = heap.pop();
    if (heap.length) { heap[0] = last; let c = 0;
      for (;;) { const l = 2 * c + 1, r = l + 1; let s = c;
        if (l < heap.length && heap[l][0] < heap[s][0]) s = l;
        if (r < heap.length && heap[r][0] < heap[s][0]) s = r;
        if (s === c) break; const t = heap[s]; heap[s] = heap[c]; heap[c] = t; c = s; } }
    return top; };
  const id = (i, j) => j * nx + i;
  g[id(si, sj)] = 0; push(h(si, sj), id(si, sj));
  const dx = [1, -1, 0, 0, 1, 1, -1, -1], dy = [0, 0, 1, -1, 1, -1, 1, -1];
  while (heap.length) {
    const c = pop()[1];
    if (closed[c]) continue; closed[c] = 1;
    const ci = c % nx, cj = (c / nx) | 0;
    if (ci === gi && cj === gj) {
      const rev = [];
      for (let k = c; k !== -1; k = par[k])
        rev.push([S.xmin + ((k % nx) + .5) * res, S.ymin + (((k / nx) | 0) + .5) * res]);
      rev.reverse();
      rev[0] = S.start.slice(); rev[rev.length - 1] = S.goal.slice();
      return rev;
    }
    for (let k = 0; k < 8; k++) {
      const ni = ci + dx[k], nj = cj + dy[k];
      if (ni < 0 || nj < 0 || ni >= nx || nj >= ny || occ[nj * nx + ni]) continue;
      const ng = g[c] + Math.hypot(dx[k], dy[k]) * res;
      if (ng < g[id(ni, nj)]) { g[id(ni, nj)] = ng; par[id(ni, nj)] = c; push(ng + h(ni, nj), id(ni, nj)); }
    }
  }
  return null;
}

/* ------------------------------------------------------------- 안전 회랑 SFC */
// 반평면 n·(x−p) ≤ 0 의 배열.  선분 주변 박스에서 시작해 장애물마다 분리선을 자른다
function corridor(S, a, b, bw) {
  const H = [];
  const dxv = b[0] - a[0], dyv = b[1] - a[1];
  const L = Math.hypot(dxv, dyv);
  const ux = L > 1e-9 ? dxv / L : 1, uy = L > 1e-9 ? dyv / L : 0;
  const wx = -uy, wy = ux;
  const cx = 0.5 * (a[0] + b[0]), cy = 0.5 * (a[1] + b[1]);
  H.push([ux, uy, b[0] + ux * bw, b[1] + uy * bw]);
  H.push([-ux, -uy, a[0] - ux * bw, a[1] - uy * bw]);
  H.push([wx, wy, cx + wx * bw, cy + wy * bw]);
  H.push([-wx, -wy, cx - wx * bw, cy - wy * bw]);
  const inside = (px, py) => {
    for (const h of H) if (h[0] * (px - h[2]) + h[1] * (py - h[3]) > 1e-9) return false;
    return true;
  };
  // 잘라 낸 장애물은 used 로 표시한다.  표시하지 않으면 접평면 위의 지지점이
  // inside() 판정에서 여전히 "안"으로 나와 같은 평면만 반복 추가된다
  const used = new Array(S.obs.length).fill(false);
  for (let it = 0; it < S.obs.length; it++) {
    let worst = -1e9, wi = -1, wnx = 0, wny = 0, wpx = 0, wpy = 0;
    for (let i = 0; i < S.obs.length; i++) {
      if (used[i]) continue;
      const o = S.obs[i];
      let t = (o.x - a[0]) * ux + (o.y - a[1]) * uy;
      t = Math.max(0, Math.min(L, t));
      const sx = a[0] + ux * t, sy = a[1] + uy * t;
      const dvx = o.x - sx, dvy = o.y - sy;
      const dn = Math.hypot(dvx, dvy);
      if (dn < 1e-9) continue;
      const nx2 = dvx / dn, ny2 = dvy / dn;
      const spx = o.x - nx2 * o.r, spy = o.y - ny2 * o.r;
      if (!inside(spx, spy)) continue;        // 이미 다른 면에 의해 잘려 나갔다
      const pen = -(dn - o.r);
      if (pen > worst) { worst = pen; wi = i; wnx = nx2; wny = ny2; wpx = spx; wpy = spy; }
    }
    if (wi < 0) break;
    used[wi] = true;
    H.push([wnx, wny, wpx, wpy]);
  }
  // 장면 경계도 반평면으로 넣는다
  H.push([1, 0, S.xmax, 0]); H.push([-1, 0, S.xmin, 0]);
  H.push([0, 1, 0, S.ymax]); H.push([0, -1, 0, S.ymin]);
  return H;
}

function buildSFC(S, path, bw) {
  const hPolys = [], keys = [];
  const rayFree = (a, b) => {
    const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const n = Math.max(2, Math.floor(L / 0.1));
    for (let k = 0; k <= n; k++) {
      const u = k / n;
      if (distToObs(S, a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u) < 0.35) return false;
    }
    return L <= bw * 1.6;
  };
  let i = 0;
  while (i + 1 < path.length) {
    let j = i;
    while (j + 1 < path.length && rayFree(path[i], path[j + 1])) j++;
    if (j === i) j = i + 1;
    hPolys.push(corridor(S, path[i], path[j], bw));
    keys.push(path[i]);
    i = j;
  }
  keys.push(path[path.length - 1]);
  return { hPolys, keys };
}

/* -------------------------------------------- 2D 반평면 → 볼록 다각형 꼭짓점 */
// geoutils::enumerateVs 의 2D 대응.  큰 상자를 반평면으로 차례로 잘라 낸다
function enumerateVs(H, box) {
  let poly = [[box[0], box[2]], [box[1], box[2]], [box[1], box[3]], [box[0], box[3]]];
  for (const h of H) {
    if (poly.length === 0) return null;
    const out = [];
    const val = p => h[0] * (p[0] - h[2]) + h[1] * (p[1] - h[3]);
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i], b = poly[(i + 1) % poly.length];
      const va = val(a), vb = val(b);
      if (va <= 0) out.push(a);
      if ((va < 0 && vb > 0) || (va > 0 && vb < 0)) {
        const t = va / (va - vb);
        out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
      }
    }
    poly = out;
  }
  if (poly.length < 3) return null;
  // 중복 정리
  const clean = [];
  for (const p of poly) {
    if (!clean.length || Math.hypot(p[0] - clean[clean.length - 1][0], p[1] - clean[clean.length - 1][1]) > 1e-7) clean.push(p);
  }
  if (clean.length > 2 && Math.hypot(clean[0][0] - clean[clean.length - 1][0], clean[0][1] - clean[clean.length - 1][1]) < 1e-7) clean.pop();
  return clean.length >= 3 ? clean : null;
}

// extractVs: 회랑 M개 → 2M−1 개의 (교집합 포함) 꼭짓점 집합
function extractVs(hPolys, box) {
  const M = hPolys.length - 1;
  const vPs = [];
  const toIOB = V => {
    const o = V[0];
    const cols = [[o[0], o[1]]];
    for (let i = 1; i < V.length; i++) cols.push([V[i][0] - o[0], V[i][1] - o[1]]);
    return cols;                       // [origin, off1, off2, ...]
  };
  for (let i = 0; i < M; i++) {
    let V = enumerateVs(hPolys[i], box); if (!V) return null;
    vPs.push(toIOB(V));
    V = enumerateVs(hPolys[i].concat(hPolys[i + 1]), box); if (!V) return null;
    vPs.push(toIOB(V));
  }
  const V = enumerateVs(hPolys[hPolys.length - 1], box); if (!V) return null;
  vPs.push(toIOB(V));
  return vPs;
}

/* ------------------------------------------------- τ↔T, p↔q 매끄러운 전단사 */
const expC2 = t => t > 0 ? ((0.5 * t + 1.0) * t + 1.0) : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
const logC2 = T => T > 1.0 ? (Math.sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - Math.sqrt(2.0 / T - 1.0));
const gdT2t = t => t > 0 ? (t + 1.0) : ((1.0 - t) / Math.pow((0.5 * t - 1.0) * t + 1.0, 2));

function forwardP(p, vPs, P) {
  let j = 0;
  for (let i = 0; i < vPs.length; i++) {
    const V = vPs[i], k = V.length - 1;
    let nsq = 0; for (let a = 0; a < k; a++) nsq += p[j + a] * p[j + a];
    const s = 2.0 / (1.0 + nsq);
    let px = V[0][0], py = V[0][1];
    for (let a = 0; a < k; a++) {
      const q = s * p[j + a], q2 = q * q;
      px += V[a + 1][0] * q2; py += V[a + 1][1] * q2;
    }
    P[i * DIM] = px; P[i * DIM + 1] = py;
    j += k;
  }
}
function addLayerPGrad(p, vPs, gradP, grad) {
  let j = 0;
  for (let i = 0; i < vPs.length; i++) {
    const V = vPs[i], k = V.length - 1;
    let nsq = 0; for (let a = 0; a < k; a++) nsq += p[j + a] * p[j + a];
    const np1 = nsq + 1.0, np1sq = np1 * np1, s = 2.0 / np1;
    const gx = gradP[i * DIM], gy = gradP[i * DIM + 1];
    const gdr = new Float64Array(k);
    let dot = 0;
    for (let a = 0; a < k; a++) {
      const r = s * p[j + a];
      gdr[a] = (V[a + 1][0] * gx + V[a + 1][1] * gy) * r * 2.0;
      dot += gdr[a] * p[j + a];
    }
    for (let a = 0; a < k; a++) grad[j + a] = gdr[a] * s - p[j + a] * 4.0 * dot / np1sq;
    j += k;
  }
}
// backwardP: 주어진 P 를 만드는 p 를 찾는 작은 비선형 최소제곱
function backwardP(P, vPs, p) {
  let j = 0;
  for (let i = 0; i < vPs.length; i++) {
    const V = vPs[i], k = V.length - 1;
    const x = new Float64Array(k).fill(1.0 / (Math.sqrt(k + 1.0) + 1.0));
    const tx = P[i * DIM], ty = P[i * DIM + 1];
    lbfgs(k, x, (xx, g) => {
      let nsq = 0; for (let a = 0; a < k; a++) nsq += xx[a] * xx[a];
      const np1 = nsq + 1.0, np1sq = np1 * np1, s = 2.0 / np1;
      let dx = V[0][0] - tx, dy = V[0][1] - ty;
      const r = new Float64Array(k);
      for (let a = 0; a < k; a++) { r[a] = s * xx[a]; dx += V[a + 1][0] * r[a] * r[a]; dy += V[a + 1][1] * r[a] * r[a]; }
      const cost = dx * dx + dy * dy;
      const g3x = 2 * dx, g3y = 2 * dy;
      const gdr = new Float64Array(k);
      let dot = 0;
      for (let a = 0; a < k; a++) { gdr[a] = (V[a + 1][0] * g3x + V[a + 1][1] * g3y) * r[a] * 2.0; dot += gdr[a] * xx[a]; }
      for (let a = 0; a < k; a++) g[a] = gdr[a] * s - xx[a] * 4.0 * dot / np1sq;
      return cost;
    }, { mem: 8, maxIter: 128, gEps: 1e-12, delta: 1e-14, past: 3 });
    for (let a = 0; a < k; a++) p[j + a] = x[a];
    j += k;
  }
}

/* ------------------------------------------------------- A. MINCO + SFC 최적화 */
const DEF = { K: 8, vmax: 3.0, amax: 6.0, rhoT: 100, rhoP: 10000, rhoV: 1000, rhoA: 1000, clearance: 0.2 };

function optimizeMINCO(hPolys0, start, goal, cfg, box) {
  cfg = Object.assign({}, DEF, cfg || {});
  const hPolys = hPolys0.slice();
  if (hPolys.length === 1) hPolys.push(hPolys[0]);
  const vPs = extractVs(hPolys, box);
  if (!vPs) return null;
  const N = 2 * hPolys.length;
  let dimP = 0; for (const V of vPs) dimP += V.length - 1;
  const dimT = N;

  const jo = new MinJerkOpt();
  jo.reset([[start[0], start[1]], [0, 0], [0, 0]], [[goal[0], goal[1]], [0, 0], [0, 0]], N);

  const T = new Float64Array(N), P = new Float64Array((N - 1) * DIM);
  const T0 = Math.hypot(goal[0] - start[0], goal[1] - start[1]) / cfg.vmax / N;
  T.fill(T0);
  for (let i = 0; i < N - 1; i++) {
    const V = vPs[i], k = V.length - 1;
    let sx = 0, sy = 0;
    for (let a = 1; a <= k; a++) { sx += V[a][0]; sy += V[a][1]; }
    P[i * DIM] = sx / (1 + k) + V[0][0];
    P[i * DIM + 1] = sy / (1 + k) + V[0][1];
  }
  const x = new Float64Array(dimT + dimP);
  for (let i = 0; i < N; i++) x[i] = logC2(T[i]);
  const p0 = new Float64Array(dimP);
  backwardP(P, vPs, p0);
  x.set(p0, dimT);

  const gradP = new Float64Array((N - 1) * DIM);
  const pos = [0, 0], vel = [0, 0], acc = [0, 0], jer = [0, 0];
  const K = cfg.K;

  const fg = (xx, grad) => {
    for (let i = 0; i < N; i++) T[i] = expC2(xx[i]);
    forwardP(xx.subarray(dimT), vPs, P);
    jo.generate(P, T);
    let cost = jo.jerkCost();
    jo.calGradsCT();
    // ---- 시간적분 페널티 (addTimeIntPenalty) ----
    const gdC = jo.gdC, gdT = jo.gdT, b = jo.b;
    for (let i = 0; i < N; i++) {
      const o = 6 * i, step = jo.T1[i] / K;
      const H = hPolys[(i / 2) | 0];
      let s1 = 0;
      for (let j = 0; j <= K; j++) {
        const s2 = s1 * s1, s3 = s2 * s1, s4 = s2 * s2, s5 = s4 * s1;
        const B0 = [1, s1, s2, s3, s4, s5];
        const B1 = [0, 1, 2 * s1, 3 * s2, 4 * s3, 5 * s4];
        const B2 = [0, 0, 2, 6 * s1, 12 * s2, 20 * s3];
        const B3 = [0, 0, 0, 6, 24 * s1, 60 * s2];
        const alpha = j / K;
        for (let k2 = 0; k2 < DIM; k2++) {
          let p0v = 0, v0 = 0, a0 = 0, j0 = 0;
          for (let q = 0; q < 6; q++) {
            const c = b[(o + q) * DIM + k2];
            p0v += c * B0[q]; v0 += c * B1[q]; a0 += c * B2[q]; j0 += c * B3[q];
          }
          pos[k2] = p0v; vel[k2] = v0; acc[k2] = a0; jer[k2] = j0;
        }
        const omg = (j === 0 || j === K) ? 0.5 : 1.0;
        // 회랑
        let gpx = 0, gpy = 0, cp = 0, hit = false;
        for (const h of H) {
          const pen = h[0] * (pos[0] - h[2] + cfg.clearance * h[0]) +
                      h[1] * (pos[1] - h[3] + cfg.clearance * h[1]);
          if (pen > 0) {
            const pen2 = pen * pen;
            gpx += cfg.rhoP * 3 * pen2 * h[0]; gpy += cfg.rhoP * 3 * pen2 * h[1];
            cp += cfg.rhoP * pen2 * pen; hit = true;
          }
        }
        if (hit) {
          for (let q = 0; q < 6; q++) {
            gdC[(o + q) * DIM] += omg * step * B0[q] * gpx;
            gdC[(o + q) * DIM + 1] += omg * step * B0[q] * gpy;
          }
          gdT[i] += omg * (cp / K + step * alpha * (gpx * vel[0] + gpy * vel[1]));
          cost += omg * step * cp;
        }
        // 속도
        const vpen = vel[0] * vel[0] + vel[1] * vel[1] - cfg.vmax * cfg.vmax;
        if (vpen > 0) {
          const gvx = cfg.rhoV * 6 * vpen * vpen * vel[0], gvy = cfg.rhoV * 6 * vpen * vpen * vel[1];
          const cv = cfg.rhoV * vpen * vpen * vpen;
          for (let q = 0; q < 6; q++) {
            gdC[(o + q) * DIM] += omg * step * B1[q] * gvx;
            gdC[(o + q) * DIM + 1] += omg * step * B1[q] * gvy;
          }
          gdT[i] += omg * (cv / K + step * alpha * (gvx * acc[0] + gvy * acc[1]));
          cost += omg * step * cv;
        }
        // 가속
        const apen = acc[0] * acc[0] + acc[1] * acc[1] - cfg.amax * cfg.amax;
        if (apen > 0) {
          const gax = cfg.rhoA * 6 * apen * apen * acc[0], gay = cfg.rhoA * 6 * apen * apen * acc[1];
          const ca = cfg.rhoA * apen * apen * apen;
          for (let q = 0; q < 6; q++) {
            gdC[(o + q) * DIM] += omg * step * B2[q] * gax;
            gdC[(o + q) * DIM + 1] += omg * step * B2[q] * gay;
          }
          gdT[i] += omg * (ca / K + step * alpha * (gax * jer[0] + gay * jer[1]));
          cost += omg * step * ca;
        }
        s1 += step;
      }
    }
    jo.calGradsPT();
    for (let i = 0; i < N; i++) jo.gdT[i] += cfg.rhoT;
    for (let i = 0; i < N; i++) cost += cfg.rhoT * T[i];
    for (let i = 0; i < N; i++) grad[i] = jo.gdT[i] * gdT2t(xx[i]);
    gradP.set(jo.gdP);
    addLayerPGrad(xx.subarray(dimT), vPs, gradP, grad.subarray(dimT));
    return cost;
  };

  const res = lbfgs(dimT + dimP, x, fg, { mem: 16, maxIter: cfg.maxIter || 500, gEps: 1e-9, delta: 1e-5, past: 3 });
  for (let i = 0; i < N; i++) T[i] = expC2(x[i]);
  forwardP(x.subarray(dimT), vPs, P);
  jo.generate(P, T);
  return { traj: jo, P, T, iter: res.iter, f: res.f, hPolys, vPs };
}

/* --------------------------------------------------------- 지표 측정 (measure) */
function measure(jo, S, hPolys, clearance) {
  const M = { len: 0, jerk2: 0, snap2: 0, minClr: 1e9, corrViol: 0, penViol: 0, vmax: 0, amax: 0, T: jo.totalDuration() };
  const NS = 4000, dt = M.T / NS;
  const p = [0, 0], v = [0, 0], a = [0, 0], j = [0, 0], jp = [0, 0], jm = [0, 0];
  let prev = null;
  for (let i = 0; i <= NS; i++) {
    const t = M.T * i / NS;
    jo.eval(t, 0, p); jo.eval(t, 1, v); jo.eval(t, 2, a); jo.eval(t, 3, j);
    if (prev) M.len += Math.hypot(p[0] - prev[0], p[1] - prev[1]);
    prev = [p[0], p[1]];
    const w = (i === 0 || i === NS) ? 0.5 : 1.0;
    M.jerk2 += w * (j[0] * j[0] + j[1] * j[1]) * dt;
    // 스냅은 저크의 수치 미분 — 3차 B-스플라인은 노트에서 저크가 불연속이라
    // 해석 미분으로는 조각 안에서 0 이 되어 그 임펄스를 놓친다 (C++ measure 와 동일)
    if (i > 0 && i < NS) {
      jo.eval(Math.min(M.T, t + dt), 3, jp);
      jo.eval(Math.max(0, t - dt), 3, jm);
      const sx = (jp[0] - jm[0]) / (2 * dt), sy = (jp[1] - jm[1]) / (2 * dt);
      M.snap2 += (sx * sx + sy * sy) * dt;
    }
    M.minClr = Math.min(M.minClr, distToObs(S, p[0], p[1]));
    M.vmax = Math.max(M.vmax, Math.hypot(v[0], v[1]));
    M.amax = Math.max(M.amax, Math.hypot(a[0], a[1]));
    if (hPolys) {
      // C++ measure 와 같게: 시각 → 조각 → 회랑(조각 2개당 1개)
      let accT = 0, pc = 0;
      for (let q = 0; q < jo.N; q++) {
        if (t <= accT + jo.T1[q] || q === jo.N - 1) { pc = q; break; }
        accT += jo.T1[q];
      }
      const idx = Math.min(hPolys.length - 1, pc >> 1);
      let worst = -1e9;
      for (const h of hPolys[idx]) worst = Math.max(worst, h[0] * (p[0] - h[2]) + h[1] * (p[1] - h[3]));
      M.corrViol = Math.max(M.corrViol, worst);
      M.penViol = Math.max(M.penViol, worst + clearance);
    }
  }
  if (M.corrViol < 0) M.corrViol = 0;
  if (M.penViol < 0) M.penViol = 0;
  return M;
}


/* -------------------------------------------------- 시간 신축 · 최대 속도/가속 */
// 계수 b 는 오름차순 t^j.  t' = k·t 로 늘리면 a'_j = a_j / k^j
function scaleTime(jo, k) {
  for (let i = 0; i < jo.N; i++) {
    for (let j = 0; j < 6; j++) {
      const f = Math.pow(k, j);
      for (let d = 0; d < DIM; d++) jo.b[(6 * i + j) * DIM + d] /= f;
    }
    jo.T1[i] *= k;
    jo.T2[i] = jo.T1[i] * jo.T1[i]; jo.T3[i] = jo.T2[i] * jo.T1[i];
    jo.T4[i] = jo.T2[i] * jo.T2[i]; jo.T5[i] = jo.T4[i] * jo.T1[i];
  }
}
function maxRates(jo, NS) {
  NS = NS || 2000;
  const T = jo.totalDuration(), v = [0, 0], a = [0, 0];
  let vm = 0, am = 0;
  for (let i = 0; i <= NS; i++) {
    const t = T * i / NS;
    jo.eval(t, 1, v); jo.eval(t, 2, a);
    vm = Math.max(vm, Math.hypot(v[0], v[1]));
    am = Math.max(am, Math.hypot(a[0], a[1]));
  }
  return [vm, am];
}
function fitLimits(jo, vmax, amax) {          // 동역학 한계에 정확히 붙인다
  const [v0, a0] = maxRates(jo);
  const k = Math.max(v0 / vmax, Math.sqrt(Math.max(0, a0 / amax)));
  if (k > 0 && isFinite(k)) scaleTime(jo, k);
  for (let it = 0; it < 30; it++) {
    const [v1, a1] = maxRates(jo);
    if (v1 <= vmax * 1.001 && a1 <= amax * 1.001) break;
    scaleTime(jo, 1.01);
  }
  return jo;
}

/* --------------------------------- 웨이포인트 통과 min-jerk (B, C 공용) */
function minJerkThrough(wps, T) {
  const N = wps.length - 1;
  if (N < 2) return null;
  const jo = new MinJerkOpt();
  jo.reset([[wps[0][0], wps[0][1]], [0, 0], [0, 0]],
           [[wps[N][0], wps[N][1]], [0, 0], [0, 0]], N);
  const P = new Float64Array((N - 1) * DIM);
  for (let i = 0; i < N - 1; i++) { P[i * DIM] = wps[i + 1][0]; P[i * DIM + 1] = wps[i + 1][1]; }
  jo.generate(P, T);
  return jo;
}

/* ------------------------------------ B. A 의 웨이포인트 + 시간 균일 고정 */
function armB(joA, start, goal) {
  const wp = [start.slice()];
  let acc = 0;
  for (let q = 0; q + 1 < joA.N; q++) { acc += joA.T1[q]; const o = [0, 0]; joA.eval(acc, 0, o); wp.push([o[0], o[1]]); }
  wp.push(goal.slice());
  const N = wp.length - 1;
  const T = new Float64Array(N).fill(joA.totalDuration() / N);   // A 와 총 시간 동일
  return minJerkThrough(wp, T);
}

/* ------------------------------------ C. A* 경로점 고정 · 회랑 없음 */
function armC(path, start, goal, cfg) {
  const stride = Math.max(1, Math.floor(path.length / 12));
  const wp = [start.slice()];
  for (let i = stride; i + stride < path.length; i += stride) wp.push(path[i].slice());
  wp.push(goal.slice());
  const N = wp.length - 1;
  if (N < 2) return null;
  const T = new Float64Array(N);
  for (let i = 0; i < N; i++)
    T[i] = Math.max(0.05, Math.hypot(wp[i + 1][0] - wp[i][0], wp[i + 1][1] - wp[i][1]) / cfg.vmax);
  const jo = minJerkThrough(wp, T);
  if (!jo) return null;
  return fitLimits(jo, cfg.vmax, cfg.amax);
}

/* ------------------------------------ D. B-스플라인 + {p,v} 앵커 (ego-planner) */
function armD(S, path, cfg) {
  const safe = 0.55, wS = 1.0, wC = 1.0e4, wF = 1.0e2, p3 = 3.0;
  const stride = Math.max(1, Math.floor(path.length / 16));
  const cp = [];
  for (let i = 0; i < 3; i++) cp.push(S.start.slice());
  for (let i = stride; i + stride < path.length; i += stride) cp.push(path[i].slice());
  for (let i = 0; i < 3; i++) cp.push(S.goal.slice());
  const n = cp.length;
  if (n < 8) return null;
  const Q = new Float64Array(n * DIM);
  for (let i = 0; i < n; i++) { Q[i * DIM] = cp[i][0]; Q[i * DIM + 1] = cp[i][1]; }
  let maxSeg = 0;
  for (let i = 0; i + 1 < n; i++) maxSeg = Math.max(maxSeg, Math.hypot(Q[(i + 1) * DIM] - Q[i * DIM], Q[(i + 1) * DIM + 1] - Q[i * DIM + 1]));
  let dt = Math.max(0.15, maxSeg * 3.0 / cfg.vmax);
  const anchors = []; for (let i = 0; i < n; i++) anchors.push([]);

  // 앵커: 부딪힌 제어점에서 가이드 쪽으로 걸어 나가 자유 공간에 처음 닿는 점 {p, v}
  const buildAnchors = () => {
    for (let i = 3; i + 3 < n; i++) {
      const qx = Q[i * DIM], qy = Q[i * DIM + 1];
      if (distToObs(S, qx, qy) >= safe) continue;
      let gi = Math.round((i - 3) / Math.max(1, n - 7) * (path.length - 1));
      gi = Math.min(Math.max(gi, 0), path.length - 1);
      let dx = path[gi][0] - qx, dy = path[gi][1] - qy;
      if (Math.hypot(dx, dy) < 1e-6) {
        let bd = 1e18, ox = 0, oy = 0;
        for (const o of S.obs) { const d = Math.hypot(qx - o.x, qy - o.y) - o.r; if (d < bd) { bd = d; ox = o.x; oy = o.y; } }
        dx = qx - ox; dy = qy - oy;
        if (Math.hypot(dx, dy) < 1e-6) { dx = 0; dy = 1; }
      }
      const dn = Math.hypot(dx, dy); dx /= dn; dy /= dn;
      let px = qx, py = qy, found = false;
      for (let s2 = 0.05; s2 <= 6.0; s2 += 0.05) {
        const cx = qx + dx * s2, cy = qy + dy * s2;
        if (distToObs(S, cx, cy) >= safe) { px = cx; py = cy; found = true; break; }
      }
      if (!found) continue;
      let dup = false;
      for (const a of anchors[i]) if (Math.hypot(a[0] - px, a[1] - py) < 0.15 && a[2] * dx + a[3] * dy > 0.95) dup = true;
      if (!dup) anchors[i].push([px, py, dx, dy]);
    }
  };
  const obj = (x, g) => {
    g.fill(0);
    let f = 0;
    for (let i = 0; i + 3 < n; i++) {
      for (let d = 0; d < DIM; d++) {
        const v = x[(i + 3) * DIM + d] - 3 * x[(i + 2) * DIM + d] + 3 * x[(i + 1) * DIM + d] - x[i * DIM + d];
        f += wS * v * v;
        g[(i + 3) * DIM + d] += 2 * wS * v; g[(i + 2) * DIM + d] += -6 * wS * v;
        g[(i + 1) * DIM + d] += 6 * wS * v; g[i * DIM + d] += -2 * wS * v;
      }
    }
    for (let i = 0; i < n; i++) for (const a of anchors[i]) {
      const pen = safe - ((x[i * DIM] - a[0]) * a[2] + (x[i * DIM + 1] - a[1]) * a[3]);
      if (pen > 0) {
        f += wC * pen * pen * pen;
        g[i * DIM] += -3 * wC * pen * pen * a[2];
        g[i * DIM + 1] += -3 * wC * pen * pen * a[3];
      }
    }
    const kv = p3 / dt;
    for (let i = 0; i + 1 < n; i++) {
      const vx = (x[(i + 1) * DIM] - x[i * DIM]) * kv, vy = (x[(i + 1) * DIM + 1] - x[i * DIM + 1]) * kv;
      const e = vx * vx + vy * vy - cfg.vmax * cfg.vmax;
      if (e > 0) {
        f += wF * e * e * e;
        const gx = wF * 6 * e * e * vx * kv, gy = wF * 6 * e * e * vy * kv;
        g[(i + 1) * DIM] += gx; g[i * DIM] -= gx;
        g[(i + 1) * DIM + 1] += gy; g[i * DIM + 1] -= gy;
      }
    }
    const ka = p3 * (p3 - 1) / (dt * dt);
    for (let i = 0; i + 2 < n; i++) {
      const ax = (x[(i + 2) * DIM] - 2 * x[(i + 1) * DIM] + x[i * DIM]) * ka;
      const ay = (x[(i + 2) * DIM + 1] - 2 * x[(i + 1) * DIM + 1] + x[i * DIM + 1]) * ka;
      const e = ax * ax + ay * ay - cfg.amax * cfg.amax;
      if (e > 0) {
        f += wF * e * e * e;
        const gx = wF * 6 * e * e * ax * ka, gy = wF * 6 * e * e * ay * ka;
        g[(i + 2) * DIM] += gx; g[(i + 1) * DIM] -= 2 * gx; g[i * DIM] += gx;
        g[(i + 2) * DIM + 1] += gy; g[(i + 1) * DIM + 1] -= 2 * gy; g[i * DIM + 1] += gy;
      }
    }
    for (let i = 0; i < 3; i++) {
      g[i * DIM] = 0; g[i * DIM + 1] = 0;
      g[(n - 1 - i) * DIM] = 0; g[(n - 1 - i) * DIM + 1] = 0;
    }
    return f;
  };
  for (let outer = 0; outer < 12; outer++) {
    buildAnchors();
    lbfgs(n * DIM, Q, obj, { mem: 16, maxIter: 400, gEps: 1e-8, delta: 1e-5, past: 3 });
  }
  // 균일 3차 B-스플라인 → 조각별 다항식 (MinJerkOpt 컨테이너를 그대로 빌려 쓴다)
  const toTraj = () => {
    const NP = n - 3;
    const jo = new MinJerkOpt();
    jo.reset([[0, 0], [0, 0], [0, 0]], [[0, 0], [0, 0], [0, 0]], NP);
    for (let i = 3; i < n; i++) {
      const o = 6 * (i - 3);
      for (let d = 0; d < DIM; d++) {
        const P0 = Q[(i - 3) * DIM + d], P1 = Q[(i - 2) * DIM + d],
              P2 = Q[(i - 1) * DIM + d], P3 = Q[i * DIM + d];
        const c0 = (P0 + 4 * P1 + P2) / 6, c1 = (-3 * P0 + 3 * P2) / 6,
              c2 = (3 * P0 - 6 * P1 + 3 * P2) / 6, c3 = (-P0 + 3 * P1 - 3 * P2 + P3) / 6;
        jo.b[(o + 0) * DIM + d] = c0;
        jo.b[(o + 1) * DIM + d] = c1 / dt;
        jo.b[(o + 2) * DIM + d] = c2 / (dt * dt);
        jo.b[(o + 3) * DIM + d] = c3 / (dt * dt * dt);
        jo.b[(o + 4) * DIM + d] = 0; jo.b[(o + 5) * DIM + d] = 0;
      }
      jo.T1[i - 3] = dt; jo.T2[i - 3] = dt * dt; jo.T3[i - 3] = dt * dt * dt;
      jo.T4[i - 3] = jo.T2[i - 3] * jo.T2[i - 3]; jo.T5[i - 3] = jo.T4[i - 3] * dt;
    }
    return jo;
  };
  // 사후 시간 신축 (ego-planner). 정확한 다항식 최댓값 기준
  {
    const t0 = toTraj();
    const [v0, a0] = maxRates(t0);
    const sc = Math.max(v0 / cfg.vmax, Math.sqrt(Math.max(0, a0 / cfg.amax)));
    if (sc > 0 && isFinite(sc)) dt *= sc;
    for (let it = 0; it < 30; it++) {
      const t_ = toTraj();
      const [v1, a1] = maxRates(t_);
      if (v1 <= cfg.vmax * 1.001 && a1 <= cfg.amax * 1.001) break;
      dt *= 1.01;
    }
  }
  return { traj: toTraj(), Q, n, dt, anchors };
}

/* --------------------------------------------------------- 전체 파이프라인 */
function planAll(S, cfg) {
  cfg = Object.assign({}, DEF, cfg || {});
  const out = { ok: false };
  const t0 = (typeof performance !== 'undefined' ? performance.now() : Date.now());
  const path = astar(S, 0.20, 0.55);
  if (!path) { out.err = 'A* 실패 — 시작점이나 목적지가 장애물 안에 있거나 길이 막혔습니다'; return out; }
  out.path = path;
  const sfc = buildSFC(S, path, 1.6);
  out.hPolys = sfc.hPolys;
  const box = [S.xmin - 2, S.xmax + 2, S.ymin - 2, S.ymax + 2];
  const tA0 = (typeof performance !== 'undefined' ? performance.now() : Date.now());
  const A = optimizeMINCO(sfc.hPolys, S.start, S.goal, cfg, box);
  const tA1 = (typeof performance !== 'undefined' ? performance.now() : Date.now());
  if (!A) { out.err = '회랑 꼭짓점 열거 실패'; return out; }
  out.A = { traj: A.traj, m: measure(A.traj, S, A.hPolys, cfg.clearance), ms: tA1 - tA0, iter: A.iter };
  out.hPolysUsed = A.hPolys;

  const jb = armB(A.traj, S.start, S.goal);
  if (jb) out.B = { traj: jb, m: measure(jb, S, A.hPolys, cfg.clearance), ms: 0 };
  const jc = armC(path, S.start, S.goal, cfg);
  if (jc) out.C = { traj: jc, m: measure(jc, S, null, cfg.clearance), ms: 0 };
  const tD0 = (typeof performance !== 'undefined' ? performance.now() : Date.now());
  const D = armD(S, path, cfg);
  const tD1 = (typeof performance !== 'undefined' ? performance.now() : Date.now());
  if (D) out.D = { traj: D.traj, m: measure(D.traj, S, null, cfg.clearance), ms: tD1 - tD0, Q: D.Q, n: D.n };
  out.msTotal = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - t0;
  out.ok = true;
  return out;
}

if (typeof module !== 'undefined') module.exports = {
  Banded, MinJerkOpt, lbfgs, distToObs, astar, corridor, buildSFC,
  enumerateVs, extractVs, optimizeMINCO, measure, expC2, logC2, gdT2t, DEF, DIM,
  scaleTime, maxRates, fitLimits, minJerkThrough, armB, armC, armD, planAll
};
