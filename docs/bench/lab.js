/* ============ 08 대화형 실험실 — 드래그하면 네 방법이 그 자리에서 다시 푼다 ============ */
(function () {
  const cv = document.getElementById('labCv');
  if (!cv || !window.PLAN) return;
  const P = window.PLAN;
  const ctx = cv.getContext('2d');
  const statusEl = document.getElementById('labStatus');
  const bodyEl = document.getElementById('labBody');

  const W = 22, H = 14;                       // 월드 크기 [m]
  const PRESETS = [
    [[5, 7, 1.1], [9, 4.2, 1.3], [9, 10.0, 1.3], [13, 7, 1.5], [17, 4.5, 1.2], [17, 9.5, 1.2]],
    [[7, 3.0, 2.6], [7, 11.0, 2.6], [13, 3.2, 2.6], [13, 10.8, 2.6], [18, 7, 1.0]],
    [[4.5, 5, .95], [4.5, 9, .95], [7.5, 7, .95], [7.5, 3, .95], [7.5, 11, .95],
     [10.5, 5, .95], [10.5, 9, .95], [13.5, 7, .95], [13.5, 3.2, .95], [13.5, 10.8, .95],
     [16.5, 5, .95], [16.5, 9, .95]]
  ];
  const METHODS = [
    { k: 'A', name: 'A. MINCO + SFC', tok: '--c-gco', dash: [] },
    { k: 'B', name: 'B. 시간 균일', tok: '--c-ego', dash: [7, 4] },
    { k: 'C', name: 'C. 웨이포인트 고정', tok: '--c-ddr', dash: [] },
    { k: 'D', name: 'D. B-스플라인 + 앵커', tok: '--c-mit', dash: [] }
  ];

  const S = { obs: [], start: [1.5, 7], goal: [20.5, 7], xmin: 0, xmax: W, ymin: 0, ymax: H };
  const show = { A: true, B: false, C: false, D: true };
  const layer = { sfc: true, astar: true };
  let result = null, drag = null, pending = 0;

  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim() || '#888';

  function loadPreset(i) {
    S.obs = PRESETS[i].map(o => ({ x: o[0], y: o[1], r: o[2] }));
    S.start = [1.5, 7]; S.goal = [20.5, 7];
  }

  /* ---------------------------------------------------------------- 좌표 변환 */
  let sc = 30, ox = 0, oy = 0;
  function resize() {
    const w = cv.parentElement.clientWidth;
    const h = Math.round(w * H / W);
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    cv.style.height = h + 'px';
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    sc = w / W; ox = 0; oy = 0;
    draw();
  }
  const X = x => ox + x * sc;
  const Y = y => oy + (H - y) * sc;
  const wx = px => (px - ox) / sc;
  const wy = py => H - (py - oy) / sc;

  /* -------------------------------------------------------------------- 그리기 */
  function draw() {
    const w = cv.width, h = cv.height, dpr = Math.min(2, window.devicePixelRatio || 1);
    ctx.clearRect(0, 0, w / dpr, h / dpr);
    const ink = css('--ink'), ink3 = css('--ink-3'), rule = css('--rule'),
          sunk = css('--sunk'), surface = css('--surface');

    ctx.fillStyle = surface; ctx.fillRect(0, 0, W * sc, H * sc);
    // 격자
    ctx.strokeStyle = rule; ctx.globalAlpha = .32; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = 2; x < W; x += 2) { ctx.moveTo(X(x), Y(0)); ctx.lineTo(X(x), Y(H)); }
    for (let y = 2; y < H; y += 2) { ctx.moveTo(X(0), Y(y)); ctx.lineTo(X(W), Y(y)); }
    ctx.stroke(); ctx.globalAlpha = 1;

    // 안전 회랑
    if (layer.sfc && result && result.hPolys) {
      const box = [S.xmin - 2, S.xmax + 2, S.ymin - 2, S.ymax + 2];
      ctx.lineWidth = 1;
      result.hPolys.forEach((Hp, i) => {
        const V = P.enumerateVs(Hp, box);
        if (!V) return;
        ctx.beginPath();
        V.forEach((p, j) => j ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
        ctx.closePath();
        ctx.fillStyle = css('--c-elt'); ctx.globalAlpha = .035; ctx.fill();
        ctx.strokeStyle = css('--c-elt'); ctx.globalAlpha = .20; ctx.stroke();
      });
      ctx.globalAlpha = 1;
    }

    // 장애물
    for (const o of S.obs) {
      ctx.beginPath(); ctx.arc(X(o.x), Y(o.y), o.r * sc, 0, 7);
      ctx.fillStyle = ink3; ctx.globalAlpha = .20; ctx.fill();
      ctx.globalAlpha = .55; ctx.lineWidth = 1.4; ctx.strokeStyle = ink3; ctx.stroke();
      ctx.globalAlpha = 1;
    }

    // A* 경로
    if (layer.astar && result && result.path) {
      ctx.beginPath();
      result.path.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
      ctx.strokeStyle = ink3; ctx.lineWidth = 1.2; ctx.setLineDash([2, 3]); ctx.globalAlpha = .8;
      ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
    }

    // 궤적 — A 를 맨 위에 그린다 (겹칠 때 보이도록)
    if (result && result.ok) {
      for (const M of METHODS.slice().reverse()) {
        if (!show[M.k] || !result[M.k]) continue;
        const jo = result[M.k].traj, T = jo.totalDuration(), p = [0, 0];
        ctx.beginPath();
        for (let i = 0; i <= 400; i++) {
          jo.eval(T * i / 400, 0, p);
          i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]));
        }
        ctx.strokeStyle = css(M.tok);
        ctx.lineWidth = M.k === 'A' ? 3.0 : 2.0;
        ctx.setLineDash(M.dash.map(v => v * sc / 30));
        ctx.lineJoin = 'round'; ctx.lineCap = 'round';
        ctx.stroke(); ctx.setLineDash([]);
      }
    }

    // 시작 · 목적지
    const g = css('--yes'), r = css('--no');
    ctx.beginPath(); ctx.arc(X(S.start[0]), Y(S.start[1]), 7, 0, 7);
    ctx.fillStyle = g; ctx.fill(); ctx.strokeStyle = surface; ctx.lineWidth = 2; ctx.stroke();
    star(X(S.goal[0]), Y(S.goal[1]), 10, 4.6);
    ctx.fillStyle = r; ctx.fill(); ctx.strokeStyle = surface; ctx.lineWidth = 2; ctx.stroke();

    // 실패 메시지
    if (result && !result.ok) {
      ctx.fillStyle = css('--no'); ctx.globalAlpha = .10;
      ctx.fillRect(0, 0, W * sc, H * sc); ctx.globalAlpha = 1;
    }
  }
  function star(cx, cy, R, r) {
    ctx.beginPath();
    for (let i = 0; i < 10; i++) {
      const a = -Math.PI / 2 + i * Math.PI / 5, rr = i % 2 ? r : R;
      i ? ctx.lineTo(cx + rr * Math.cos(a), cy + rr * Math.sin(a))
        : ctx.moveTo(cx + rr * Math.cos(a), cy + rr * Math.sin(a));
    }
    ctx.closePath();
  }

  /* -------------------------------------------------------------------- 계산 */
  function replan(quick) {
    // 예약된 미리보기가 나중에 전체 결과를 덮어쓰지 않도록 먼저 취소한다
    if (pending) { cancelAnimationFrame(pending); pending = 0; }
    const t0 = performance.now();
    let r;
    if (quick) {
      // 드래그 중에는 A 만 적은 반복으로 — 미리보기
      r = { ok: false };
      const path = P.astar(S, 0.20, 0.55);
      if (path) {
        r.path = path;
        const sfc = P.buildSFC(S, path, 1.6);
        r.hPolys = sfc.hPolys;
        const box = [S.xmin - 2, S.xmax + 2, S.ymin - 2, S.ymax + 2];
        const A = P.optimizeMINCO(sfc.hPolys, S.start, S.goal, { maxIter: 140 }, box);
        if (A) { r.A = { traj: A.traj, m: P.measure(A.traj, S, A.hPolys, 0.2), ms: 0 }; r.ok = true; }
      }
      if (!r.ok) r.err = 'A* 실패 — 시작점·목적지가 막혔거나 장애물 안에 있습니다';
      r.quick = true;
    } else {
      r = P.planAll(S, {});
    }
    r.wall = performance.now() - t0;
    result = r;
    draw();
    renderTable();
  }
  function renderTable() {
    const r = result;
    if (!r) return;
    if (!r.ok) {
      statusEl.className = 'lab-status err';
      statusEl.textContent = r.err || '계산 실패';
      bodyEl.innerHTML = '<tr><td class="mname" colspan="10">—</td></tr>';
      return;
    }
    statusEl.className = 'lab-status';
    const bits = [`장애물 <b>${S.obs.length}</b>개`,
                  `A* <b>${r.path.length}</b>점`,
                  `회랑 <b>${r.hPolys.length}</b>개`];
    if (r.quick) bits.push('<b>미리보기</b> (A만, 반복 140회)');
    else bits.push(`전체 <b>${r.wall.toFixed(0)} ms</b>`);
    statusEl.innerHTML = bits.join(' · ');

    const rows = [];
    for (const M of METHODS) {
      const d = r[M.k];
      const on = show[M.k];
      if (!d) {
        rows.push(`<tr class="off"><td class="mname"><i style="background:var(${M.tok})"></i>${M.name}</td>` +
                  '<td class="num" colspan="9">—</td></tr>');
        continue;
      }
      const m = d.m;
      const clrBad = m.minClr < 0 ? ' bad' : '';
      const vBad = m.vmax > 3.05 ? ' bad' : '';
      const cvBad = m.corrViol > 1e-4 ? ' bad' : '';
      rows.push(
        `<tr class="${on ? '' : 'off'}"><td class="mname"><i style="background:var(${M.tok})"></i>${M.name}</td>` +
        `<td class="num">${m.len.toFixed(2)}</td>` +
        `<td class="num">${m.jerk2.toFixed(0)}</td>` +
        `<td class="num">${m.snap2.toFixed(0)}</td>` +
        `<td class="num${clrBad}">${m.minClr.toFixed(3)}</td>` +
        `<td class="num${cvBad}">${(M.k === 'A' || M.k === 'B') ? m.corrViol.toFixed(4) : '—'}</td>` +
        `<td class="num${vBad}">${m.vmax.toFixed(2)}</td>` +
        `<td class="num">${m.amax.toFixed(2)}</td>` +
        `<td class="num">${m.T.toFixed(2)}</td>` +
        `<td class="num">${(M.k === 'A' || M.k === 'D') ? (d.ms > 0 ? d.ms.toFixed(1) : '&lt;0.1') : '<span title="반복 최적화가 아니라 밴디드 선형계를 한 번 푸는 것이라 비교 대상이 아닙니다">—</span>'}</td></tr>`);
    }
    bodyEl.innerHTML = rows.join('');
  }
  function schedule(quick) {
    if (pending) return;
    pending = requestAnimationFrame(() => { pending = 0; replan(quick); });
  }

  /* -------------------------------------------------------------------- 입력 */
  function pick(px, py) {
    const mx = wx(px), my = wy(py), tolPx = 9 / sc;
    if (Math.hypot(mx - S.start[0], my - S.start[1]) < 12 / sc) return { t: 'start' };
    if (Math.hypot(mx - S.goal[0], my - S.goal[1]) < 12 / sc) return { t: 'goal' };
    for (let i = S.obs.length - 1; i >= 0; i--) {
      const o = S.obs[i], d = Math.hypot(mx - o.x, my - o.y);
      if (Math.abs(d - o.r) < tolPx) return { t: 'rim', i };
      if (d < o.r) return { t: 'obs', i, dx: mx - o.x, dy: my - o.y };
    }
    return null;
  }
  cv.addEventListener('pointermove', e => {
    const R = cv.getBoundingClientRect();
    const px = e.clientX - R.left, py = e.clientY - R.top;
    if (!drag) {
      const h = pick(px, py);
      cv.style.cursor = !h ? 'default' : (h.t === 'rim' ? 'nwse-resize' : 'grab');
      return;
    }
    const mx = wx(px), my = wy(py);
    const cl = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
    if (drag.t === 'start') S.start = [cl(mx, .3, W - .3), cl(my, .3, H - .3)];
    else if (drag.t === 'goal') S.goal = [cl(mx, .3, W - .3), cl(my, .3, H - .3)];
    else if (drag.t === 'obs') {
      const o = S.obs[drag.i];
      o.x = cl(mx - drag.dx, -1, W + 1); o.y = cl(my - drag.dy, -1, H + 1);
    } else if (drag.t === 'rim') {
      const o = S.obs[drag.i];
      o.r = cl(Math.hypot(mx - o.x, my - o.y), 0.25, 5);
    }
    draw();
    schedule(true);
  });
  cv.addEventListener('pointerdown', e => {
    const R = cv.getBoundingClientRect();
    const h = pick(e.clientX - R.left, e.clientY - R.top);
    if (!h) return;
    drag = h; cv.setPointerCapture(e.pointerId); cv.style.cursor = 'grabbing';
    e.preventDefault();
  });
  const endDrag = () => { if (!drag) return; drag = null; cv.style.cursor = 'default'; replan(false); };
  cv.addEventListener('pointerup', endDrag);
  cv.addEventListener('pointercancel', endDrag);
  cv.addEventListener('dblclick', e => {
    const R = cv.getBoundingClientRect();
    const px = e.clientX - R.left, py = e.clientY - R.top;
    const h = pick(px, py);
    if (h && (h.t === 'obs' || h.t === 'rim')) S.obs.splice(h.i, 1);
    else if (!h) S.obs.push({ x: wx(px), y: wy(py), r: 1.0 });
    else return;
    replan(false);
  });

  /* -------------------------------------------------------------------- 버튼 */
  document.querySelectorAll('.lab-btn[data-scene]').forEach(b => b.addEventListener('click', () => {
    document.querySelectorAll('.lab-btn[data-scene]').forEach(o => o.setAttribute('aria-pressed', String(o === b)));
    loadPreset(+b.dataset.scene); replan(false);
  }));
  document.querySelectorAll('.lab-btn[data-m]').forEach(b => b.addEventListener('click', () => {
    const k = b.dataset.m; show[k] = !show[k];
    b.setAttribute('aria-pressed', String(show[k]));
    draw(); renderTable();
  }));
  document.querySelectorAll('.lab-btn[data-layer]').forEach(b => b.addEventListener('click', () => {
    const k = b.dataset.layer; layer[k] = !layer[k];
    b.setAttribute('aria-pressed', String(layer[k]));
    draw();
  }));

  /* ------------------------------------------------------------------ 시작 */
  // 컨테이너 폭 변화(목차 접기 등)에도 배율을 다시 잡는다 —
  // 배율이 낡으면 그림이 늘어나고 클릭 좌표가 어긋난다
  if (window.ResizeObserver) {
    let lastW = 0, roPending = 0;
    new ResizeObserver(() => {
      // 콜백 안에서 바로 레이아웃을 바꾸면 "undelivered notifications" 경고가 난다.
      // 다음 프레임으로 미뤄 관측 루프 밖에서 처리한다
      if (roPending) return;
      roPending = requestAnimationFrame(() => {
        roPending = 0;
        const w = cv.parentElement.clientWidth;
        if (w > 0 && Math.abs(w - lastW) > 0.5) { lastW = w; resize(); }
      });
    }).observe(cv.parentElement);
  }
  addEventListener('resize', () => { resize(); });
  const mq = matchMedia('(prefers-color-scheme:dark)');
  (mq.addEventListener ? mq.addEventListener('change', draw) : mq.addListener(draw));
  loadPreset(0);
  resize();
  // 첫 계산은 화면에 들어왔을 때 (긴 문서라 위에서 바로 돌릴 필요가 없다)
  const io = new IntersectionObserver(es => {
    if (es.some(x => x.isIntersecting)) { io.disconnect(); replan(false); }
  }, { rootMargin: '200px' });
  io.observe(cv);
})();
