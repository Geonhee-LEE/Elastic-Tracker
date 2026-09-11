#!/usr/bin/env python3
"""bench 가 뱉은 scene.csv · traj.csv 로 bench.png 를 그린다.

  왼쪽  경로 — 장애물(원판), A* 경로, 네 방법의 궤적
  오른쪽 속도 프로파일 — A 와 B 는 경로가 같아 차이가 여기서만 보인다

사용:  python3 plot.py            (docs/bench 에서)
"""
import csv, collections, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

NAMES = {"A": "A. MINCO + SFC (원본 코드)", "B": "B. 같은 웨이포인트 · 시간 균일",
         "C": "C. 웨이포인트 고정 · 회랑 없음", "D": "D. B-스플라인 + {p,v} 앵커"}
COLOR = {"A": "#2B5CE6", "B": "#E07A2B", "C": "#3F7A4E", "D": "#A8431C"}
STYLE = {"A": dict(lw=2.4, ls="-", zorder=6), "B": dict(lw=1.8, ls=(0, (5, 3)), zorder=5),
         "C": dict(lw=1.6, ls="-", zorder=4), "D": dict(lw=1.6, ls="-", zorder=4)}
TITLE = ["장면 0 — 성긴 숲", "장면 1 — 좁은 통로", "장면 2 — 조밀"]

# 한글이 들어가므로 CJK 글꼴을 찾는다.  KR 판이 없으면 JP/HK 판도 한글 자모를 담고 있다
_have = {f.name for f in matplotlib.font_manager.fontManager.ttflist}
for _f in ("Noto Sans CJK KR", "NanumGothic", "Noto Sans KR",
           "Noto Sans CJK JP", "Noto Sans CJK HK", "Noto Sans CJK SC"):
    if _f in _have:
        plt.rcParams["font.family"] = _f
        break
else:
    print("경고: CJK 글꼴을 찾지 못했습니다 — 한글이 깨집니다")
plt.rcParams["axes.unicode_minus"] = False

scene = collections.defaultdict(lambda: {"obs": [], "path": [], "start": None, "goal": None})
for r in csv.reader(open("scene.csv")):
    if not r or r[0] == "scene":
        continue
    s, kind, x, y, z = int(r[0]), r[1], float(r[2]), float(r[3]), float(r[4])
    if kind == "obs":
        scene[s]["obs"].append((x, y, z))
    elif kind == "path":
        scene[s]["path"].append((x, y))
    else:
        scene[s][kind] = (x, y)

traj = collections.defaultdict(list)
for r in csv.reader(open("traj.csv")):
    if not r or r[0] == "scene":
        continue
    traj[(int(r[0]), r[1])].append((float(r[2]), float(r[3]), float(r[4])))

VMAX = 3.0
fig, axes = plt.subplots(3, 2, figsize=(15.6, 12.2),
                         gridspec_kw=dict(width_ratios=[1.62, 1.0], hspace=0.30, wspace=0.16))

for s in range(3):
    axp, axv = axes[s]
    S = scene[s]
    for (x, y, r) in S["obs"]:
        axp.add_patch(Circle((x, y), r, fc="#D8DCE4", ec="#9AA2B4", lw=1.0, zorder=1))
    if S["path"]:
        px, py = zip(*S["path"])
        axp.plot(px, py, color="#9AA2B4", lw=1.0, ls=":", zorder=2, label="A* 프런트엔드")
    for k in "ABCD":
        pts = traj.get((s, k))
        if not pts:
            continue
        t, x, y = zip(*pts)
        axp.plot(x, y, color=COLOR[k], label=NAMES[k], **STYLE[k])
        v = [math.hypot((x[i + 1] - x[i - 1]) / (t[i + 1] - t[i - 1]),
                        (y[i + 1] - y[i - 1]) / (t[i + 1] - t[i - 1]))
             for i in range(1, len(t) - 1)]
        axv.plot(t[1:-1], v, color=COLOR[k], **STYLE[k])
    for p, m, c in ((S["start"], "o", "#1B7F3B"), (S["goal"], "*", "#B3261E")):
        if p:
            axp.plot(*p, marker=m, ms=13 if m == "*" else 9, color=c, mec="white", mew=1.4, zorder=8)

    axp.set_xlim(0, 22); axp.set_ylim(0, 14); axp.set_aspect("equal")
    axp.set_title(TITLE[s], fontsize=13, loc="left", pad=8)
    axp.grid(alpha=.18, lw=.6); axp.set_xlabel("x [m]"); axp.set_ylabel("y [m]")

    axv.axhline(VMAX, color="#B3261E", lw=1.0, ls="--", alpha=.7)
    axv.text(0.4, VMAX + .08, "v_max = 3.0", color="#B3261E", fontsize=9)
    axv.set_title("속도 프로파일", fontsize=11, loc="left", pad=8)
    axv.grid(alpha=.18, lw=.6); axv.set_xlabel("t [s]"); axv.set_ylabel("|v| [m/s]")
    axv.set_ylim(0, max(4.4, VMAX * 1.45))

h, l = axes[0][0].get_legend_handles_labels()
fig.legend(h, l, loc="lower center", ncol=5, fontsize=10, frameon=False,
           bbox_to_anchor=(.5, -.012))
fig.savefig("bench.png", dpi=110, bbox_inches="tight", facecolor="white")
print("bench.png 생성")
