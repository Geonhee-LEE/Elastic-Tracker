# Elastic-Tracker 문서

ZJU FAST Lab의 **Elastic Tracker**(ICRA 2022) ROS1 워크스페이스를 코드 레벨에서 분석한 한국어 기술 문서입니다.

> 논문: *Elastic Tracker: A Spatio-temporal Trajectory Planner for Flexible Aerial Tracking*,
> Jialin Ji, Neng Pan, Chao Xu, Fei Gao — IEEE ICRA 2022. ([arXiv:2109.07111](https://arxiv.org/abs/2109.07111))

## 목차

| 문서 | 내용 |
|---|---|
| [01. 개요](01-overview.md) | 무엇을 하는 프로젝트인가, 핵심 아이디어 3가지 |
| [02. 시스템 아키텍처](02-architecture.md) | 노드 구성, 토픽 그래프, 데이터 플로우, 리플랜 상태 머신 |
| [03. 패키지 레퍼런스](03-packages.md) | 12개 ROS 패키지 각각의 역할과 주요 파일 |
| [04. 알고리즘 심층 분석](04-algorithms.md) | 예측 → 가시 경로 탐색 → 가시 영역 → SFC → MINCO 최적화 |
| [05. ROS 인터페이스](05-ros-interface.md) | 커스텀 메시지, 토픽, 파라미터 전수 목록 |
| [06. 빌드 & 실행](06-build-run.md) | 의존성, CUDA 설정, 4가지 시나리오 실행법, 트러블슈팅 |
| [07. 코드 리딩 노트](07-code-notes.md) | 호출 체인, 메모리 특성, 코드에서 발견한 이슈 |
| [08. ZJU FAST Lab 계보](08-zju-fast-lab-family.md) | **MINCO · 미분평탄성 배경**, 계보, **솔버 스택 7종**, **원본 코드를 링크한 실측 벤치마크**와 **장애물을 끌어 옮기는 대화형 도구**([bench/](bench/README.md)), 후속 연구 · 경쟁 학파, **최근 연구 지형**(3D·동적 장애물, 약 50편 · 연구실 14곳)과 **다음 연구 방향**(공백 · 동적 가림 정식화 · 로드맵) · 참고문헌 82건 |

## HTML 버전

GitHub에서 HTML 파일을 누르면 소스 코드가 보입니다. 웹페이지로 보려면 **웹에서 보기** 링크를 누르세요
(GitHub Pages가 이 `docs/` 폴더를 그대로 게시합니다).

- `docs/index.html` — [**웹에서 보기**](https://geonhee-lee.github.io/Elastic-Tracker/) ·
  01–07을 한 페이지로 묶은 문서. **수식(MathML)**, **직접 조작하는 도해 2종**
  (시간 파라미터화 / 가시성 페널티), **페널티 함수 차트**, 안전 회랑 생성 3단계 도해, 심각도순 이슈 목록
- `docs/lineage.html` — [**웹에서 보기**](https://geonhee-lee.github.io/Elastic-Tracker/lineage.html) ·
  08의 HTML 버전. 계보도, 결정변수 수식 대비, "표현 × 충돌 처리" 2×2 매트릭스,
  **다섯 저장소 파일 탐색기**(탭 전환 + 검증 깊이 배지), 그리고 13–14절의 **2019–2026 연구 연표**,
  **환경 × 가시성 3×3 매트릭스**, **동적 가림 비용 도해**와 제안 로드맵

수식은 브라우저 네이티브 MathML, 도해와 차트는 인라인 SVG입니다 — 외부 라이브러리를 쓰지 않습니다.

```
xdg-open docs/index.html
xdg-open docs/lineage.html
```

## 30초 요약

카메라를 단 쿼드로터가 움직이는 목표물(드론/차량)을 **장애물 사이에서 놓치지 않고** 따라다니게 하는
스페이시오템포럴(spatio-temporal) 궤적 플래너입니다. 세 가지가 동시에 보장됩니다.

1. **안전(Safety)** — Safe Flight Corridor(볼록 다면체 열) 안에서만 궤적을 만든다.
2. **가시성(Visibility)** — 목표물과 드론을 잇는 시선이 장애물에 가리지 않도록 부채꼴 "가시 영역"
   제약을 비용 함수에 넣는다.
3. **탄성(Elasticity)** — 궤적의 공간(제어점)뿐 아니라 **시간 배분까지 동시에** 최적화해서,
   목표물 속도 변화에 궤적 전체가 늘어나고 줄어든다.

20 Hz로 전 파이프라인(예측 → 경로 → 회랑 → 최적화 → 충돌 검사)을 다시 돌리는 리플래닝 구조입니다.

## 문서 작성 기준

- 이 문서는 **저장소에 실제로 존재하는 코드**만을 근거로 작성되었습니다.
  파일 경로와 줄 번호를 함께 표기했으므로 원문과 대조하며 읽을 수 있습니다.
- `src/planning/DecompROS/`는 외부 라이브러리([sikang/DecompROS](https://github.com/sikang/DecompROS))
  를 그대로 벤더링한 것이라 인터페이스 수준에서만 다룹니다.
