# MATLAB 실행 묶음 (2026-11-02)

이번에 Octave로 돌린 Step 2 보완과 Step 3 보강을 MATLAB에서 다시 돌려 숫자를 확정하기 위한 파일이다.
Octave 결과는 `참고_Octave결과/`에 있다. 난류와 센서 잡음 난수가 MATLAB과 달라 **결정론적 조건은 같아야 하고, 나머지는 경향만 같으면 된다.**

## 0. 준비

- 필요: MATLAB R2024b, Robust Control Toolbox (LMI Lab: `ts_design`, `run_tests`의 `t_lmi_design`). Step 3는 기본 MATLAB만 있으면 된다.
- **저장소 브랜치를 새로 만든 뒤** 이 묶음의 `MATLAB/` 폴더를 저장소의 `MATLAB/` 위에 덮어쓴다. 덮어쓰는 파일:
  - `step2/`: `guidance_v2.m`, `ts_design.m`, `gains.m`, `run_tests.m`, `step2_paired_ci.m` (수정), `step2_sf_simtuned.m`, `step2_ts_model_check.m` (새 파일)
  - `step3/`: 조원 파일 4개(`kf_track`, `test_kf_track`, `step3_paths`, `step3_kf_eval`)는 그대로이고, 새 파일 9개가 추가된다
- `참고_스크립트/` 폴더는 저장소 최상위(= `MATLAB/` 옆)에 둔다. 상대 경로 `../../MATLAB/step2`를 쓴다.
- 긴 실행 중에는 노트북 절전을 끈다.

## 1. Step 2 보완 (순서대로)

| # | 실행 (`MATLAB/step2`에서) | 하는 일 | 확인할 값 (Octave 기준) |
|---|---|---|---|
| 1 | `run_tests` | 회귀 테스트 7개 | **7/7 통과**. 7번 `t_model_consistency` 최대 오차 0.007%. (Octave에서는 LMI Lab이 없어 `t_lmi_design`만 실행 불가, 나머지 6개 통과) |
| 2 | `step2_ts_model_check` | 모델 검증 표: 모델과 실제 기체의 횡가속 오차, z1 범위 이탈, 포화 비율 → `ts_model_check.csv` | 직선/측풍 15 오차 6.45% → 0.00%, 회피/측풍 15 + 난류 z1 범위 밖 20% → 0% |
| 3 | `p = ...; ts = ts_design(p); norm(ts.K{1})` (아래 참고) | 새 모델의 LMI 고정 게인과 크기 | K = [−0.767 −1.151 −0.530 0.689], 크기 **1.634** (Python cvxpy로 계산) |
| 4 | `gains.m`의 `G.cap_hi`를 3번 값으로 고침 | 게인 크기 상한을 새 LMI 게인과 같게 | |
| 5 | `step2_pid_best_alpha` → `G.pidt` | 극 기준 PID (새 섹터, 새 상한) | [9.453 11.991 4.866], 감쇠율 1.468 |
| 6 | `step2_pid_simtuned` → `G.simtuned` | 시뮬 튜닝 PID (새 상한) | Octave에서는 시작점 문제로 기존 게인 [6.626 8.807 9.059]보다 나쁜 국소해에 걸렸다. **결과의 목적함수가 기존 게인보다 나쁘면 기존 게인을 유지**한다(PID 경로는 모델 수정과 무관하고 기존 게인 크기 1.454는 상한 안) |
| 7 | `step2_sf_simtuned` → `G.sf_simtuned` | 시뮬 튜닝 상태 피드백 (2×2의 빈칸) | Octave는 시간 제한으로 약 100회에서 멈춤: [−0.750 −1.020 −0.978 0.330], 목적함수 −4.60. 끝까지 돌리면 더 낮아질 수 있다 |
| 8 | `step2_paired_ci` | 14장 표 재계산 (`branch`, `same_branch`, `two_by_two`, `robustness_same_branch` 묶음) | `참고_Octave결과/step2/summary_raw.txt`와 비교. 결정론적 행은 같은 게인이면 소수 넷째 자리까지 같아야 한다 |

3번 예시 (`step2_sf_simtuned.m` 앞부분과 같은 설정):
```matlab
p.m = 1.35; p.rho = 1.225; p.Cd_h = 0.47; p.Cd_v = 0.77; p.A = 0.073; p.g = 9.81;
p.thrust = 5; p.target_dir = [1;0;0]; p.cruise_alt = 30; p.tilt_max = 35; p.att_tau = 0.15;
p.ts_tau = 0.15; p.ts_Vmin = 5; p.ts_Vmax = 35; p.ts_Vnom = 15;
ts = ts_design(p);  K = ts.K{1},  norm(K)
```
이전 모델(롤 각, 비율 섹터)은 `p.ts_tilt = 'roll'; p.ts_sector = 'ratio';`로 재현된다(크기 1.462).

**게인 상한에 대한 참고**: 이전 LMI 게인(1.462)도 새 섹터에서 감쇠율 1이 인증된다(`참고_스크립트/step2_14장_재계산/cert.py`). 그래서 상한을 1.462로 두는 방법도 있지만, 그러려면 LMI 설계에 게인 크기 제약을 따로 넣어야 한다. 지금 코드 구조(상한 = `ts_design`이 낸 FIX의 크기)에서는 **1.634로 맞추는 것이 간단하고, Octave 재계산도 이 설정**이다.

## 2. Step 3 (순서대로, `MATLAB/step3`에서)

| # | 실행 | 하는 일 | 시간(Octave) | 확인할 값 (σ = 0.5 m, Octave) |
|---|---|---|---|---|
| 1 | `test_kf_track`, `test_kf_batch`, `test_imm_track` | 회귀 테스트 | 수 초 | 모두 PASS. `kf_batch`·`imm_track`은 `kf_track`과 차이 0 수준 |
| 2 | `step3_kf_eval` | 조원 최소 구현 (기존) | 약 45분 | `step3.md` 3절 |
| 3 | `step3_kf_extra` | q 선택 방식, J(q), NEES, 방향 변화 α, 연속 놓침 | 약 4분 | 단일 q가 `kf_qsel.csv`와 같음. 회피 CA − CV(모드별 q) +0.051 m, NEES 회피 CV 2.56, α 직진/무풍 원 측정 5.71° → CV 2.33° |
| 4 | `step3_kf_lead` | 리드 조준 예측 오차 | 약 1분 | τ = 0.3 s 조준각 CV 직진 5.8, 회피 10.8 mrad |
| 5 | `step3_switch_tracks` | 모드 전환 궤적 200개 → `switch_tracks.mat` | 약 8분(코어 2개) | 무풍 1개 계산 + 측풍 5 시드 100개 |
| 6 | `step3_imm_switch` | 단일 CV · IMM · 상한 비교 | 약 6분 | p_stay 0.99, 전체 위치 0.445 → 0.422 m, NEES 1.38 → 0.79, 감지 지연 중앙값 2.6 s |

결과는 모두 `step3/04_결과데이터/`에 저장된다(`kfx_*`, `kfl_*`, `kfs_*` CSV와 mat). `참고_Octave결과/step3/`의 같은 이름 파일과 비교하면 된다.
- `step3_switch_tracks`는 `step2/guidance_v2.m`의 `'switch'` 모드가 필요하다(이 묶음의 `guidance_v2.m`에 들어 있음). 기존 직진·회피 모드와 T-S 수정에는 영향이 없다.
- 2번(`step3_kf_eval`)은 3~6번에 필요하지 않다. 3번이 같은 q 선택을 다시 계산한다.

## 3. 참고 스크립트

`참고_스크립트/step2_14장_재계산/`: 이번 Octave 14장 재계산을 그대로 재현하는 스크립트. 공식 경로는 1절(`step2_paired_ci`)이고 이것은 대조용이다.
- `base_p.m`(기본 파라미터, `'old'`/`'new'` 모델), `rms_one.m`(조건 하나의 횡 오차 RMS), `obj.m`(튜닝 목적함수)
- `tune_pid.m`, `tune_sf.m`: 상한 1.6336으로 튜닝 → `sim_new.txt`, `sim4_new.txt`
- `eval_part.m`: `eval_part(1)`, `eval_part(2)`로 10개 조건 평가 → `eval_1.csv`, `eval_2.csv` (각 약 40분, Octave)
- `fix.json`, `pidt.json`: Python(cvxpy)으로 구한 LMI 게인과 극 기준 PID. `lmi.py`, `pidt.py`, `cert.py`(고정 게인 감쇠율 인증)는 Python 참고용
- 확인: `base_p('old')`로 시뮬 튜닝 PID 회피/무풍 RMS = 0.0233 (발표 값과 같음)

## 4. 결과를 받으면

- Step 2: `step2_paired_ci` 출력 CSV와 `ts_model_check.csv`를 보내 주면 14장 표, 결론, 초록 문구를 정리한다.
- Step 3: `04_결과데이터/`의 `kfx_*`, `kfl_*`, `kfs_*` CSV를 보내 주면 `step3.md` 6~8절과 발표 자료 숫자를 MATLAB 값으로 바꾼다.
