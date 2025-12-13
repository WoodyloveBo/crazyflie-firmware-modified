# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

This project contains the source code for the firmware used in the Crazyflie range of platforms, including the Crazyflie 2.x and the Roadrunner.

# `app_fs`: Spring Tension Estimation (World Frame)

이 모듈은 Crazyflie 펌웨어에서 월드 좌표계(World Frame) 기반의 스프링 장력 벡터($\vec{F}_S^W$)를 실시간으로 추정하는 애플리케이션입니다.

---

## 1. Methodology (방법론)

스프링 장력 추정은 뉴턴의 제2법칙($\Sigma \vec{F} = m\vec{a}$)에 기반하며, 비행체에 작용하는 모든 힘($\Sigma \vec{F}$)에서 중력($m\vec{g}$)과 추력($\vec{F}_T$)을 제외한 잔여 힘을 스프링 장력으로 간주합니다.

### Dynamic Acceleration (동역학적 가속도)

$$\vec{F}_S^W = \vec{F}_T^W - m\vec{a}_{dyn}^W$$

여기서 $\vec{a}_{dyn}^W$는 월드 좌표계에서 중력 성분이 제거된 순수한 동역학적 가속도입니다.

$$\vec{a}_{dyn}^W = \vec{R}(q) \cdot \vec{a}_{body} - \vec{g}^W$$

* $\vec{R}(q)$: MoCap 쿼터니언에서 파생된 회전 행렬 (Body $\to$ World).
* $\vec{a}_{body}$: IMU에서 측정된 Raw 가속도 (Body Frame, 단위 $g$ 사용).
* $\vec{g}^W$: 월드 좌표계 중력 벡터 (Z축 방향). 

### 2. Implementation Details (구현 세부 사항)

| 기능 | 설명 | 사용된 데이터 소스 |
| :--- | :--- | :--- |
| **자세 (Rotation)** | 외부 위치 서비스(MoCap)의 쿼터니언(`locSrv.q*`)을 사용하여, Body Frame의 가속도 및 추력 벡터를 World Frame으로 변환합니다. | `locSrv.q*` |
| **추력 (Thrust)** | PWM 커맨드(`stabilizer.thrust`)를 2차 다항식 캘리브레이션 모델을 통해 힘(gf)으로 변환합니다. | `stabilizer.thrust` |
| **필터링 (Filtering)** | 추정된 원시 장력($\vec{F}_{S, raw}^W$)의 노이즈를 줄이기 위해 **Moving Average Filter**가 적용됩니다. | `fs.maN` 파라미터 |

### 3. Log Output & Parameters (로그 및 파라미터)

#### Parameters (사용자 설정 파라미터)

| Group.Name | Type | Unit | Default | Description |
| :--- | :--- | :--- | :--- | :--- |
| `fs.mass` | Float | [kg] | `0.04f` | 드론의 질량 |
| `fs.useMA` | Uint8 | N/A | `1` | 이동 평균 필터 사용 여부 (0: Off, 1: On) |
| `fs.maN` | Int32 | [samples] | `15` | 이동 평균 윈도우 길이 (1..200). 100Hz 기준 15 samples = 0.15초 |

#### Log Variables (추출 로그)

`log.fs` 그룹을 통해 다음 값들을 확인할 수 있습니다.

| Name | Type | Unit | Description |
| :--- | :--- | :--- | :--- |
| `Fsx_raw`, `Fsy_raw`, `Fsz_raw` | Float | [gf] | 필터링되지 않은 원시 스프링 장력 |
| **`Fsx`, `Fsy`, `Fsz`** | Float | [gf] | **이동 평균 필터가 적용된 최종 스프링 장력** |
| `Ft` | Float | [gf] | PWM 기반 추력 크기 스칼라 |
| `FTwx`, `FTwy`, `FTwz` | Float | [gf] | 월드 좌표계 추력 벡터($\vec{F}_T^W$) 성분 (Debug) |
| `maw_dx`, `maw_dy`, `maw_dz` | Float | [gf] | 질량 곱하기 동역학적 가속도($m\vec{a}_{dyn}^W$) 성분 (Debug) |
