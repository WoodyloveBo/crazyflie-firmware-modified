# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

This project contains the source code for the firmware used in the Crazyflie range of platforms, including the Crazyflie 2.x and the Roadrunner.

# ⚙️ `app_fs`: Spring Tension Estimation (World Frame)

This module implements a real-time application within the Crazyflie firmware to estimate the **Spring Tension Vector** ($\vec{F}_S^W$) in the World Coordinate Frame.

---

## 1. Methodology

The spring tension is estimated based on Newton's Second Law ($\Sigma \vec{F} = m\vec{a}$). It calculates the remaining force acting on the drone after accounting for gravity ($m\vec{g}$) and thrust ($\vec{F}_T$).

### Dynamic Acceleration Approach

The equation is formulated using the dynamic acceleration ($\vec{a}_{dyn}^W$), which is the total acceleration with gravity removed:

$$\vec{F}_S^W = \vec{F}_T^W - m\vec{a}_{dyn}^W$$

Where the dynamic acceleration in the World Frame is derived from the Body Frame IMU reading and the MoCap quaternion:

$$\vec{a}_{dyn}^W = \vec{R}(q) \cdot \vec{a}_{body} - \vec{g}^W$$

* $\vec{R}(q)$: Rotation matrix derived from the MoCap quaternion (Body $\to$ World).
* $\vec{a}_{body}$: Raw acceleration measured by the IMU in the Body Frame (units of $G$).
* $\vec{g}^W$: Gravity vector in the World Frame (downward on the Z-axis). 

### 2. Implementation Details

| Feature | Description | Data Source |
| :--- | :--- | :--- |
| **Attitude (Rotation)** | The quaternion (`locSrv.q*`) from the external positioning service (MoCap) is used to transform the Body Frame acceleration and thrust vectors into the World Frame. | `locSrv.q*` |
| **Thrust (Scalar)** | The PWM command (`stabilizer.thrust`) is converted into force [gf] using a **calibrated second-order polynomial model**. | `stabilizer.thrust` |
| **Filtering (MA)** | To reduce noise in the raw tension estimate ($\vec{F}_{S, raw}^W$), an **Axis-by-Axis Moving Average Filter** is applied. | `fs.maN` parameter |

### 3. Log Output & Parameters

#### Parameters (User Configuration)

| Group.Name | Type | Unit | Default | Description |
| :--- | :--- | :--- | :--- | :--- |
| `fs.mass` | Float | [kg] | `0.04f` | Mass of the drone. |
| `fs.useMA` | Uint8 | N/A | `1` | Enable/Disable Moving Average Filter (0: Off, 1: On). |
| `fs.maN` | Int32 | [samples] | `15` | Moving Average window length (clamped 1..200). (e.g., 15 samples at 100Hz = 0.15s). |

#### Log Variables (Available Logs)

The following values are available for logging under the `log.fs` group.

| Name | Type | Unit | Description |
| :--- | :--- | :--- | :--- |
| `Fsx_raw`, `Fsy_raw`, `Fsz_raw` | Float | [gf] | Raw spring tension before filtering. |
| **`Fsx`, `Fsy`, `Fsz`** | Float | [gf] | **Final spring tension after Moving Average filtering.** |
| `Ft` | Float | [gf] | Scaled thrust magnitude from PWM. |
| `FTwx`, `FTwy`, `FTwz` | Float | [gf] | World Frame Thrust Vector ($\vec{F}_T^W$) components (Debug). |
| `maw_dx`, `maw_dy`, `maw_dz` | Float | [gf] | Mass times Dynamic Acceleration ($m\vec{a}_{dyn}^W$) components (Debug). |
| `Ft` | Float | [gf] | PWM 기반 추력 크기 스칼라 |
| `FTwx`, `FTwy`, `FTwz` | Float | [gf] | 월드 좌표계 추력 벡터($\vec{F}_T^W$) 성분 (Debug) |
| `maw_dx`, `maw_dy`, `maw_dz` | Float | [gf] | 질량 곱하기 동역학적 가속도($m\vec{a}_{dyn}^W$) 성분 (Debug) |
