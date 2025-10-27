// Minimal thrust-fitting helper app:
// Publishes m a_IMU^B  and  R_W^B (m g e3) and their sum, all in [gf].
// Uses acc.{x,y,z} (body frame, in G) and stateEstimate quaternion.

#include "app.h"
#include "log.h"
#include "param.h"
#include "console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "string.h"

// ---------- Parameters ----------
static float ft_mass = 0.040f;   // [kg] vehicle mass (default 40 g)

PARAM_GROUP_START(ft)
PARAM_ADD(PARAM_FLOAT, mass, &ft_mass)
PARAM_GROUP_STOP(ft)

// ---------- Constants ----------
#define G               9.80665f              // [m/s^2] 1G
#define NEWTON_PER_GF   0.00980665f           // 1 gf = 0.00980665 N
#define GF_PER_NEWTON   (1.0f/NEWTON_PER_GF)  // ≈ 101.971621 gf/N

// ---------- Logs (all in [gf]) ----------
// m a_IMU^B (gravity-removed body acceleration)
static float ft_mabx = 0.f, ft_maby = 0.f, ft_mabz = 0.f;
// R_W^B (m g e3) : world gravity projected to body
static float ft_mgBx = 0.f, ft_mgBy = 0.f, ft_mgBz = 0.f;
// sum: m a_IMU^B + R_W^B (m g e3)  (== 피팅 타겟의 좌변)
static float ft_sumBx = 0.f, ft_sumBy = 0.f, ft_sumBz = 0.f;

// (선택) 진단용: R_B^W 원소 9개와 R_W^B e3 (원하면 주석 해제)
// static float ft_R00=0, ft_R01=0, ft_R02=0;
// static float ft_R10=0, ft_R11=0, ft_R12=0;
// static float ft_R20=0, ft_R21=0, ft_R22=0;
// static float ft_e3Bx=0, ft_e3By=0, ft_e3Bz=0;

LOG_GROUP_START(ft)
// gravity-removed body term (gf)
LOG_ADD(LOG_FLOAT, mabx, &ft_mabx)
LOG_ADD(LOG_FLOAT, maby, &ft_maby)
LOG_ADD(LOG_FLOAT, mabz, &ft_mabz)
// body gravity term (gf)
LOG_ADD(LOG_FLOAT, mgBx, &ft_mgBx)
LOG_ADD(LOG_FLOAT, mgBy, &ft_mgBy)
LOG_ADD(LOG_FLOAT, mgBz, &ft_mgBz)
// sum (gf)
LOG_ADD(LOG_FLOAT, sumBx, &ft_sumBx)
LOG_ADD(LOG_FLOAT, sumBy, &ft_sumBy)
LOG_ADD(LOG_FLOAT, sumBz, &ft_sumBz)
// // diag (optional)
// LOG_ADD(LOG_FLOAT, R00, &ft_R00)
// LOG_ADD(LOG_FLOAT, R01, &ft_R01)
// LOG_ADD(LOG_FLOAT, R02, &ft_R02)
// LOG_ADD(LOG_FLOAT, R10, &ft_R10)
// LOG_ADD(LOG_FLOAT, R11, &ft_R11)
// LOG_ADD(LOG_FLOAT, R12, &ft_R12)
// LOG_ADD(LOG_FLOAT, R20, &ft_R20)
// LOG_ADD(LOG_FLOAT, R21, &ft_R21)
// LOG_ADD(LOG_FLOAT, R22, &ft_R22)
// LOG_ADD(LOG_FLOAT, e3Bx, &ft_e3Bx)
// LOG_ADD(LOG_FLOAT, e3By, &ft_e3By)
// LOG_ADD(LOG_FLOAT, e3Bz, &ft_e3Bz)
LOG_GROUP_STOP(ft)

// ---------- Utils ----------
static inline void quatToR_BW(const float qx, const float qy, const float qz, const float qw, float R[3][3]) {
  // Rotation: Body -> World
  const float xx=qx*qx, yy=qy*qy, zz=qz*qz;
  const float xy=qx*qy, xz=qx*qz, yz=qy*qz;
  const float wx=qw*qx, wy=qw*qy, wz=qw*qz;
  R[0][0]=1.f-2.f*(yy+zz); R[0][1]=2.f*(xy-wz);   R[0][2]=2.f*(xz+wy);
  R[1][0]=2.f*(xy+wz);     R[1][1]=1.f-2.f*(xx+zz); R[1][2]=2.f*(yz-wx);
  R[2][0]=2.f*(xz-wy);     R[2][1]=2.f*(yz+wx);   R[2][2]=1.f-2.f*(xx+yy);
}

// ---------- TOC IDs ----------
static logVarId_t id_qx=0, id_qy=0, id_qz=0, id_qw=0; // stateEstimate quaternion
static logVarId_t id_axG=0, id_ayG=0, id_azG=0;       // acc.{x,y,z} in [G] (Body frame)

void appMain(void) {
  vTaskDelay(M2T(500));
  consolePrintf("app_ft: start (publishing m a_IMU^B and R_W^B(m g e3) in [gf])\n");

  // Acquire TOC IDs
  while (1) {
    if (!id_qx)  id_qx  = logGetVarId("stateEstimate", "qx");
    if (!id_qy)  id_qy  = logGetVarId("stateEstimate", "qy");
    if (!id_qz)  id_qz  = logGetVarId("stateEstimate", "qz");
    if (!id_qw)  id_qw  = logGetVarId("stateEstimate", "qw");

    if (!id_axG) id_axG = logGetVarId("acc", "x");   // body, [G]
    if (!id_ayG) id_ayG = logGetVarId("acc", "y");
    if (!id_azG) id_azG = logGetVarId("acc", "z");

    if (id_qx && id_qy && id_qz && id_qw && id_axG && id_ayG && id_azG) break;
    consolePrintf("app_ft: waiting for log TOC...\n");
    vTaskDelay(M2T(200));
  }
  consolePrintf("app_ft: IDs ready\n");

  float R_BW[3][3];  // Body->World
  float R_WB[3][3];  // World->Body (= R_BW^T)

  while (1) {
    // --- Read sensors ---
    const float qx = logGetFloat(id_qx);
    const float qy = logGetFloat(id_qy);
    const float qz = logGetFloat(id_qz);
    const float qw = logGetFloat(id_qw);

    // Body-frame accelerometer (in G) -> [m/s^2]
    const float aBx = logGetFloat(id_axG) * G;
    const float aBy = logGetFloat(id_ayG) * G;
    const float aBz = logGetFloat(id_azG) * G;

    // --- Build rotations ---
    quatToR_BW(qx, qy, qz, qw, R_BW);
    // transpose for World->Body
    R_WB[0][0]=R_BW[0][0]; R_WB[0][1]=R_BW[1][0]; R_WB[0][2]=R_BW[2][0];
    R_WB[1][0]=R_BW[0][1]; R_WB[1][1]=R_BW[1][1]; R_WB[1][2]=R_BW[2][1];
    R_WB[2][0]=R_BW[0][2]; R_WB[2][1]=R_BW[1][2]; R_WB[2][2]=R_BW[2][2];

    // (선택) 진단 로그
    // ft_R00=R_BW[0][0]; ft_R01=R_BW[0][1]; ft_R02=R_BW[0][2];
    // ft_R10=R_BW[1][0]; ft_R11=R_BW[1][1]; ft_R12=R_BW[1][2];
    // ft_R20=R_BW[2][0]; ft_R21=R_BW[2][1]; ft_R22=R_BW[2][2];

    // --- Gravity in world and body ---
    // World gravity vector: g_W = [0, 0, -G]
    const float gWx = 0.0f;
    const float gWy = 0.0f;
    const float gWz = -G;

    // Body gravity: g_B = R_W^B * g_W
    const float gBx = R_WB[0][0]*gWx + R_WB[0][1]*gWy + R_WB[0][2]*gWz;
    const float gBy = R_WB[1][0]*gWx + R_WB[1][1]*gWy + R_WB[1][2]*gWz;
    const float gBz = R_WB[2][0]*gWx + R_WB[2][1]*gWy + R_WB[2][2]*gWz;

    // --- Gravity-removed body acceleration: a_IMU^B = a_meas^B - g_B ---
    const float aIMUBx = aBx - gBx;
    const float aIMUBy = aBy - gBy;
    const float aIMUBz = aBz - gBz;

    // --- Convert to forces in [gf] ---
    // 1) m a_IMU^B
    ft_mabx = (ft_mass * aIMUBx) * GF_PER_NEWTON;
    ft_maby = (ft_mass * aIMUBy) * GF_PER_NEWTON;
    ft_mabz = (ft_mass * aIMUBz) * GF_PER_NEWTON;

    // 2) R_W^B (m g e3) = m * (R_W^B * g_W)
    ft_mgBx = (ft_mass * gBx) * GF_PER_NEWTON;
    ft_mgBy = (ft_mass * gBy) * GF_PER_NEWTON;
    ft_mgBz = (ft_mass * gBz) * GF_PER_NEWTON;

    // 3) Sum = m a_IMU^B + R_W^B (m g e3)
    ft_sumBx = ft_mabx + ft_mgBx;
    ft_sumBy = ft_maby + ft_mgBy;
    ft_sumBz = ft_mabz + ft_mgBz;

    // (선택) diag: body e3 = R_W^B * e3_W, 여기선 g_W만 썼으므로 생략 가능
    // const float e3Bx = R_WB[0][2];
    // const float e3By = R_WB[1][2];
    // const float e3Bz = R_WB[2][2];
    // ft_e3Bx = e3Bx; ft_e3By = e3By; ft_e3Bz = e3Bz;

    // ~100 Hz
    vTaskDelay(M2T(10));
  }
}
