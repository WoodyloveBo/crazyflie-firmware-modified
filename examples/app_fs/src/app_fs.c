/* ============================================================
 * app_fs.c
 * Spring tension estimation (World frame) with Moving Average
 *  - Uses raw IMU acc (acc.x/y/z) and MoCap quaternion (locSrv.q*)
 *  - Fs^W = F_T^W - m * a_W^dyn
 *  - a_W^dyn = R(q) * (acc_body[G]*G) - [0,0,G]
 *  - Moving Average per axis with window length N = fs.maN
 * ============================================================
 */

#include "app.h"
#include "log.h"
#include "param.h"
#include "console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "system.h"

// -------------------- Parameters --------------------
static float   fs_mass   = 0.04f;  // [kg] drone mass
static uint8_t fs_useMA  = 1;      // 0=off, 1=on
static int32_t fs_maN    = 15;     // window length (samples)

PARAM_GROUP_START(fs)
  PARAM_ADD(PARAM_FLOAT,  mass,  &fs_mass)
  PARAM_ADD(PARAM_UINT8,  useMA, &fs_useMA)
  PARAM_ADD(PARAM_INT32,  maN,   &fs_maN)   // clamp 1..200 internally
PARAM_GROUP_STOP(fs)

// -------------------- Outputs (Logs) ----------------
// Raw spring tension [gf] (before filtering)
static float fs_Fsx_raw = 0.f, fs_Fsy_raw = 0.f, fs_Fsz_raw = 0.f;
// Filtered spring tension [gf]
static float fs_Fsx = 0.f, fs_Fsy = 0.f, fs_Fsz = 0.f;
// Thrust scalar [gf]
static float fs_Ft  = 0.f;
// Debug: F_T^W components [gf]
static float fs_FTwx = 0.f, fs_FTwy = 0.f, fs_FTwz = 0.f;
// Debug: m*a_W^dyn components [gf]
static float fs_maw_dx = 0.f, fs_maw_dy = 0.f, fs_maw_dz = 0.f;

LOG_GROUP_START(fs)
  LOG_ADD(LOG_FLOAT, Fsx_raw, &fs_Fsx_raw)
  LOG_ADD(LOG_FLOAT, Fsy_raw, &fs_Fsy_raw)
  LOG_ADD(LOG_FLOAT, Fsz_raw, &fs_Fsz_raw)

  LOG_ADD(LOG_FLOAT, Fsx, &fs_Fsx)
  LOG_ADD(LOG_FLOAT, Fsy, &fs_Fsy)
  LOG_ADD(LOG_FLOAT, Fsz, &fs_Fsz)

  LOG_ADD(LOG_FLOAT, Ft,  &fs_Ft)

  LOG_ADD(LOG_FLOAT, FTwx, &fs_FTwx)
  LOG_ADD(LOG_FLOAT, FTwy, &fs_FTwy)
  LOG_ADD(LOG_FLOAT, FTwz, &fs_FTwz)

  LOG_ADD(LOG_FLOAT, maw_dx, &fs_maw_dx)
  LOG_ADD(LOG_FLOAT, maw_dy, &fs_maw_dy)
  LOG_ADD(LOG_FLOAT, maw_dz, &fs_maw_dz)
LOG_GROUP_STOP(fs)

// -------------------- Constants ---------------------
#define G 9.80665f
#define NEWTON_PER_GF 0.00980665f
#define GF_PER_NEWTON (1.0f / NEWTON_PER_GF)
#define LAMBDA_ZERO_MAX 1000.0f  // lambda ∈ [0,1000] → Ft=0
#define MA_MAX_N 200

//   TOC IDs -----------------------
// Raw body acc [G]
static logVarId_t id_axB = 0, id_ayB = 0, id_azB = 0;
// PWM thrust
static logVarId_t id_lambda = 0;
// MoCap quaternion (external pose via localization service)
static logVarId_t id_qx = 0, id_qy = 0, id_qz = 0, id_qw = 0;

// -------------------- Math Helpers ------------------
static inline void quatNormalize(float* x, float* y, float* z, float* w) {
  const float n = sqrtf((*x)*(*x) + (*y)*(*y) + (*z)*(*z) + (*w)*(*w));
  if (n > 1e-6f) { *x /= n; *y /= n; *z /= n; *w /= n; }
}

static inline void quatToR(const float qx, const float qy, const float qz, const float qw, float R[3][3]) {
  const float xx = qx*qx, yy = qy*qy, zz = qz*qz;
  const float xy = qx*qy, xz = qx*qz, yz = qy*qz;
  const float wx = qw*qx, wy = qw*qy, wz = qw*qz;

  R[0][0] = 1.f - 2.f*(yy + zz);
  R[0][1] = 2.f*(xy - wz);
  R[0][2] = 2.f*(xz + wy);
  R[1][0] = 2.f*(xy + wz);
  R[1][1] = 1.f - 2.f*(xx + zz);
  R[1][2] = 2.f*(yz - wx);
  R[2][0] = 2.f*(xz - wy);
  R[2][1] = 2.f*(yz + wx);
  R[2][2] = 1.f - 2.f*(xx + yy);
}

// -------------------- Moving Average State ----------
// 각각의 축(Fsx, Fsy, Fsz)에 대해 링버퍼 + 런닝서姆
static float ma_buf_x[MA_MAX_N];
static float ma_buf_y[MA_MAX_N];
static float ma_buf_z[MA_MAX_N];
static int   ma_idx = 0;      // 공용 인덱스(동일 길이 가정)
static int   ma_count = 0;    // 채워진 개수(<= N)
static int   maN_active = 15; // 현재 적용 중인 N
static float sum_x = 0.f, sum_y = 0.f, sum_z = 0.f;

static inline int clampMaN(int n) {
  if (n < 1) n = 1;
  if (n > MA_MAX_N) n = MA_MAX_N;
  return n;
}

static void maReset(int N) {
  maN_active = clampMaN(N);
  ma_idx = 0;
  ma_count = 0;
  sum_x = sum_y = sum_z = 0.f;
  for (int i = 0; i < MA_MAX_N; i++) {
    ma_buf_x[i] = 0.f;
    ma_buf_y[i] = 0.f;
    ma_buf_z[i] = 0.f;
  }
}

static void maPush(float x, float y, float z) {
  // 윈도우 길이가 바뀌었으면 리셋
  int N_target = clampMaN(fs_maN);
  if (N_target != maN_active) {
    maReset(N_target);
  }

  // 오래된 샘플 제거
  if (ma_count >= maN_active) {
    sum_x -= ma_buf_x[ma_idx];
    sum_y -= ma_buf_y[ma_idx];
    sum_z -= ma_buf_z[ma_idx];
  } else {
    ma_count++;
  }

  // 새 샘플 추가
  ma_buf_x[ma_idx] = x;
  ma_buf_y[ma_idx] = y;
  ma_buf_z[ma_idx] = z;
  sum_x += x;
  sum_y += y;
  sum_z += z;

  // 인덱스 갱신
  ma_idx++;
  if (ma_idx >= maN_active) ma_idx = 0;
}

static void maGet(float* x_avg, float* y_avg, float* z_avg) {
  const float denom = (ma_count > 0) ? (float)ma_count : 1.f;
  *x_avg = sum_x / denom;
  *y_avg = sum_y / denom;
  *z_avg = sum_z / denom;
}
// Crazyswarm world is z-up → gravity is negative Z
const float g_vec[3] = {0.f, 0.f, -G};

// -------------------- App Main ----------------------
void appMain(void) {
  vTaskDelay(M2T(500));
  consolePrintf("app_fs(MA): waiting for log TOC...\n");

  // --- Acquire TOC IDs (blocking retry) ---
  while (1) {
    // Raw body-frame accelerometer (unit: G)
    if (!id_axB) id_axB = logGetVarId("acc", "x");
    if (!id_ayB) id_ayB = logGetVarId("acc", "y");
    if (!id_azB) id_azB = logGetVarId("acc", "z");

    // PWM / thrust command
    if (!id_lambda) id_lambda = logGetVarId("stabilizer", "thrust");

    // External MoCap quaternion from localization service
    if (!id_qx) id_qx = logGetVarId("locSrv", "qx");
    if (!id_qy) id_qy = logGetVarId("locSrv", "qy");
    if (!id_qz) id_qz = logGetVarId("locSrv", "qz");
    if (!id_qw) id_qw = logGetVarId("locSrv", "qw");

    if (id_axB && id_ayB && id_azB && id_lambda && id_qx && id_qy && id_qz && id_qw)
      break;

    consolePrintf("app_fs(MA): waiting TOC acc/stabilizer/locSrv...\n");
    vTaskDelay(M2T(200));
  }
  consolePrintf("app_fs(MA): IDs ready\n");

  // MA 초기화
  maReset(fs_maN);

  // -------------------- Main Loop --------------------
  while (1) {
    // 1) Raw acc in body [G] -> [m/s^2]
    const float axB = logGetFloat(id_axB) * G;
    const float ayB = logGetFloat(id_ayB) * G;
    const float azB = logGetFloat(id_azB) * G;

    // 2) External quaternion (MoCap)
    float qx = logGetFloat(id_qx);
    float qy = logGetFloat(id_qy);
    float qz = logGetFloat(id_qz);
    float qw = logGetFloat(id_qw);
    quatNormalize(&qx, &qy, &qz, &qw);

    // 3) Rotation matrix R (body -> world)
    float R[3][3];
    quatToR(qx, qy, qz, qw, R);

    // 4) World total accel and dynamic accel
    const float aW_total_x = R[0][0]*axB + R[0][1]*ayB + R[0][2]*azB;
    const float aW_total_y = R[1][0]*axB + R[1][1]*ayB + R[1][2]*azB;
    const float aW_total_z = R[2][0]*axB + R[2][1]*ayB + R[2][2]*azB;

    const float aW_dyn_x = aW_total_x;
    const float aW_dyn_y = aW_total_y;
    const float aW_dyn_z = aW_total_z - G;    // subtract gravity (0,0,G)

    // 5) Thrust scalar from PWM
    const float lambda = logGetFloat(id_lambda);
    float Ft_gf = 0.0f;
    if (lambda > LAMBDA_ZERO_MAX) {
      // ↙ 너의 캘리브레이션 다항식으로 바꿔도 됨
      Ft_gf = (-6.5341e-9f) * lambda * lambda + (1.5165e-3f) * lambda + 0.7149f;
      if (Ft_gf < 0.0f) Ft_gf = 0.0f;
    }
    fs_Ft = Ft_gf;

    // 6) Thrust vector in world: FT^W = R * [0,0,Ft]^T
    const float FTw_x = R[0][2]*Ft_gf;
    const float FTw_y = R[1][2]*Ft_gf;
    const float FTw_z = R[2][2]*Ft_gf;
    fs_FTwx = FTw_x;
    fs_FTwy = FTw_y;
    fs_FTwz = FTw_z;

    // 7) m * a_W^dyn -> [gf]
    const float m = fs_mass;
    const float maw_dx_gf = (m * aW_dyn_x) * GF_PER_NEWTON;
    const float maw_dy_gf = (m * aW_dyn_y) * GF_PER_NEWTON;
    const float maw_dz_gf = (m * aW_dyn_z) * GF_PER_NEWTON;
    const float mgw_dx_gf = -(fs_mass * g_vec[0]) * GF_PER_NEWTON;
    const float mgw_dy_gf = -(fs_mass * g_vec[1]) * GF_PER_NEWTON;
    const float mgw_dz_gf = -(fs_mass * g_vec[2]) * GF_PER_NEWTON;
    
    fs_maw_dx = maw_dx_gf;
    fs_maw_dy = maw_dy_gf;
    fs_maw_dz = maw_dz_gf;

    // 8) Spring tension (raw): Fs^W = FT^W - m a_W^dyn
    const float Fsx_raw = maw_dx_gf - FTw_x - mgw_dx_gf;
    const float Fsy_raw = maw_dy_gf - FTw_y - mgw_dy_gf;
    const float Fsz_raw = maw_dz_gf - FTw_z - mgw_dz_gf;

    fs_Fsx_raw = Fsx_raw;
    fs_Fsy_raw = Fsy_raw;
    fs_Fsz_raw = Fsz_raw;

    // 9) Moving Average
    if (fs_useMA) {
      maPush(Fsx_raw, Fsy_raw, Fsz_raw);
      float mx, my, mz;
      maGet(&mx, &my, &mz);
      fs_Fsx = mx;
      fs_Fsy = my;
      fs_Fsz = mz;
    } else {
      fs_Fsx = Fsx_raw;
      fs_Fsy = Fsy_raw;
      fs_Fsz = Fsz_raw;
    }

    // ~100 Hz
    vTaskDelay(M2T(10));
  }
}
// Moving Average Filter 사용.
