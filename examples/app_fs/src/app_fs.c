#include "app.h"
#include "log.h"
#include "param.h"
#include "console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "system.h"

// -------------------- Parameters --------------------
static float fs_mass = 0.04f;  // [kg] 기체 질량 (기본 40 g)

PARAM_GROUP_START(fs)
PARAM_ADD(PARAM_FLOAT, mass, &fs_mass)
PARAM_GROUP_STOP(fs)

// -------------------- Outputs (Logs) ----------------
// 스프링 장력 추정치 [gf]
static float fs_Fsx = 0.f, fs_Fsy = 0.f, fs_Fsz = 0.f;
// 추력 스칼라 [gf]
static float fs_Ft = 0.f;

LOG_GROUP_START(fs)
LOG_ADD(LOG_FLOAT, Fsx, &fs_Fsx)
LOG_ADD(LOG_FLOAT, Fsy, &fs_Fsy)
LOG_ADD(LOG_FLOAT, Fsz, &fs_Fsz)
LOG_ADD(LOG_FLOAT, Ft,  &fs_Ft)
LOG_GROUP_STOP(fs)

// -------------------- Constants ---------------------
#define G 9.80665f                         // [m/s^2]
#define NEWTON_PER_GF 0.00980665f          // 1 gf = 0.00980665 N
#define GF_PER_NEWTON (1.0f / NEWTON_PER_GF)
#define LAMBDA_ZERO_MAX 1000.0f            // lambda ∈ [0,1000] -> Ft=0

// -------------------- TOC IDs -----------------------
static logVarId_t id_axW = 0, id_ayW = 0, id_azW = 0;  // [G] World frame accel (gravity removed)
static logVarId_t id_lambda = 0;                       // stabilizer.thrust

// -------------------- App Main ----------------------
void appMain(void) {
  vTaskDelay(M2T(500));  // 초기화 대기
  consolePrintf("app_fs: waiting for log TOC...\n");

  // --- Acquire TOC IDs (blocking retry) ---
  while (1) {
    if (!id_axW)   id_axW   = logGetVarId("stateEstimate", "ax");
    if (!id_ayW)   id_ayW   = logGetVarId("stateEstimate", "ay");
    if (!id_azW)   id_azW   = logGetVarId("stateEstimate", "az");
    if (!id_lambda) id_lambda = logGetVarId("stabilizer", "thrust");

    if (id_axW && id_ayW && id_azW && id_lambda) break;
    vTaskDelay(M2T(200));
  }
  consolePrintf("app_fs: IDs ready\n");

  // -------------------- Main Loop --------------------
  while (1) {
    // 1) 가속도 [m/s^2] (stateEstimate.* = G 단위, z축은 중력 제거됨)
    const float ax_ms2 = logGetFloat(id_axW) * G;
    const float ay_ms2 = logGetFloat(id_ayW) * G;
    const float az_ms2 = logGetFloat(id_azW) * G;

    // 2) 추력 (lambda→gf)
    const float lambda = logGetFloat(id_lambda);
    float fti_gf = 0.0f;
    if (lambda > LAMBDA_ZERO_MAX) {
      // 실험 기반 PWM→추력 보정식
      fti_gf = (-3.27e-8f) * lambda * lambda + 0.00390f * lambda - 49.2f; // [gf]
    }
    fs_Ft = fti_gf;

    // 3) m a^W [gf] (z에 mg 더하기)
    const float mg_gf = (fs_mass * G) * GF_PER_NEWTON;
    const float maw_x_gf = (fs_mass * ax_ms2) * GF_PER_NEWTON;
    const float maw_y_gf = (fs_mass * ay_ms2) * GF_PER_NEWTON;
    const float maw_z_gf = (fs_mass * az_ms2) * GF_PER_NEWTON + mg_gf;

    // 4) 스프링 장력 추정 [gf]:  f_s^W = m a^W - f_t e3
    fs_Fsx = maw_x_gf;
    fs_Fsy = maw_y_gf;
    fs_Fsz = maw_z_gf - fti_gf;

    // 100 Hz 주기
    vTaskDelay(M2T(10));
  }
}
