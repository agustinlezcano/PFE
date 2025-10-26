#include "cinv_geometrica.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

/* ================= Utilidades de matrices 4x4 (CMSIS-DSP) ================= */

static inline void mat4_init(arm_matrix_instance_f32* M, float* data) {
    arm_mat_init_f32(M, 4, 4, data);
}

static inline void mat3_init(arm_matrix_instance_f32* M, float* data) {
    arm_mat_init_f32(M, 3, 3, data);
}

static void mat4_identity(float* T) {
    memset(T, 0, 16*sizeof(float));
    T[0] = 1.0f; T[5] = 1.0f; T[10] = 1.0f; T[15] = 1.0f;
}

static void trotz_f32(float q, float* T) {
    mat4_identity(T);
    float c = arm_cos_f32(q);
    float s = arm_sin_f32(q);
    T[0] = c;   T[1] = -s;
    T[4] = s;   T[5] =  c;
    // T[2]=T[6]=0; T[10]=1; T[15]=1
}

static void trotx_f32(float a, float* T) {
    mat4_identity(T);
    float c = arm_cos_f32(a);
    float s = arm_sin_f32(a);
    T[5] = c;   T[6] = -s;
    T[9] = s;   T[10]=  c;
}

static void transl_f32(float x, float y, float z, float* T) {
    mat4_identity(T);
    T[3]  = x;
    T[7]  = y;
    T[11] = z;
}

static void A_dh_f32(const float dh_row[4], float q, float* T_out) {
    // dh_row = [theta, d, a, alpha]
    float Tz[16], Tx[16], Tl[16], tmp[16];

    // MATLAB: A_dh = trotz(q) * transl(a,0,d) * trotx(alpha)
    trotz_f32(q, Tz);
    transl_f32(dh_row[2], 0.0f, dh_row[1], Tl);
    trotx_f32(dh_row[3], Tx);

    arm_matrix_instance_f32 mTz, mTl, mTx, mTmp, mTout;
    mat4_init(&mTz, Tz); mat4_init(&mTl, Tl); mat4_init(&mTx, Tx);
    mat4_init(&mTmp, tmp); mat4_init(&mTout, T_out);

    // tmp = Tz * Tl
    arm_mat_mult_f32(&mTz, &mTl, &mTmp);
    // Tout = tmp * Tx
    arm_mat_mult_f32(&mTmp, &mTx, &mTout);
}

// Inversa homogénea para transformaciones rígidas (R ortonormal):
// T = [ R p ; 0 1 ] -> T^{-1} = [ R^T  -R^T p ; 0 1 ]
static void invHomog4_f32(const float* T, float* Tinv) {
    mat4_identity(Tinv);

    // Extrae R (3x3) y p (3x1)
    float R[9] = {
        T[0], T[1], T[2],
        T[4], T[5], T[6],
        T[8], T[9], T[10]
    };
    float Rt[9];
    arm_matrix_instance_f32 mR, mRt;
    mat3_init(&mR, (float*)R);
    mat3_init(&mRt, (float*)Rt);
    arm_mat_trans_f32(&mR, &mRt);

    // Copia R^T a Tinv(0:2,0:2)
    Tinv[0] = Rt[0]; Tinv[1] = Rt[1]; Tinv[2]  = Rt[2];
    Tinv[4] = Rt[3]; Tinv[5] = Rt[4]; Tinv[6]  = Rt[5];
    Tinv[8] = Rt[6]; Tinv[9] = Rt[7]; Tinv[10] = Rt[8];

    // -R^T * p
    float p[3] = { T[3], T[7], T[11] };
    float np[3] = {0,0,0};

    // np = -Rt * p
    // [3x3]*[3x1]
    for (int i=0;i<3;i++) {
        float acc = 0.0f;
        acc += Rt[3*i + 0] * p[0];
        acc += Rt[3*i + 1] * p[1];
        acc += Rt[3*i + 2] * p[2];
        np[i] = -acc;
    }
    Tinv[3]  = np[0];
    Tinv[7]  = np[1];
    Tinv[11] = np[2];
}

// Multiplica 4x4: C = A * B (CMSIS)
static void mat4_mul(const float* A, const float* B, float* C) {
    arm_matrix_instance_f32 mA, mB, mC;
    // arm_mat_* no es const-correct, hacemos cast
    mat4_init(&mA, (float*)A);
    mat4_init(&mB, (float*)B);
    mat4_init(&mC, C);
    arm_mat_mult_f32(&mA, &mB, &mC);
}

// Aplica T * [v;1] (punto) -> salida 3D
static void apply_T_point(const float* T, const float v[3], float out[3]) {
    out[0] = T[0]*v[0] + T[1]*v[1] + T[2]*v[2] + T[3];
    out[1] = T[4]*v[0] + T[5]*v[1] + T[6]*v[2] + T[7];
    out[2] = T[8]*v[0] + T[9]*v[1] + T[10]*v[2] + T[11];
}

/* ===================== Función principal ===================== */
//Si Base y Tool son matrices identidad no es necesario calcularle la inversa #############
int cinv_geometrica_f32(const float dh[4][4],
                        const float T_obj_in[4][4],
                        const float Base[4][4], const float Tool[4][4],
                        const float offset[4], const float lim[4][2],
                        float qf[4]) {

    // Buffers (alineados por conveniencia)
    float T_obj[16], Ttmp1[16], Ttmp2[16], Binv[16], Toinv[16];

    // Copias de entrada a buffers lineales row-major
    memcpy(T_obj, (const float*)T_obj_in,  sizeof(T_obj));
    // inv(Base) * T_obj * inv(Tool)
    invHomog4_f32((const float*)Base, Binv);
    invHomog4_f32((const float*)Tool, Toinv);

    mat4_mul(Binv, T_obj, Ttmp1);
    mat4_mul(Ttmp1, Toinv, Ttmp2);
    memcpy(T_obj, Ttmp2, sizeof(T_obj));

    // Pf = T_obj(1:3,4)
    float Pf[3] = {
        T_obj[3],   // x
        T_obj[7],   // y
        T_obj[11]   // z
    };

    // q1
    float ang_base = atan2f(Pf[1], Pf[0]);
    float q1 = ang_base;

    // versor en XY
    float c1 = arm_cos_f32(ang_base);
    float s1 = arm_sin_f32(ang_base);
    float versor[3] = { c1, s1, 0.0f };

    // Pm = Pf - L4 * versor, con L4 = dh(4,3) = dh[3][2]
    float L4 = dh[3][2];
    float Pm[3] = {
        Pf[0] - L4 * versor[0],
        Pf[1] - L4 * versor[1],
        Pf[2] - L4 * versor[2]
    };

    // T01 = A_dh(dh(1,:), q1)
    float T01[16], T01inv[16];
    A_dh_f32(dh[0], q1, T01);
    invHomog4_f32(T01, T01inv);

    // p2m = inv(T01) * [Pm;1]
    float p2m[3];
    apply_T_point(T01inv, Pm, p2m);

    // B, d, C, q2 (config codo arriba: B - C)
    float B = atan2f(p2m[0], p2m[1]);
    float d = sqrtf(p2m[0]*p2m[0] + p2m[1]*p2m[1]);

    float L2 = dh[1][2];
    float L3 = dh[2][2];

    float numC = d*d + L2*L2 - L3*L3;
    float denC = 2.0f * d * L2;
    float cosC = numC / denC;
    // clamp por estabilidad numérica
    if (cosC > 1.0f)  cosC = 1.0f;
    if (cosC < -1.0f) cosC = -1.0f;
    float C = acosf(cosC);

    float q2 = PI_F*0.5f - (B - C);

    // q3
    float numPhi = L2*L2 + L3*L3 - d*d;
    float denPhi = 2.0f * L2 * L3;
    float cosPhi = numPhi / denPhi;
    if (cosPhi > 1.0f)  cosPhi = 1.0f;
    if (cosPhi < -1.0f) cosPhi = -1.0f;
    float phi = acosf(cosPhi);
    float q3  = -(PI_F - phi);

    // q4 (ángulo ficticio)
    float T1[16], T2[16], T2inv[16];
    A_dh_f32(dh[0], q1, T1);
    float tmpMul[16];
    A_dh_f32(dh[1], q2, tmpMul);
    mat4_mul(T1, tmpMul, T2);

    invHomog4_f32(T2, T2inv);

    float p2f[3];
    apply_T_point(T2inv, Pf, p2f);
    float r = sqrtf(p2f[0]*p2f[0] + p2f[1]*p2f[1]);

    float numG = L4*L4 + L3*L3 - r*r;
    float denG = 2.0f * L4 * L3;
    float cosG = numG / denG;
    if (cosG > 1.0f)  cosG = 1.0f;
    if (cosG < -1.0f) cosG = -1.0f;
    float gamma = acosf(cosG);
    float q4 = PI_F - gamma;

    // Resultado con offset
    qf[0] = q1 - offset[0];
    qf[1] = q2 - offset[1];
    qf[2] = q3 - offset[2];
    qf[3] = q4 - offset[3];

    // Validación contra límites
    int flag = 1;
    for (int i = 0; i < 4; i++) {
        if (qf[i] < lim[i][0] || qf[i] > lim[i][1]) {
            flag = 0;
        }
    }

    return flag;
}
