#pragma once
#include "arm_math.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// dh: 4x4 -> cada fila: [theta, d, a, alpha] (en rad / metros)
// q_ini no se usa en esta versión pero se mantiene por compatibilidad
// T_obj, Base, Tool: 4x4 homogéneas (rotación ortonormal + traslación)
// offset: 4; lim: [4][2]; qf: 4
int cinv_geometrica_f32(const float dh[4][4],
                        const float T_obj_in[4][4],
                        const float Base[4][4], const float Tool[4][4],
                        const float offset[4], const float lim[4][2],
                        float qf[4]);

#ifdef __cplusplus
}
#endif
