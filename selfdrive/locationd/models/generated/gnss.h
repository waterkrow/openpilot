#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3631407973428835199);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5342789079704114990);
void gnss_H_mod_fun(double *state, double *out_1054722445164097470);
void gnss_f_fun(double *state, double dt, double *out_6478813366141022744);
void gnss_F_fun(double *state, double dt, double *out_3372636745199449397);
void gnss_h_6(double *state, double *sat_pos, double *out_3531507309030658532);
void gnss_H_6(double *state, double *sat_pos, double *out_4398972779158541573);
void gnss_h_20(double *state, double *sat_pos, double *out_903109109737014458);
void gnss_H_20(double *state, double *sat_pos, double *out_1115835581583121524);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5825584699279810175);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1125053678659573064);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5825584699279810175);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1125053678659573064);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}