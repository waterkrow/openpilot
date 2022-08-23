#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3631407973428835199) {
   out_3631407973428835199[0] = delta_x[0] + nom_x[0];
   out_3631407973428835199[1] = delta_x[1] + nom_x[1];
   out_3631407973428835199[2] = delta_x[2] + nom_x[2];
   out_3631407973428835199[3] = delta_x[3] + nom_x[3];
   out_3631407973428835199[4] = delta_x[4] + nom_x[4];
   out_3631407973428835199[5] = delta_x[5] + nom_x[5];
   out_3631407973428835199[6] = delta_x[6] + nom_x[6];
   out_3631407973428835199[7] = delta_x[7] + nom_x[7];
   out_3631407973428835199[8] = delta_x[8] + nom_x[8];
   out_3631407973428835199[9] = delta_x[9] + nom_x[9];
   out_3631407973428835199[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5342789079704114990) {
   out_5342789079704114990[0] = -nom_x[0] + true_x[0];
   out_5342789079704114990[1] = -nom_x[1] + true_x[1];
   out_5342789079704114990[2] = -nom_x[2] + true_x[2];
   out_5342789079704114990[3] = -nom_x[3] + true_x[3];
   out_5342789079704114990[4] = -nom_x[4] + true_x[4];
   out_5342789079704114990[5] = -nom_x[5] + true_x[5];
   out_5342789079704114990[6] = -nom_x[6] + true_x[6];
   out_5342789079704114990[7] = -nom_x[7] + true_x[7];
   out_5342789079704114990[8] = -nom_x[8] + true_x[8];
   out_5342789079704114990[9] = -nom_x[9] + true_x[9];
   out_5342789079704114990[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1054722445164097470) {
   out_1054722445164097470[0] = 1.0;
   out_1054722445164097470[1] = 0;
   out_1054722445164097470[2] = 0;
   out_1054722445164097470[3] = 0;
   out_1054722445164097470[4] = 0;
   out_1054722445164097470[5] = 0;
   out_1054722445164097470[6] = 0;
   out_1054722445164097470[7] = 0;
   out_1054722445164097470[8] = 0;
   out_1054722445164097470[9] = 0;
   out_1054722445164097470[10] = 0;
   out_1054722445164097470[11] = 0;
   out_1054722445164097470[12] = 1.0;
   out_1054722445164097470[13] = 0;
   out_1054722445164097470[14] = 0;
   out_1054722445164097470[15] = 0;
   out_1054722445164097470[16] = 0;
   out_1054722445164097470[17] = 0;
   out_1054722445164097470[18] = 0;
   out_1054722445164097470[19] = 0;
   out_1054722445164097470[20] = 0;
   out_1054722445164097470[21] = 0;
   out_1054722445164097470[22] = 0;
   out_1054722445164097470[23] = 0;
   out_1054722445164097470[24] = 1.0;
   out_1054722445164097470[25] = 0;
   out_1054722445164097470[26] = 0;
   out_1054722445164097470[27] = 0;
   out_1054722445164097470[28] = 0;
   out_1054722445164097470[29] = 0;
   out_1054722445164097470[30] = 0;
   out_1054722445164097470[31] = 0;
   out_1054722445164097470[32] = 0;
   out_1054722445164097470[33] = 0;
   out_1054722445164097470[34] = 0;
   out_1054722445164097470[35] = 0;
   out_1054722445164097470[36] = 1.0;
   out_1054722445164097470[37] = 0;
   out_1054722445164097470[38] = 0;
   out_1054722445164097470[39] = 0;
   out_1054722445164097470[40] = 0;
   out_1054722445164097470[41] = 0;
   out_1054722445164097470[42] = 0;
   out_1054722445164097470[43] = 0;
   out_1054722445164097470[44] = 0;
   out_1054722445164097470[45] = 0;
   out_1054722445164097470[46] = 0;
   out_1054722445164097470[47] = 0;
   out_1054722445164097470[48] = 1.0;
   out_1054722445164097470[49] = 0;
   out_1054722445164097470[50] = 0;
   out_1054722445164097470[51] = 0;
   out_1054722445164097470[52] = 0;
   out_1054722445164097470[53] = 0;
   out_1054722445164097470[54] = 0;
   out_1054722445164097470[55] = 0;
   out_1054722445164097470[56] = 0;
   out_1054722445164097470[57] = 0;
   out_1054722445164097470[58] = 0;
   out_1054722445164097470[59] = 0;
   out_1054722445164097470[60] = 1.0;
   out_1054722445164097470[61] = 0;
   out_1054722445164097470[62] = 0;
   out_1054722445164097470[63] = 0;
   out_1054722445164097470[64] = 0;
   out_1054722445164097470[65] = 0;
   out_1054722445164097470[66] = 0;
   out_1054722445164097470[67] = 0;
   out_1054722445164097470[68] = 0;
   out_1054722445164097470[69] = 0;
   out_1054722445164097470[70] = 0;
   out_1054722445164097470[71] = 0;
   out_1054722445164097470[72] = 1.0;
   out_1054722445164097470[73] = 0;
   out_1054722445164097470[74] = 0;
   out_1054722445164097470[75] = 0;
   out_1054722445164097470[76] = 0;
   out_1054722445164097470[77] = 0;
   out_1054722445164097470[78] = 0;
   out_1054722445164097470[79] = 0;
   out_1054722445164097470[80] = 0;
   out_1054722445164097470[81] = 0;
   out_1054722445164097470[82] = 0;
   out_1054722445164097470[83] = 0;
   out_1054722445164097470[84] = 1.0;
   out_1054722445164097470[85] = 0;
   out_1054722445164097470[86] = 0;
   out_1054722445164097470[87] = 0;
   out_1054722445164097470[88] = 0;
   out_1054722445164097470[89] = 0;
   out_1054722445164097470[90] = 0;
   out_1054722445164097470[91] = 0;
   out_1054722445164097470[92] = 0;
   out_1054722445164097470[93] = 0;
   out_1054722445164097470[94] = 0;
   out_1054722445164097470[95] = 0;
   out_1054722445164097470[96] = 1.0;
   out_1054722445164097470[97] = 0;
   out_1054722445164097470[98] = 0;
   out_1054722445164097470[99] = 0;
   out_1054722445164097470[100] = 0;
   out_1054722445164097470[101] = 0;
   out_1054722445164097470[102] = 0;
   out_1054722445164097470[103] = 0;
   out_1054722445164097470[104] = 0;
   out_1054722445164097470[105] = 0;
   out_1054722445164097470[106] = 0;
   out_1054722445164097470[107] = 0;
   out_1054722445164097470[108] = 1.0;
   out_1054722445164097470[109] = 0;
   out_1054722445164097470[110] = 0;
   out_1054722445164097470[111] = 0;
   out_1054722445164097470[112] = 0;
   out_1054722445164097470[113] = 0;
   out_1054722445164097470[114] = 0;
   out_1054722445164097470[115] = 0;
   out_1054722445164097470[116] = 0;
   out_1054722445164097470[117] = 0;
   out_1054722445164097470[118] = 0;
   out_1054722445164097470[119] = 0;
   out_1054722445164097470[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6478813366141022744) {
   out_6478813366141022744[0] = dt*state[3] + state[0];
   out_6478813366141022744[1] = dt*state[4] + state[1];
   out_6478813366141022744[2] = dt*state[5] + state[2];
   out_6478813366141022744[3] = state[3];
   out_6478813366141022744[4] = state[4];
   out_6478813366141022744[5] = state[5];
   out_6478813366141022744[6] = dt*state[7] + state[6];
   out_6478813366141022744[7] = dt*state[8] + state[7];
   out_6478813366141022744[8] = state[8];
   out_6478813366141022744[9] = state[9];
   out_6478813366141022744[10] = state[10];
}
void F_fun(double *state, double dt, double *out_3372636745199449397) {
   out_3372636745199449397[0] = 1;
   out_3372636745199449397[1] = 0;
   out_3372636745199449397[2] = 0;
   out_3372636745199449397[3] = dt;
   out_3372636745199449397[4] = 0;
   out_3372636745199449397[5] = 0;
   out_3372636745199449397[6] = 0;
   out_3372636745199449397[7] = 0;
   out_3372636745199449397[8] = 0;
   out_3372636745199449397[9] = 0;
   out_3372636745199449397[10] = 0;
   out_3372636745199449397[11] = 0;
   out_3372636745199449397[12] = 1;
   out_3372636745199449397[13] = 0;
   out_3372636745199449397[14] = 0;
   out_3372636745199449397[15] = dt;
   out_3372636745199449397[16] = 0;
   out_3372636745199449397[17] = 0;
   out_3372636745199449397[18] = 0;
   out_3372636745199449397[19] = 0;
   out_3372636745199449397[20] = 0;
   out_3372636745199449397[21] = 0;
   out_3372636745199449397[22] = 0;
   out_3372636745199449397[23] = 0;
   out_3372636745199449397[24] = 1;
   out_3372636745199449397[25] = 0;
   out_3372636745199449397[26] = 0;
   out_3372636745199449397[27] = dt;
   out_3372636745199449397[28] = 0;
   out_3372636745199449397[29] = 0;
   out_3372636745199449397[30] = 0;
   out_3372636745199449397[31] = 0;
   out_3372636745199449397[32] = 0;
   out_3372636745199449397[33] = 0;
   out_3372636745199449397[34] = 0;
   out_3372636745199449397[35] = 0;
   out_3372636745199449397[36] = 1;
   out_3372636745199449397[37] = 0;
   out_3372636745199449397[38] = 0;
   out_3372636745199449397[39] = 0;
   out_3372636745199449397[40] = 0;
   out_3372636745199449397[41] = 0;
   out_3372636745199449397[42] = 0;
   out_3372636745199449397[43] = 0;
   out_3372636745199449397[44] = 0;
   out_3372636745199449397[45] = 0;
   out_3372636745199449397[46] = 0;
   out_3372636745199449397[47] = 0;
   out_3372636745199449397[48] = 1;
   out_3372636745199449397[49] = 0;
   out_3372636745199449397[50] = 0;
   out_3372636745199449397[51] = 0;
   out_3372636745199449397[52] = 0;
   out_3372636745199449397[53] = 0;
   out_3372636745199449397[54] = 0;
   out_3372636745199449397[55] = 0;
   out_3372636745199449397[56] = 0;
   out_3372636745199449397[57] = 0;
   out_3372636745199449397[58] = 0;
   out_3372636745199449397[59] = 0;
   out_3372636745199449397[60] = 1;
   out_3372636745199449397[61] = 0;
   out_3372636745199449397[62] = 0;
   out_3372636745199449397[63] = 0;
   out_3372636745199449397[64] = 0;
   out_3372636745199449397[65] = 0;
   out_3372636745199449397[66] = 0;
   out_3372636745199449397[67] = 0;
   out_3372636745199449397[68] = 0;
   out_3372636745199449397[69] = 0;
   out_3372636745199449397[70] = 0;
   out_3372636745199449397[71] = 0;
   out_3372636745199449397[72] = 1;
   out_3372636745199449397[73] = dt;
   out_3372636745199449397[74] = 0;
   out_3372636745199449397[75] = 0;
   out_3372636745199449397[76] = 0;
   out_3372636745199449397[77] = 0;
   out_3372636745199449397[78] = 0;
   out_3372636745199449397[79] = 0;
   out_3372636745199449397[80] = 0;
   out_3372636745199449397[81] = 0;
   out_3372636745199449397[82] = 0;
   out_3372636745199449397[83] = 0;
   out_3372636745199449397[84] = 1;
   out_3372636745199449397[85] = dt;
   out_3372636745199449397[86] = 0;
   out_3372636745199449397[87] = 0;
   out_3372636745199449397[88] = 0;
   out_3372636745199449397[89] = 0;
   out_3372636745199449397[90] = 0;
   out_3372636745199449397[91] = 0;
   out_3372636745199449397[92] = 0;
   out_3372636745199449397[93] = 0;
   out_3372636745199449397[94] = 0;
   out_3372636745199449397[95] = 0;
   out_3372636745199449397[96] = 1;
   out_3372636745199449397[97] = 0;
   out_3372636745199449397[98] = 0;
   out_3372636745199449397[99] = 0;
   out_3372636745199449397[100] = 0;
   out_3372636745199449397[101] = 0;
   out_3372636745199449397[102] = 0;
   out_3372636745199449397[103] = 0;
   out_3372636745199449397[104] = 0;
   out_3372636745199449397[105] = 0;
   out_3372636745199449397[106] = 0;
   out_3372636745199449397[107] = 0;
   out_3372636745199449397[108] = 1;
   out_3372636745199449397[109] = 0;
   out_3372636745199449397[110] = 0;
   out_3372636745199449397[111] = 0;
   out_3372636745199449397[112] = 0;
   out_3372636745199449397[113] = 0;
   out_3372636745199449397[114] = 0;
   out_3372636745199449397[115] = 0;
   out_3372636745199449397[116] = 0;
   out_3372636745199449397[117] = 0;
   out_3372636745199449397[118] = 0;
   out_3372636745199449397[119] = 0;
   out_3372636745199449397[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3531507309030658532) {
   out_3531507309030658532[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_4398972779158541573) {
   out_4398972779158541573[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4398972779158541573[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4398972779158541573[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4398972779158541573[3] = 0;
   out_4398972779158541573[4] = 0;
   out_4398972779158541573[5] = 0;
   out_4398972779158541573[6] = 1;
   out_4398972779158541573[7] = 0;
   out_4398972779158541573[8] = 0;
   out_4398972779158541573[9] = 0;
   out_4398972779158541573[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_903109109737014458) {
   out_903109109737014458[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1115835581583121524) {
   out_1115835581583121524[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1115835581583121524[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1115835581583121524[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1115835581583121524[3] = 0;
   out_1115835581583121524[4] = 0;
   out_1115835581583121524[5] = 0;
   out_1115835581583121524[6] = 1;
   out_1115835581583121524[7] = 0;
   out_1115835581583121524[8] = 0;
   out_1115835581583121524[9] = 1;
   out_1115835581583121524[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_5825584699279810175) {
   out_5825584699279810175[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_1125053678659573064) {
   out_1125053678659573064[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[6] = 0;
   out_1125053678659573064[7] = 1;
   out_1125053678659573064[8] = 0;
   out_1125053678659573064[9] = 0;
   out_1125053678659573064[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_5825584699279810175) {
   out_5825584699279810175[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_1125053678659573064) {
   out_1125053678659573064[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1125053678659573064[6] = 0;
   out_1125053678659573064[7] = 1;
   out_1125053678659573064[8] = 0;
   out_1125053678659573064[9] = 0;
   out_1125053678659573064[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3631407973428835199) {
  err_fun(nom_x, delta_x, out_3631407973428835199);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5342789079704114990) {
  inv_err_fun(nom_x, true_x, out_5342789079704114990);
}
void gnss_H_mod_fun(double *state, double *out_1054722445164097470) {
  H_mod_fun(state, out_1054722445164097470);
}
void gnss_f_fun(double *state, double dt, double *out_6478813366141022744) {
  f_fun(state,  dt, out_6478813366141022744);
}
void gnss_F_fun(double *state, double dt, double *out_3372636745199449397) {
  F_fun(state,  dt, out_3372636745199449397);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3531507309030658532) {
  h_6(state, sat_pos, out_3531507309030658532);
}
void gnss_H_6(double *state, double *sat_pos, double *out_4398972779158541573) {
  H_6(state, sat_pos, out_4398972779158541573);
}
void gnss_h_20(double *state, double *sat_pos, double *out_903109109737014458) {
  h_20(state, sat_pos, out_903109109737014458);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1115835581583121524) {
  H_20(state, sat_pos, out_1115835581583121524);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5825584699279810175) {
  h_7(state, sat_pos_vel, out_5825584699279810175);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1125053678659573064) {
  H_7(state, sat_pos_vel, out_1125053678659573064);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5825584699279810175) {
  h_21(state, sat_pos_vel, out_5825584699279810175);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1125053678659573064) {
  H_21(state, sat_pos_vel, out_1125053678659573064);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
