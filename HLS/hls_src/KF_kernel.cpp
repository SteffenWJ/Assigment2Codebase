#include "KF_kernel.h"

void KalmanFilterKernel(float din[6], float dout[6], float q, float r , ap_uint<32> clockLow) {

#pragma HLS INTERFACE ap_memory storage_type=ram_1p port=din
#pragma HLS INTERFACE ap_memory storage_type=ram_1p port=dout
#pragma HLS INTERFACE s_axilite port=q bundle=AXI_CPU
#pragma HLS INTERFACE s_axilite port=r bundle=AXI_CPU
#pragma HLS INTERFACE s_axilite port=return bundle=AXI_CPU
#pragma HLS INTERFACE ap_none   port=clockLow

		KF_data_t din_[6];
		KF_data_t dout_[6];
		static KF_data_t clockOld = 0;
		KF_data_t deltaT = (KF_data_t)((clockLow-clockOld)/10); //It is devided by 10 a to change the
//		if (deltaT == 0) { //This was also added for testing the filter
//			deltaT = 0.1;
//		}
		clockOld = (KF_data_t)clockLow;
		//read in all state variables at din
		for (int i = 0; i < (6); i++) {
#pragma HLS pipeline off
//#pragma HLS UNROLL
			din_[i] = (KF_data_t)din[i];
		}
		clockOld = clockLow;
		static KF_data_t Q[36] = {
				q, 0, 0, 0, 0, 0,
				0, q, 0, 0, 0, 0,
				0, 0, q, 0, 0, 0,
				0, 0, 0, q, 0, 0,
				0, 0, 0, 0, q, 0,
				0, 0, 0, 0, 0, q
		};
		static KF_data_t R[9] = {
				r, 0, 0,
				0, r, 0,
				0, 0, r
		};
		//This is where the orginal prediction step was
		static KF_data_t x_hat[N_STATE_VARS] = { din_[0], din_[1], din_[2], 0, 0, 0 };
		//static KF_data_t x_hat[6] = { din[0], din[1], din[2], 0, 0, 0 };
		//init P estimation
		static KF_data_t P_hat[36] = {
				1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0,
				0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1
		};
		KF_data_t A[36] = {
				1,0,0,deltaT,0,0,
				0,1,0,0,deltaT,0,
				0,0,1,0,0,deltaT,
				0,0,0,1,0,0,
				0,0,0,0,1,0,
				0,0,0,0,0,1
		};
		KF_data_t B[18] = {
				0.5*deltaT*deltaT,0,0,
				0,0.5*deltaT*deltaT,0,
				0,0,0.5*deltaT*deltaT,
				deltaT,0,0,
				0,deltaT,0,
				0,0,deltaT
		};
		KF_data_t H[18] = {
				1,0,0,0,0,0,
				0,1,0,0,0,0,
				0,0,1,0,0,0
		};

		KF_data_t I[36] = {
				1,0,0,0,0,0,
				0,1,0,0,0,0,
				0,0,1,0,0,0,
				0,0,0,1,0,0,
				0,0,0,0,1,0,
				0,0,0,0,0,1
		};
		static KF_data_t u[3] = {0,0,0};

		KF_data_t z[3];
		KF_data_t x[6];
		KF_data_t P[36];

		KF_data_t x_minus[6];
		KF_data_t P_minus[36];

		KF_data_t x_plus[6];
		KF_data_t P_plus[36];

		KF_data_t tmp_mat_1[36];
		KF_data_t tmp_mat_2[36];

		KF_data_t tmp_mat_3[36];

		/******Load data step******/

		//read current measurements (t+1)
		//for (int j = 0; j < N_MEAS_VARS; j++) z[j] = (din_[j]);
//		for (int j = 0; j < 3; j++){
#pragma HLS pipeline off
#pragma HLS LOOP_TRIPCOUNT max=3
//			z[j] = (din[j]);
//		}
		for (int j = 0; j < 3; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=3
			z[j] = (din_[j]);
		}
		//set state variables according to previous estimate (t)
		for (int j = 0; j < 6; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=3
			x[j] = x_hat[j];
		}
		//set Predict matrix according to previous estimate (t)
		for (int j = 0; j < 36; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=36
			P[j] = P_hat[j];
		}
		// Predict step
		matMultiply<KF_data_t, 6, 6, 6>(A, x, tmp_mat_1, 6, 6, 1);
		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(B, u, tmp_mat_2, N_STATE_VARS, N_CTRL_VARS, 1);
		matAdd<KF_data_t, N_STATE_VARS, N_STATE_VARS>(tmp_mat_1, tmp_mat_2, x_minus, N_STATE_VARS, 1);

		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(A, P, tmp_mat_1, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS);
		matTranspose<KF_data_t, N_STATE_VARS, N_STATE_VARS>(A, tmp_mat_2, N_STATE_VARS, N_STATE_VARS);
		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(tmp_mat_1, tmp_mat_2, tmp_mat_3, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS);
		matAdd<KF_data_t, N_STATE_VARS, N_STATE_VARS>(tmp_mat_3, Q, P_minus, N_STATE_VARS, N_STATE_VARS);

		for (int j = 0; j < N_CTRL_VARS; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=3
			u[j] = (din_[N_MEAS_VARS+j]);
		}

		for (int j = 0; j < N_STATE_VARS; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=6
			x_plus[j] = x_minus[j];
		}
		for (int j = 0; j < N_STATE_VARS*N_STATE_VARS; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=36
			P_plus[j] = P_minus[j];
		}

		// Update step
		KF_data_t y_bar[N_MEAS_VARS];
		KF_data_t H_T[N_STATE_VARS*N_MEAS_VARS];
		KF_data_t S[N_MEAS_VARS*N_MEAS_VARS];
		KF_data_t S_inv[N_MEAS_VARS*N_MEAS_VARS];
		KF_data_t K[N_STATE_VARS*N_MEAS_VARS];

		matTranspose<KF_data_t, 6, 6>(H, H_T, 3, 6);

		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(H, x_minus, tmp_mat_3, N_MEAS_VARS, N_STATE_VARS, 1);
		matSubtract<KF_data_t, N_STATE_VARS, N_STATE_VARS>(z, tmp_mat_3, y_bar, N_MEAS_VARS, 1);

		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(H, P_minus, tmp_mat_3, N_MEAS_VARS, N_STATE_VARS, N_STATE_VARS);
		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(tmp_mat_3, H_T, tmp_mat_2, N_MEAS_VARS, N_STATE_VARS, N_MEAS_VARS);
		matAdd<KF_data_t, N_STATE_VARS, N_STATE_VARS>(tmp_mat_2, R, S, N_MEAS_VARS, N_MEAS_VARS);

		matDiagInverse<KF_data_t, N_MEAS_VARS>(S, S_inv, N_MEAS_VARS);
		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(P_minus, H_T, tmp_mat_3, N_STATE_VARS, N_STATE_VARS, N_MEAS_VARS);
		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(tmp_mat_3, S_inv, K, N_STATE_VARS, N_MEAS_VARS, N_MEAS_VARS);

		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(K, y_bar, tmp_mat_3, N_STATE_VARS, N_MEAS_VARS, 1);
		matAdd<KF_data_t, N_STATE_VARS, N_STATE_VARS>(x_minus, tmp_mat_3, x_plus, N_STATE_VARS, 1);

		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(K, H, tmp_mat_2, N_STATE_VARS, N_MEAS_VARS, N_STATE_VARS);
		matSubtract<KF_data_t, N_STATE_VARS, N_STATE_VARS>(I, tmp_mat_2, tmp_mat_1, N_STATE_VARS, N_STATE_VARS);
		matMultiply<KF_data_t, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS>(tmp_mat_1, P_minus, P_plus, N_STATE_VARS, N_STATE_VARS, N_STATE_VARS);

		// Write data
		for (int j = 0; j < 6; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=6
			x_hat[j] = x_plus[j];
		}
		for (int j = 0; j < 36; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=36
			P_hat[j] = P_plus[j];
		}
		for (int j = 0; j < 6; j++){
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=6
			dout_[j] = x_plus[j];
		}


		for (int i = 0; i < 6; i++) {
#pragma HLS pipeline off
//#pragma HLS UNROLL
#pragma HLS LOOP_TRIPCOUNT max=6
			dout[i] = (float)dout_[i];
		}
}


