#include "KF_kernel.h"

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

#define INPUT_FILENAME "/home/steffen/uni/projects/assigment2/sensor_and_control_measurements.txt"
#define OUTPUT_FILENAME "/home/steffen/uni/projects/assigment2/hls_output.txt"
#define DELIMITER ','

vector<vector<float>> parseFile(string filename) {
	vector <vector <float> > data;

	ifstream infile( INPUT_FILENAME );

	bool first_line = true;

	while(infile) {

		string s;
		if (!getline(infile, s)) break;

		if (first_line) {
			first_line = false;
			continue;
		}

		istringstream ss(s);
		vector<float> record;

		while(ss) {
			string s;
			if(!getline(ss,s,DELIMITER)) break;
			record.push_back(stof(s));
		}
		data.push_back(record);
	}
	if (!infile.eof()) {
		cerr << "Fooey!\n";
	}
	return data;
}

/*
void writeDataToFile(float dout[N_SAMPLES*N_STATE_VARS], string filename) {

	ofstream f;
	f.open(filename);

	f << "x_hat_0,x_hat_1,x_hat_2,x_hat_3,x_hat_4,x_hat_5\n";

	for (int i = 0; i < N_SAMPLES; i++) {
		for (int j = 0; j < N_STATE_VARS; j++) {
			f << dout[i*N_STATE_VARS+j];
			if (j < N_STATE_VARS-1) {
				f << ",";
			}
		}
		if (i < N_SAMPLES-1) {
			f << "\n";
		}
	}
}
*/
int main(int argc, char *argv[]) {

//	vector<vector<float>> data_vec = parseFile(INPUT_FILENAME);

	//float din[N_SAMPLES*(N_MEAS_VARS+N_CTRL_VARS)];
	//float dout[N_SAMPLES*N_STATE_VARS];


	float dout[300*N_STATE_VARS];
//	for (int i = 0; i < N_SAMPLES; i++) {
//		for (int j = 0; j < (N_MEAS_VARS+N_CTRL_VARS); j++) {
//			din[i*(N_MEAS_VARS+N_CTRL_VARS)+j] = data_vec[i][j];
//		}
//	}

#include "data.h"

	vector <vector <float>> input_data;
	input_data = parseFile(INPUT_FILENAME);
/*
	for (int k = 0; k < 10; ++k) {
		std::cout << input_data[0][k] << std::endl;
	}
*/
	float temp[6];
	int moved_data_step = 6;
	bool clock_low = false;
	for (int i = 0; i < 301; ++i) {
		std::cout << "Din : ";
		for (int j = 0;j < 6; ++j) {
			temp[j] = KF_data_t(din[j+(6*i)]);
			std::cout << temp[j] << " , ";
		}
		std::cout << std::endl;
		KalmanFilterKernel(temp, dout, 0.05, 0.95);  //added a 7 to test if it fixed to simulation
		std::cout << "Dout: ";
		for (int k = 0; k < 6; ++k) {
			std::cout << dout[k] << " , ";
		}
		std::cout << std::endl;
	}
	//KalmanFilterKernel(din, dout, 0.05, 0.95, 7);  //added a 7 to test if it fixed to simulation

//	writeDataToFile(dout, OUTPUT_FILENAME);
/*
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < N_STATE_VARS; j++) {
			cout << dout[i*N_STATE_VARS+j] << "\t";
		}
		cout << "\n";
	}
*/
}
