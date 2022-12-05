#include "KF_kernel.h"

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "data.h"
using namespace std;

int main(int argc, char *argv[]) {

	float dout[N_STATE_VARS];
	float temp[6];
	for (int i = 0; i < 300; ++i) {
		std::cout << "Din : ";
		for (int j = 0;j < 6; ++j) {
			temp[j] = KF_data_t(din[j+(6*i)]);
			std::cout << temp[j] << " , ";
		}
		std::cout << std::endl;
		KalmanFilterKernel(temp, dout, 0.05, 0.95, 1);
		std::cout << "Dout: ";
		for (int k = 0; k < 6; ++k) {
			std::cout << dout[k] << " , ";
		}
		std::cout << std::endl;
	}
	std::cout << "Small change to see if it works" << std::endl;
}
