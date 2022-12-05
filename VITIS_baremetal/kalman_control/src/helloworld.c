#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"

#include "xbram.h"
#include "xkalmanfilterkernel.h"
#include "xkalmanfilterkernel_hw.h"

#include "data.h"

#define BRAM0(A) ((volatile u32*)px_config0->MemBaseAddress)[A]
#define BRAM1(A) ((volatile u32*)px_config1->MemBaseAddress)[A]

XBram x_bram0;
XBram_Config *px_config0;

XBram x_bram1;
XBram_Config *px_config1;

XKalmanfilterkernel kf_kernel;
XKalmanfilterkernel_Config *kf_config;
float q = 0.05;
float r = 0.95;

int main()
{
    init_platform();

    initBRAM();
    //initKF();
	kf_config = XKalmanfilterkernel_LookupConfig(XPAR_KALMANFILTERKERNEL_0_DEVICE_ID);
	int status = XKalmanfilterkernel_CfgInitialize(&kf_kernel, kf_config);
	status = XKalmanfilterkernel_Initialize(&kf_kernel, XPAR_KALMANFILTERKERNEL_0_DEVICE_ID);
	XKalmanfilterkernel_Set_q(&kf_kernel, q);
	XKalmanfilterkernel_Set_r(&kf_kernel, r);
	for (int i = 0; i < 300; ++i) {
		for (int k = 0; k < 6; k++) {
			BRAM0(k) = *(u32 *)(&(din[k+(6*i)]));
		}
	float bufferInput[1000];
	//Reading the memory DIN
	for (int j = 0; j < 6; j++) {
		u32 valIn = BRAM0(j);
		if (j == 0){
			sprintf(bufferInput, "%f", *(float *)(&(valIn)));
			}
		else{
			sprintf(bufferInput, "%s, %f", bufferInput, *(float *)(&(valIn)));
			}
	}
	sprintf(bufferInput, "%s\n\r", bufferInput);

	XKalmanfilterkernel_Start(&kf_kernel);
	while(!XKalmanfilterkernel_IsDone(&kf_kernel)){
	}

	float bufferOutput[1000];
	for (int t = 0; t < 6; t++) {
		u32 valOut = BRAM1(t);
		if (t == 0){
			sprintf(bufferOutput, "%f", *(float *)(&(valOut)));
			}
		else{
			sprintf(bufferOutput, "%s, %f", bufferOutput, *(float *)(&(valOut)));
			}
	}

	sprintf(bufferOutput, "%s\n\r", bufferOutput);
	xil_printf(bufferInput);
	xil_printf(bufferOutput);
	}
	xil_printf("Finished\n\r");
    cleanup_platform();
    return 0;
}
int initKF(){
    int Status;
	kf_config = XKalmanfilterkernel_LookupConfig(XPAR_KALMANFILTERKERNEL_0_DEVICE_ID);
	if (kf_config == (XKalmanfilterkernel_Config*) NULL) {
		return XST_FAILURE;
	}
	Status = XKalmanfilterkernel_CfgInitialize(&kf_kernel, kf_config);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	Status = XKalmanfilterkernel_Initialize(&kf_kernel, XPAR_KALMANFILTERKERNEL_0_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

}

int initBRAM()
{
	//BRAM initialization
	xil_printf("Initializing block RAM...\n\r");
	int Status;

	px_config0 = XBram_LookupConfig(XPAR_BRAM_0_DEVICE_ID);
	if (px_config0 == (XBram_Config *) NULL) {
		return XST_FAILURE;
	}

	Status = XBram_CfgInitialize(&x_bram0, px_config0, px_config0->CtrlBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	px_config1 = XBram_LookupConfig(XPAR_BRAM_1_DEVICE_ID);
	if (px_config1 == (XBram_Config *) NULL) {
		return XST_FAILURE;
	}

	Status = XBram_CfgInitialize(&x_bram1, px_config1, px_config1->CtrlBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	xil_printf("Done Starting BRAM.\r\n");
	return XST_SUCCESS;
}
