#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

const double pi= PI;

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.4389;//0;//-0.43244; //-0.37;0.479/-0.43244
float offset_Enc3_rad = 0.22689;//03; //0.27;-0.188/0.203

// Your global varialbes.  

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(robo, ".my_vars")
float robo = 10.0;

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float theta1_dh2 = 0;
float theta2_dh2 = 0;
float theta3_dh2 = 0;

float t1m = 0;
float t2m = 0;
float t3m = 0;


float x = 0;
float y = 0;
float z = 0;

float r = 0;
float gamma = 0;
float alpha = 0;
float beta = 0;


// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


	*tau1 = 0;
	*tau2 = 0;
	*tau3 = 0;


	//Motor torque limitation(Max: 5 Min: -5)

	// save past states
	if ((mycount%50)==0) {

		theta1array[arrayindex] = theta1motor;
		theta2array[arrayindex] = theta2motor;

		if (arrayindex >= 100) {
			arrayindex = 0;
		} else {
			arrayindex++;
		}

	}

	if ((mycount%500)==0) {
		if (whattoprint > 0.5) {
			serial_printf(&SerialA, "I love robotics\n\r");
		} else {
			printtheta1motor = theta1motor;
			printtheta2motor = theta2motor;
			printtheta3motor = theta3motor;
			float L = 10;


			float theta1 = theta1motor;
			float theta2  = theta2motor - pi/2;
			float theta3 = pi/2 - theta2motor + theta3motor;

			x = L*cos(theta1)*(cos(theta2 + theta3) + cos(theta2) );
			y = L*sin(theta1)*(cos(theta2 + theta3) + cos(theta2) );
			z = L*(1- sin(theta2) - sin(theta2 + theta3));


			r = sqrt(x*x + y*y);
			gamma = -atan2(z-10,r);
			alpha = acos((r*r + (z-10)*(z-10))/(20*sqrt(r*r + (z-10)*(z-10))));
			beta = acos((200-(r*r+(z-10)*(z-10)))/200);
			theta1_dh2 = atan2(y,x);
			theta2_dh2 = gamma - alpha;
			theta3_dh2 = pi - beta;

			t1m = theta1_dh2;
			t2m = theta2_dh2 + PI/2;
			t3m = theta3_dh2 + theta2_dh2;



			SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		}
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
		GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
	}


	Simulink_PlotVar1 = theta1motor;
	Simulink_PlotVar2 = theta2motor;
	Simulink_PlotVar3 = theta3motor;
	Simulink_PlotVar4 = 0;

	mycount++;
}



void printing(void){
	serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
    serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",x,y,z);

    serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",t1m*180/PI,t2m*180/PI,t3m*180/PI);

}

