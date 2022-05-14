#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.415; //-0.37;
float offset_Enc3_rad = 0.233; //0.27;

// Your global variables.
float px = 0; // x coordinate of end effector
float py = 0; // y coordinate of end effector
float pz = 0; // z coordinate of end effector
float theta1m_IK = 0; // theta1 motor from inverse kinematics
float theta2m_IK = 0; // theta2 motor from inverse kinematics
float theta3m_IK = 0; // theta3 motor from inverse kinematics
long mycount = 0;
long counter = 0;
int cycle = 0;

// Omega calculations
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;
float e1_old = 0;
float integral1_old = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;
float e2_old = 0;
float integral2_old = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;
float e3_old = 0;
float integral3_old = 0;


// Constants
float desired = 0;
float dt = 0.001;
float threshold1 = 0.01;
float threshold2 = 0.06;
float threshold3 = 0.02;

float kp1 = 85.0;
float kd1 = 2.3;
float ki1 = 420;

float kp2 = 50;
float kd2 = 1.7;
float ki2 = 280;

float kp3 = 50;
float kd3 = 1.4;
float ki3 = 260;

float theta_dotdot = 0.0;

float e1 = 0;
float e2 = 0;
float e3 = 0;

float x_desired = 0;
float y_desired = 0;
float z_desired = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(whatnottoprint, ".my_vars")
float whatnottoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

void inverseKinematics(float px, float py, float pz) {
    // Calculate DH angles from end effector position
    float theta1 = atan2(py, px); // DH theta 1
    float z = pz - 10;
    float beta = sqrt(px*px + py*py);
    float L = sqrt(z*z + beta*beta);
    float theta3 = acos((L*L - 200)/200); // DH theta 3
    float theta2 = -theta3/2 - atan2(z, beta); // DH theta 2


    // Convert DH angles to motor angles
    theta1m_IK = theta1;
    theta2m_IK = (theta2 + PI/2);
    theta3m_IK = (theta3 + theta2m_IK - PI/2);
}

// Velocity estimation
void omega(float theta1motor, float theta2motor, float theta3motor) {
    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;

    Theta1_old = theta1motor;

    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;

    Theta2_old = theta2motor;

    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;

    Theta3_old = theta3motor;

    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;
}

float cubic_func(float a1, float a2, float a3, float a4, float t) {
    return a1*t*t*t + a2*t*t + a3*t + a4;
}
float dot(float a1, float a2, float a3, float t) {
    return 3*a1*t*t + 2*a2*t + a3;
}
float doubledot(float a1, float a2, float t) {
    return 6*a1*t + 2*a2;
}

void trajectory(float scale, float t) {
    // polar coordinates for Butterfly Trajectory
    float r = scale*(8 - sin(t) + 2*sin(5*t) - sin(7*t) + 3*cos(2*t) - 2*cos(4*t));
    // origin for trajectory (14, 0, 14)
    x_desired = 14;
    y_desired = r*cos(t) + 0;
    z_desired = r*sin(t) + 14;
}
// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {
    // Forward Kinematics
    px = 10*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    py = 10*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    pz = 10*cos(theta2motor) - 10*sin(theta3motor) + 10;

    // Butterfly Trajectory
    float time = counter*dt;
    trajectory(0.5, time); // determine desired end-effector position
    inverseKinematics(x_desired, y_desired, z_desired); // determine desired thetas

    // PID + Feedforward with Cubic Trajectory
 /*
    //
    if (counter % 2000 == 0) {
        counter = 0;
    }
    if (counter % 1000 == 0) {
        cycle++;
    }

    float time = counter*dt;
    float desired_doubledot = 0;
    float desired_dot = 0;
    if (cycle % 2 != 0) {
        // First cubic function
        desired = cubic_func(-1, 1.5, 0, 0, time);
        desired_doubledot = doubledot(-1, 1.5, time);
        desired_dot = dot(-1, 1.5, 0, time);
    } else {
        // Second cubic function
        desired = cubic_func(1, -4.5, 6, -2, time);
        desired_doubledot = doubledot(1, -4.5, time);
        desired_dot = dot(1, -4.5, 6, time);
    }
*/
    // Calculate omegas
    omega(theta1motor, theta2motor, theta3motor);

/*  // Desired theta - current theta
    float e1 = desired - theta1motor;
    float e2 = desired - theta2motor;
    float e3 = desired - theta3motor;
*/

    e1 = theta1m_IK - theta1motor;
    e2 = theta2m_IK - theta2motor;
    e3 = theta3m_IK - theta3motor;

    // Integral approximation
    float integral1 = integral1_old + (e1 + e1_old) * dt / 2;
    float integral2 = integral2_old + (e2 + e2_old) * dt / 2;
    float integral3 = integral3_old + (e3 + e3_old) * dt / 2;

    // Prevents integral windup
    if (fabs(e1) > threshold1) {
        integral1 = 0;
        integral1_old = 0;
    }

    if (fabs(e2) > threshold2) {
            integral2 = 0;
            integral2_old  = 0;
    }

    if (fabs(e3) > threshold3) {
            integral3 = 0;
            integral3_old = 0;
    }
    // Desired theta_dot and theta_dot_dot for Butterfly Trajectory
    float desired_dot = 0;
    float desired_doubledot = 0;

    // Feedforward + PID Controller
	*tau1 = 0.0167 * desired_doubledot + kp1 * e1 + kd1 * (desired_dot - Omega1) + ki1 * integral1;
	*tau2 = 0.03 * desired_doubledot + kp2 * e2 + kd2 * (desired_dot - Omega2) + ki2 * integral2;
	*tau3 = 0.0128 * desired_doubledot + kp3 * e3 + kd3 * (desired_dot - Omega3)+ ki3 * integral3;

	//Motor torque limitation(Max: 5 Min: -5)
	//Prevents integral windup
	if (*tau1 >= 5) {
	    *tau1 = 5;
	    integral1 = integral1_old;
	} else if (*tau1 < -5) {
	    *tau1 = -5;
	    integral1 = integral1_old;
	}

    if (*tau2 >= 5) {
        *tau2 = 5;
        integral2 = integral2_old;
    } else if (*tau2 < -5) {
        *tau2 = -5;
        integral2 = integral2_old;
    }

    if (*tau3 >= 5) {
        *tau3 = 5;
        integral3 = integral3_old;
    } else if (*tau3 < -5) {
        *tau3 = -5;
        integral3 = integral3_old;
    }

    e1_old = e1;
    e2_old = e2;
    e3_old = e3;
    integral1_old = integral1;
    integral2_old = integral2;
    integral3_old = integral3;


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
			SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		}

		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
		GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
	}


	Simulink_PlotVar1 = desired; //yellow
	Simulink_PlotVar2 = theta1motor; //blue
	Simulink_PlotVar3 = theta2motor; //orange
	Simulink_PlotVar4 = theta3motor; //green

	mycount++;
    counter++;
}

void printing(void){
    // Printing (theta1, theta2, theta3, px, py, pz) and then on a new line (theta1_IK, theta2_IK, theta3_IK)
	serial_printf(&SerialA, "px: %.2f, py: %.2f, pz: %.2f \n\r", px, py, pz);
}

