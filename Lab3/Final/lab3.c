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
float dt = 0.001;
float threshold1 = 0.01;
float threshold2 = 0.06;
float threshold3 = 0.02;

//Inverse Dynamics Gains
float kp1 = 300.0;
float kd1 = 4;
float ki1 = 420;

float kp2 = 5000;
float kd2 = 250;
float ki2 = 280;

float kp3 = 7000;
float kd3 = 300;
float ki3 = 260;

//float kp1 = 85.0;
//float kd1 = 5;
//float ki1 = 420;
//
//float kp2 = 200;
//float kd2 = 20;
//float ki2 = 280;
//
//float kp3 = 200;
//float kd3 = 20;
//float ki3 = 260;

//PD Gains
float fkp1 = 300;
float fkp2 = 350;
float fkp3 = 300;

float fkd1 = 4;
float fkd2 = 4;
float fkd3 = 4;


float theta_dotdot = 0.0;

float e1 = 0;
float e2 = 0;
float e3 = 0;

float x_desired = 0;
float y_desired = 0;
float z_desired = 0;

//Friction Coefficients
float posviscous1 = 0.14;
float negviscous1 = 0.165;
float poscoloumb1 = 0.3637;
float negcoloumb1 = 0.2948;
float slope1 = 3.6;

float posviscous2 = 0.10;
float negviscous2 = 0.10;
float poscoloumb2 = 0.250;
float negcoloumb2 = 0.40;
float slope2 = 3.6;

float posviscous3 = 0.12;
float negviscous3 = 0.2132;
float poscoloumb3 = 0.5339;
float negcoloumb3 = 0.5190;
float slope3 = 3.6;

//Without Mass
float p1 = 0.0300;
float p2 = 0.0128;
float p3 = 0.0076;
float p4 = 0.0753;
float p5 = 0.0298;

//With Mass
// float p1 = 0.0466;
// float p2 = 0.0388;
// float p3 = 0.0284;
// float p4 = 0.1405;
// float p5 = 0.1298;

float a_theta2;
float a_theta3;

float sintheta2;
float costheta2;
float sintheta3;
float costheta3;
float g = 9.81;

float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;

float a0,a1,a2,a3;
float tau1cur,tau2cur,tau3cur;

float desired_1, desired_2, desired_3;
float desired_dot_1, desired_dot_2, desired_dot_3;
float desired_doubledot_1, desired_doubledot_2, desired_doubledot_3;

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


//Trajectory Generation
float cubic_func(float a0, float a1, float a2, float a3, float t) {
    return a0*t*t*t + a1*t*t + a2*t + a3;
}
float dot(float a0, float a1, float a2, float t) {
    return 3*a0*t*t + 2*a1*t + a2;
}
float doubledot(float a0, float a1, float t) {
    return 6*a0*t + 2*a1;
}

void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    //
    if (counter == 8000)
        {
            counter = 0;
        }

    float time = counter*0.001;

    //Coefficients for different time steps
    if (counter < 330){
        a0 = -27.826474107465835; a1 = 13.774104683195590; a2 = 0; a3 = 0.25;
    }
    if (counter >= 330 && counter < 4000){
        a0 = 0; a1 = 0; a2 = 0; a3 = 0.75;
    }
    if (counter >= 4000 && counter < 4330){
        a0 = 27.826474107496760; a1 = -347.6917939731718; a2 = 1445.863594625530; a3 = -2000.530017811164;
    }
    if (counter >= 4330 && counter < 8000){
        a0 = 0; a1 = 0; a2 = 0; a3 = 0.25;
    }
    //Desired Trajectories
    desired_1 = cubic_func(a0, a1 , a2, a3, time);
    desired_dot_1 = dot(a0, a1, a2, time);
    desired_doubledot_1 = doubledot(a0, a1, time);

    desired_2 = cubic_func(a0, a1 , a2, a3, time);
    desired_dot_2 = dot(a0, a1, a2, time);
    desired_doubledot_2 = doubledot(a0, a1, time);

    desired_3 = cubic_func(a0, a1 , a2, a3, time);
    desired_dot_3 = dot(a0, a1, a2, time);
    desired_doubledot_3 = doubledot(a0, a1, time);

    // Calculate/Update omegas
    omega(theta1motor, theta2motor, theta3motor);

//   Desired theta - current theta
    float e1 = desired_1 - theta1motor;
    float e2 = desired_2 - theta2motor;
    float e3 = desired_3 - theta3motor;


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

    int mode = 0; //Mode = 0: Inverse Dynamics Control; Mode = 1: PD
    if (mode == 0){
        // acceleration of joint 2/3
//
            float D1 = p1;
            float D2 = -p3*sin(theta3motor-theta2motor);
            float D3 = -p3*sin(theta3motor-theta2motor);
            float D4 = p2;
            float C1 = 0;
            float C2 = -p3*cos(theta3motor-theta2motor)*Omega3;
            float C3 = p3*cos(theta3motor-theta2motor)*Omega2;
            float C4 = 0;
            float G1 = -p4*g*sin(theta2motor);
            float G2 = -p5*g*cos(theta3motor);


            a_theta2 = desired_doubledot_2 + kp2*(e2) + kd2*(desired_dot_2-Omega2);
            a_theta3 = desired_doubledot_2 + kp3*(e3) + kd3*(desired_dot_3-Omega3);

            *tau1 = J1*desired_doubledot_2 + kp1*(e1)+kd1*(desired_dot_2-Omega1);
            *tau2 = (D1*a_theta2+D2*a_theta2)+(C1*Omega2+C2*Omega3)+G1;
            *tau3 = (D3*a_theta2+D4*a_theta3)+(C3*Omega2+C4*Omega3)+G2;
    }
    else if(mode == 1){//Feed Forward + PD
        *tau1 = 0.0167 * desired_doubledot_1 + fkp1 * e1 + fkd1 * (desired_dot_1 - Omega1); // + ki1 * integral1;
        *tau2 = 0.03 * desired_doubledot_2 + fkp2 * e2 + fkd2 * (desired_dot_2 - Omega2); // + ki2 * integral2;
        *tau3 = 0.0128 * desired_doubledot_3 + fkp3 * e3 + fkd3 * (desired_dot_3 - Omega3);
    }

//
     //  Friction Compensation
    //If omega greater than 0.1 (Max Velocity) for joint 1
    if (Omega1 > 0.1) {
        *tau1 = *tau1 + 0.6*(Viscouspos1 * Omega1 + Coulombpos1) ;
        }
    //If omega greater than -0.1 (Min Velocity) for joint 1
    else if (Omega1 <-0.1) {
        *tau1 = *tau1 +0.6*(Viscousneg1 * Omega1 + Coulombneg1);
    }
    //If omega between min and max velocity for joint 1
    else {
        *tau1 = *tau1 + 0.6*(slope1*Omega1);
    }

    //If omega greater than 0.05(Max Velocity) for joint 2
    if (Omega2 > 0.05) {
        *tau2 = *tau2 + 0.6*(Viscouspos2 * Omega2 + Coulombpos2);
    }
    //If omega greater than -0.05 (Min Velocity) for joint 2
    else if (Omega2 <-0.05) {
        *tau2 = *tau2 + 0.6*(Viscousneg2 * Omega2 + Coulombneg2);
    }
    //If omega between min and max velocity for joint 2
    else {
        *tau2 = *tau2 + 0.6*(slope2*Omega2);
    }
    //If omega greater than -0.05 (Max Velocity) for joint 3
    if (Omega3 > 0.05) {
        *tau2 = *tau2 + 0.6*(Viscouspos3 * Omega3 + Coulombpos3) ;
    }
    //If omega greater than -0.05 (Min Velocity) for joint 3
    else if (Omega3 <-0.05) {
        *tau2 = *tau2 + 0.6*(Viscousneg3 * Omega3 + Coulombneg3);
    }
    //If omega between min and max velocity for joint 3
    else {
        *tau2 = *tau2 + 0.6*(slope3*Omega3);
    }

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



    Simulink_PlotVar1 = desired_1;  //yellow
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

