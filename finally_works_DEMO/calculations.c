/*
 * calculations.c
 *
 *  Created on: Apr 9, 2020
 *      Author: cberger4
 */

#include "calculations.h"
#include "math.h"

/*
float getFlowRate(float p_delta) //
{
    // Set the constants based on calibrations of sensors
    float C = 0.94; // Concentration
    float beta = 0.8; // d/D
    float epsilon = 1.2; // Expansibility Factors
    float d = 1.2; // diameter
    float rho_1 = 1.2; // density

    float q_m = C/sqrt(1-pow(beta,4))*epsilon*M_PI/4.0*pow(d,2)*sqrt(2*p_delta*rho_1); // mass flowrate equation
    float q_v = q_m/rho_1; // volumetric flowrate equation
    return q_v;
}
*/

float getFlowRate(float p_delta, float N_N2, float N_O2, float N_CO2, float N_H2O, float T_air,float flow_dir) //
{
    // Set the constants based on calibrations of sensors
    float R_bar = 8.314;       // gas constant [J/mol-K]
    float MW_N2 = 28.013;      // molecular weight of N2 [kg/kmol]
    float MW_O2 = 31.999;      // molecular weight of N2 [kg/kmol]
    float MW_CO2 = 44.010;     // molecular weight of N2 [kg/kmol]
    float MW_H2O = 18.015;     // molecular weight of N2 [kg/kmol]
    float C = 1.2;             // discharge coefficient (Calculated by Ashwin)
    float d = 0.008;           // diameter of the throat [m]
    float D = 0.01795;         // entrance diameter [m]
    float p_air = 101325;      // absolute pressure of the air [Pa]

    float R_mix = R_bar*1000/(MW_N2*N_N2 + MW_O2*N_O2 + MW_CO2*N_CO2 + MW_H2O*N_H2O);

    float density = p_air/(T_air*R_mix);
    float density_atm = p_air/(293*287);

//    float A_1 = C/(sqrt(1-pow(d/D,4)))*(M_PI/4)*pow(d,2);
    //float Const=0.000057726;
    float m_dot = flow_dir*sqrt(abs(p_delta)/(p_air/(T_air*R_mix))); // SLPM
    //float m_dot = A_1*sqrt(2*p_delta*(p_air/(T_air*R_mix)));   // mass flowrate
    float q_v = (m_dot)/60; // get volumetric flowrate in L/min

    return q_v;
}


float getVolume(float flowrate_current, float flowrate_past, float frequency)
{
    flowrate_current *= 1000;
    flowrate_past *= 1000;
    float period = 1/frequency; //Period in seconds, "delta x of integral"
//    float midpoint = (flowrate_current + flowrate_past)/2; //midpoint
//    float volume = midpoint * period; //discrete volume
    float flowrate_min = 0;
    if (flowrate_current < flowrate_past)
    {
        flowrate_min = flowrate_current;
    }
    else
    {
        flowrate_min = flowrate_past;
    }

//    float volume = 0.5*abs(flowrate_current-flowrate_past)*period + flowrate_min*period;
    float volume = period*flowrate_current;


    return volume;
}

float filter(float *A, float *B, float uk, float u_km1, float u_km2, float y_km1, float y_km2)
{
    // this function uses a predefined 2-pole (2nd order) LPF

    float yk = - A[0] * y_km1 - A[1] * y_km2 + B[0] * uk
            + B[1] * u_km1 + B[2] * u_km2;
    return yk;
}

