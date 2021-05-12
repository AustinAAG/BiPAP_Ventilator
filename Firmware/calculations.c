/*
 * calculations.c
 *
 *  Created on: Apr 9, 2020
 *      Author: cberger4
 */

#include "calculations.h"
#include "math.h"

float getFlowRate(float p_delta, float N_N2, float N_O2, float N_CO2,
                  float N_H2O, float T_air, float calib_c, int isInhale) //
{
    /* Set the constants based on calibrations of sensors */
    float R_bar = 8.314;       // gas constant [J/mol-K]
    float MW_N2 = 28.013;      // molecular weight of N2 [kg/kmol]
    float MW_O2 = 31.999;      // molecular weight of O2 [kg/kmol]
    float MW_CO2 = 44.010;     // molecular weight of CO2 [kg/kmol]
    float MW_H2O = 18.015;     // molecular weight of H2O [kg/kmol]
    float p_air = 101325;      // absolute pressure of the air [Pa]
    float p_stp = 101325;      // STP pressure [Pa]
    float T_stp = 298;         // STP temperature [K]


    float MW_Inhale = N_O2*MW_O2 + (1-N_O2)*MW_N2;
    float MW_Total = MW_Inhale;

    if (isInhale == 1) {
        // If inhale do nothing
    }
    else {
        MW_Total = N_CO2*MW_CO2+N_H2O*MW_H2O+(1-N_CO2-N_H2O)*MW_Inhale;
    }

    float R_mix = R_bar*1000/MW_Total; // Specific gas constant for the mixture [J/mol-K]

    float density = p_air/(298*R_mix); // Actual density of the air [kg/m^3]
    float density_stp = p_stp/(T_stp*R_mix); // Density of the air at STP [kg/m^3]

    // The calibration constant corrects the calculations using experimental data
    // Since the area is also constant, the calibration constant (in our case) consumes that value
    // which gives units of m^2

    //Calculate the volumetric flow rate using orifice plate equation (https://en.wikipedia.org/wiki/Orifice_plate)
    //Constant consumes C_d,beta,e,area,and multiplier 2
    //The calculations are given in SLPM due to our measuring devices and calibration constant, despite using the actual density

    float q_slpm = calib_c*sqrt(abs(p_delta)/density); // [SLPM]
    float q_lpm = q_slpm*(density/density_stp); // Convert to actual LPM by multiplying the ratio of densities [LPM]

    q_lpm = q_lpm/60; //Convert to per seconds

    return q_lpm;

}

float getVolume(float flowrate_current, float flowrate_past, float frequency)
{
    flowrate_current *= 1000; //Scale by 1000
    flowrate_past *= 1000; // scale by 1000
    float period = 1 / frequency; //Period in seconds, "delta x of integral"

    float volume = period * flowrate_current;

    return volume;
}

float filter(float *A, float *B, float uk, float u_km1, float u_km2,
             float y_km1, float y_km2)
{
    // this function uses a predefined 2-pole (2nd order) LPF

    float yk = -A[0] * y_km1 - A[1] * y_km2 + B[0] * uk + B[1] * u_km1
            + B[2] * u_km2;
    return yk;
}

float adc_to_pressure(int adcResult)
{
    float v_supply = 5.0;
    float p_max = 1.0; // 1 psi
    float p_min = -1.0; // -1 psi

    float out_voltage =  adcResult * (3.3 / 16384.0);

    return (out_voltage - (.1 * v_supply)) * ((p_max - p_min) / (.8 * v_supply))
            + p_min;
}

