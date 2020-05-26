/*
 * calculations.h
 *
 *  Created on: Apr 9, 2020
 *      Author: cberger4
 */

#ifndef CALCULATIONS_H_
#define CALCULATIONS_H_

//float getFlowRate(float p_delta);
float getFlowRate(float p_delta, float N_N2, float N_O2, float N_CO2, float N_H2O, float T_air,float flow_dir);
float getVolume(float p_current, float p_past, float frequency);
float filter(float *A, float *B, float uk, float u_km1, float u_km2, float y_km1, float y_km2);

#endif /* CALCULATIONS_H_ */
