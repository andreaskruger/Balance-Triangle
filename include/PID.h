#ifndef PID_H
#define PID_h

/*Defines*/

/*Prototypes*/

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
void PID_setProportional(double P);

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
void PID_setIntegral(double I);

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
void PID_setDerivate(double D);

/**
 * @brief Set the Proportional object
 * 
 * @param P 
 */
float PID_angle(float angle);


#endif