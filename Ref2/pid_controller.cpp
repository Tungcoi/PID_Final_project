/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>

#include <math.h>
#include <cmath> //fix issue using math.h

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/

    // Initialize PID coefficients
    // Kp, Ki, Kd
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;

    // Initialize PID errors 
    // p_error, i_error, d_error
    // p_error = Kp * (setpoint - actual_value);
    // i_error = Ki * (int_cte += (setpoint - actual_value) * dt);
    // d_error = Kd * ((setpoint - actual_value) - (prev_cte - actual_value) / dt);
    // p_error = 0.0;
    // i_error = 0.0;
    // d_error = 0.0;
    // Initialize PID output limits
    // output_lim_maxi, output_lim_mini
   output_lim_maxi = output_lim_maxi;
   output_lim_mini = output_lim_mini;

   //Detalta time
   delta_time = 1.0; //Default Time of 1 second reference section PID Control in Udacity

}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/

    // Update PID errors  
    d_error = (cte- p_error)/delta_time; // Reference https://www.ni.com/en/shop/labview/pid-theory-explained.html
    p_error = cte; //refence https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
    i_error += cte*delta_time; // reference https://www.ni.com/en/shop/labview/pid-theory-explained.html
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = 0;//Default Output
    
     // Calculate the total control output reference section 1 PID Control in Udacity
     control = Kp * p_error + Ki * i_error + Kd * d_error;
     //delta_time = 1.0;
     // Clamp the control output to the specified limits
    if (control > output_lim_max) //Check if control is greater than output_lim_max
    {
        control = output_lim_max; 
    }
    //else if (control < output_lim_min) //Check if control is less than output_lim_min
    //Fix bug steer is zero
    if (control < output_lim_min)
    {
        control = output_lim_min;
    }

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
    delta_time = new_delta_time; //Update delta_time with new value new_delta_time
    return delta_time;
}