/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

// PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   this->k_p = Kpi;
   this->k_i = Kii;
   this->k_d = Kdi;
   this->lim_max_output = output_lim_maxi;
   this->lim_min_output = output_lim_mini;
   this->cte = 0.0;
   this->diff_cte = 0.0;
   this->sum_cte = 0.0;
   this->delta_t = 0.0;
   if (is_log) 
   {    
        std::cout<<"k_p = "<< k_p << ",k_i = " << k_i<< ",k_d = " << k_d<<endl;
        std::cout<<"lim_max_output = "<< lim_max_output << ",lim_min_output = " << lim_min_output<<endl;
        std::cout<<"cte = "<< cte << ",diff_cte = " << diff_cte<< ",sum_cte = " << sum_cte<< ",delta_t = " << delta_t<<endl;
   }
   

}


void PID::UpdateError(double update_cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
    if (this->cte == 0.0 && this->sum_cte == 0.0) {
        if (is_log) std::cout<<"UpdateError first time"<<endl;
        this->cte = update_cte;
    }

    if (std::abs(this->delta_t) > 0.0001) {
        this->diff_cte = (update_cte - this->cte) / this->delta_t;
    } else {
        this->diff_cte = 0.0;
    }

    this->sum_cte += cte * this->delta_t;
    this->cte = update_cte;
    if (is_log) std::cout<<"UpdateError diff_cte = "<< this->diff_cte << ", sum_cte = "<< this->sum_cte << ", cte = "<< this->cte <<endl;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   // // Calculate the PID control value and limit it within the output range
    double control = (this->k_p * this->cte) + (this->k_d * this->diff_cte) + (this->k_i * this->sum_cte);
    if (is_log) std::cout<<"TotalError control = "<< control<<endl;

    // Check and limit the control value within the range lim_min_output and lim_max_output
    if (control > this->lim_max_output) {
        return this->lim_max_output;
    } else if (control < this->lim_min_output) {
        return this->lim_min_output;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   this->delta_t = new_delta_time;
   return this->delta_t;
}