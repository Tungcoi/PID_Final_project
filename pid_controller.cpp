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

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   this->k_p = Kpi;
   this->k_i = Kii;
   this->k_d = Kdi;
   this->lim_max_output = lim_max_output;
   this->lim_min_output = lim_min_output;
   this->cte = 0.0;
   this->diff_cte = 0.0;
   this->sum_cte = 0.0;
   this->delta_t = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
    if (this->cte == 0.0 && this->sum_cte == 0.0) {
        this->cte = cte;
    }

    if (std::abs(this->delta_t) > 0.0001) {
        this->diff_cte = (cte - this->cte) / this->delta_t;
    } else {
        this->diff_cte = 0.0;
    }

    this->sum_cte += cte * this->delta_t;
    this->cte = cte;

}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   // TODO: Tính toán giá trị điều khiển PID và giới hạn trong phạm vi đầu ra
    double control = - (this->k_p * this->cte) - (this->k_d * this->diff_cte) + (this->k_i * this->sum_cte);

    // Kiểm tra và giới hạn giá trị điều khiển trong khoảng lim_min_output và lim_max_output
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