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
   this->k_p = k_p;
   this->k_i = k_i;
   this->k_d = k_d;
   this->lim_max_output = lim_max_output;
   this->lim_min_output = lim_min_output;
   this->delta_t = 0.0;
   this->error_p = 0.0;
   this->error_i = 0.0;
   this->error_d = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   this->error_p = cte;

   // TODO: Cập nhật lỗi Derivative (kiểm tra delta_t để tránh chia cho 0)
    if (this->delta_t > 0.0) {
        this->error_d = (cte - this->error_p) / this->delta_t;
    } else {
        this->error_d = 0.0;
    }

    // TODO: Cập nhật lỗi Integral
    this->error_i += cte * this->delta_t;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   // TODO: Tính toán giá trị điều khiển PID và giới hạn trong phạm vi đầu ra
    double control = (this->k_p * this->error_p) + (this->k_d * this->error_d) + (this->k_i * this->error_i);

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