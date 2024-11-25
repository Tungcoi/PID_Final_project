/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
/*
    Student Notes
    Student: Nicolas Nguyen Van Au
    Date Edit: 12/11/2020
    Description:
    PID controller
    PID stand for Proportional Integral Derivative
*/
class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Errors
    */
   
   //Reference documentation url: https://www.ni.com/en/shop/labview/pid-theory-explained.html
    double p_error;//Proportional Error (P) 
    //p_error = Kp * (setpoint - actual_value);
    //Kp: Proportional gain, which determines how strongly the controller reacts to the current error.
    double i_error;//Integral Error (I)
    // i_error = Ki * (int_cte += (setpoint - actual_value) * dt);
    // Ki: Integral gain, which determines how fast the controller integrates the error.
    double d_error;//Derivative Error (D)
    //d_error = Kd * ((setpoint - actual_value) - (prev_cte - actual_value) / dt);
    //Kd:Derivative gain, which measures the rate of change of the error and adjusts accordingly to smooth the response and prevent overshoot.
    

    /*
    * Coefficients
    */

    //Reference documentation url  https://www.ni.com/en/shop/labview/pid-theory-explained.html
    double Kp; //Proportional coefficient
    double Ki; //Integral coefficient
    double Kd; //Derivative coefficient


    /*
    * Output limits
    */

    //Delarce the maximum and minimum output values
    double output_lim_max ;//Maximum output value
    double output_lim_min ;  //Minimum output value


    /*
    * Delta time
    */
    //Declare the delta time
    double delta_time ; //Time interval between each update of the PID controller
    //double prev_cte = 0.0; //Previous cross track error

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


