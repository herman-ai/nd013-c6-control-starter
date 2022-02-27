/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <assert.h>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  output_min = output_lim_mini;
  output_max = output_lim_maxi;
  cte = 0.0;
  icte = 0.0;
  dcte = 0.0;
}


void PID::UpdateError(double icte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
//   assert(delta_time > 0);
  dcte = (icte - cte)/delta_time;
  cte = icte;
  icte += cte*delta_time;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = Kp*cte + Kd*dcte + Ki*icte;
    if (control > output_max)
      control = output_max;
   if (control < output_min)
      control = output_min;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  if (new_delta_time == 0) {
   cout << "*** WARNING: delta_time provided as 0, using default value of 1" << endl;
  }
  delta_time = new_delta_time > 0 ? new_delta_time : 1;
}