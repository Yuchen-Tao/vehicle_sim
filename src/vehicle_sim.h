#include "std_msgs/String.h"
#include <SDL/SDL.h>
#include <automated_driving_msgs/ObjectState.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <time.h>

class Vehicle {
public:
  Vehicle();
  inline virtual ~Vehicle() {
    /* Clean up */
    SDL_Quit();
    exit(0);
  };
  void SetVideoMode();
  void keyboard_control(ros::NodeHandle &n, ros::Publisher &msg_pub_);
  void create_message(ros::NodeHandle &n, ros::Publisher &msg_pub_);
  void update_state();

private:
  // front steering angle
  double delta_f;
  // the angle(rad) of the current velocityof the center of mass with respect to
  // the longitudinal axis ofthe car
  double beta;
  // acceleration of center of mass
  // double acc;
  // velocity
  double vel;
  // x position of center of mass
  double x;
  double x_der;
  // y position of center of mass
  double y;
  double y_del;
  // inertial heading
  double psi;
  double psi_der;
  // distancefrom the center of the mass of the vehicle to the front axle
  double l_f = 130;
  // distancefrom the center of the mass of the vehicle to the rear axle
  double l_r = 140;

  double dt = 1;
  double acc;

  double time_step;
  double time_counter;
};