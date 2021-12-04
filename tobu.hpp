#ifndef TOBU_HPP
#define TOBU_HPP
#include "pwm_uart.hpp"
#include "ekf_sensor.hpp"
#include "pico/multicore.h"
extern semaphore_t sem;
extern Matrix<float, 7 ,1> xp ;
extern Matrix<float, 7 ,1> xe ;
extern Matrix<float, 7 ,1> x_sim ;
extern Matrix<float, 7 ,7> P ;
extern Matrix<float, 6 ,1> z ;
extern Matrix<float, 6 ,1> z_sim ;
extern Matrix<float, 6 ,1> z_noise ;
extern Matrix<float, 3, 1> omega_m ;
extern Matrix<float, 3, 1> omega_sim;
extern Matrix<float, 3, 1> domega;
extern Matrix<float, 3, 1> domega_sim;
extern Matrix<float, 6, 6> Q ;
extern Matrix<float, 6, 6> R ;
extern Matrix<float, 7 ,6> G;
extern Matrix<float, 3 ,1> beta;
void MAINLOOP(void);
#endif
