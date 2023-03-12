#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <M5Atom.h>
#include <INA3221.h>

extern float Ax,Ay,Az,Wp,Wq,Wr,dp,dq,dr,Voltage;

void sensor_init(void);
void sensor_read(void);

#endif