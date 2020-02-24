#ifndef _VOLTAGE_H
#define _VOLTAGE_H
#include <CayenneLPP.h>

extern CayenneLPP lpp;
extern float voltage;
extern float height;
extern uint16_t adc_reading;

void initVoltage();
void readSensorValues();

#endif