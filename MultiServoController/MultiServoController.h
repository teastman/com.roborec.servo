/*
 * MultiServoController.h
 *
 * Created: 07/05/2012 6:34:57 PM
 *  Author: Tyler
 */ 


#ifndef MULTISERVOCONTROLLER_H_
#define MULTISERVOCONTROLLER_H_

#define _useTimer1

#include <inttypes.h>
#include <stdlib.h> 
#include "Servo.h"

#ifndef F_CPU
#error "F_CPU not defined for ServoController.h"
#endif

#if defined(__AVR_ATmega328P__)
#define NUM_PORTS 3
#else
#error "Unsupported AVR type"
#endif

void attach(Servo *servo, volatile uint8_t *port, uint8_t mask);
void dettach(Servo *servo);

#endif /* MULTISERVOCONTROLLER_H_ */