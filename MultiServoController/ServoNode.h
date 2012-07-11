/*
 * ServoNode.h
 *
 * Created: 07/05/2012 6:26:21 PM
 *  Author: Tyler
 */ 


#ifndef SERVONODE_H_
#define SERVONODE_H_

#include <inttypes.h>
#include "Servo.h"
#include "MaskNode.h"

struct ServoNode{
	volatile uint8_t*	port;
	uint8_t		mask;
	MaskNode*	maskNode;
	Servo*		servo;
	ServoNode*	previous;
	ServoNode*	next;
	
	ServoNode(volatile uint8_t* p_port, uint8_t p_mask, MaskNode* p_maskNode, Servo* p_servo, ServoNode* p_previous, ServoNode* p_next)
	{
		port = p_port;
		mask = p_mask;
		maskNode = p_maskNode;
		servo = p_servo;
		previous = p_previous;
		next = p_next;
	}
};

#endif /* SERVONODE_H_ */