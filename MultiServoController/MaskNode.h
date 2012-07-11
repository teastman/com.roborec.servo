/*
 * MaskNode.h
 *
 * Created: 07/05/2012 6:36:41 PM
 *  Author: Tyler
 */ 


#ifndef MASKNODE_H_
#define MASKNODE_H_

#include <inttypes.h>

struct MaskNode{
	volatile uint8_t*	port;
	uint8_t		mask;
	uint16_t	time;
	MaskNode*	previous;
	MaskNode*	next;
	
	MaskNode(volatile uint8_t* p_port, uint8_t p_mask, uint16_t p_time, MaskNode* p_previous, MaskNode* p_next)
	{
		port = p_port;
		mask = p_mask;
		time = p_time;
		previous = p_previous;
		next = p_next;
	}
};


#endif /* MASKNODE_H_ */