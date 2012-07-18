/*
 * ServoController.cpp
 *
 * Created: 30/04/2012 12:19:44 PM
 *  Author: Tyler Eastman
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h> 
#include "MultiServoController.h"
#include "ServoNode.h"
#include "MaskNode.h"

static MaskNode *firstClearMaskNode;
static ServoNode *firstServoNode;

// Holds a pointer to the maskNode to be used the next time comparator A interrupts.
static MaskNode *nextInterruptMaskNode;

static bool initialized = false;
// Dependant on the processor speed, 
static uint16_t PRESCALER = 8;
static uint16_t TOP = 0;
static uint16_t SERVO_MAX_PULSE = 2600;

// Function to move a servoNodes maskNode to the appropriate ordered location in the maskNode list.
static void moveMaskNode(ServoNode* servoNode, uint16_t newTime)
{
	// Set up linked mask node variables.
	MaskNode *previousNode = servoNode->maskNode->previous;
	MaskNode *nextNode = servoNode->maskNode->next;
	
	if((servoNode->mask ^ servoNode->maskNode->mask) != 0)
	// If the current nodes mask node is shared with another node, create a new mask node that is detached, and remove it from the old mask.
	{
		MaskNode* maskNode = new MaskNode(servoNode->port, servoNode->mask, newTime, NULL, NULL);
		servoNode->maskNode->mask &= ~(servoNode->mask);
		// Set the 'previousNode' to be the node that was separated from.
		previousNode = servoNode->maskNode;
		servoNode->maskNode = maskNode;
	}
	else
	// Else detach current node from the list.
	{
		if(servoNode->maskNode->next != NULL && servoNode->maskNode->next->previous == servoNode->maskNode)
			servoNode->maskNode->next->previous = servoNode->maskNode->previous;
		if(servoNode->maskNode->previous != NULL && servoNode->maskNode->previous->next == servoNode->maskNode)
			servoNode->maskNode->previous->next = servoNode->maskNode->next;
		// Set the current mask node's time to the new time.
		servoNode->maskNode->time = newTime;
	}

	// Iterate previousNodes looking for a spot.
	while(previousNode != NULL && newTime < previousNode->time)
	{
		nextNode = previousNode;
		previousNode = previousNode->previous;
	}

	// Iterate nextNodes looking for a spot.
	while(nextNode != NULL && newTime > nextNode->time)
	{
		previousNode = nextNode;
		nextNode = nextNode->next;
	}

	// Insert the servoNode->maskNode back into the list and create appropriate links.
	if(previousNode != NULL && previousNode->time == newTime && previousNode->port == servoNode->port)
	// If the current time matches the previous nodes time and port, merge.
	{
		previousNode->mask |= servoNode->mask;
		delete servoNode->maskNode;
		servoNode->maskNode = previousNode;
	} 
	else if(nextNode != NULL && nextNode->time == newTime && nextNode->port == servoNode->port)
	// Else If the current time matches the next nodes time and port, merge.
	{
		nextNode->mask |= servoNode->mask;
		delete servoNode->maskNode;
		servoNode->maskNode = nextNode;
	}
	else
	// Else insert the node in between the previouse and next nodes.
	{
		servoNode->maskNode->previous = previousNode;
		servoNode->maskNode->next = nextNode;
		if(previousNode != NULL)
			previousNode->next = servoNode->maskNode;
		if(nextNode != NULL)
			nextNode->previous = servoNode->maskNode;
	}
}

ISR(TIMER1_COMPA_vect) 
{ 
	// Check that the maskNode to interrupt is not null, and that it's time has arrived.
	// Important to have in a while loop in case multiple mask nodes in a row share an interrupt time.
	// This can happen when two different ports need to interrupt at the same time.
	while(nextInterruptMaskNode != NULL && nextInterruptMaskNode->time <= TCNT1)
	{
		// Apply the mask to the port to set the pins.
		*(nextInterruptMaskNode->port) &= ~(nextInterruptMaskNode->mask);
			
		// Advance the pointer to the next mask to be interrupted.
		nextInterruptMaskNode = nextInterruptMaskNode->next;
			
		// If it's not null, set the next Comparator A time to interrupt to the masks time.
		if(nextInterruptMaskNode != NULL)
		{
			OCR1A = nextInterruptMaskNode->time;
		}
	}
	return;
}

ISR(TIMER1_COMPB_vect) 
{ 
	// Comparator B is used when the max pulse width of any servos has passed.
	// The goal is to use the time between the end of signal sending (usually around 2400) and the next beginning (19999)
	// to calculate the next pulse widths to be sent, and reorder the masks accordingly.
	if(OCR1B >= TOP)
	{
		TCNT1 = 0;
		// Turn on all pins in the clear mask nodes.
		MaskNode* currentMaskNode = firstClearMaskNode;
		while(currentMaskNode != NULL)
		{
			*(currentMaskNode->port) |= currentMaskNode->mask;
			currentMaskNode = currentMaskNode->next;
		}
		OCR1B = SERVO_MAX_PULSE;
	}
	else
	{
		// Iterate through the ServoNodes
		ServoNode* currentServoNode = firstServoNode;
		while(currentServoNode != NULL)
		{
			// Calculate the next pulse width.
			uint16_t nextPulse = currentServoNode->servo->calculateNextPulse();
			
			// If it's different than the current pulse width that is being sent.
			if(nextPulse != currentServoNode->maskNode->time)
			{
				// Move the mask node the the appropriate location.
				moveMaskNode(currentServoNode, nextPulse);
			}
			currentServoNode = currentServoNode->next;
		}
		
		// Once all the mask nodes have been arranged, we need to reset the nextInterruptMaskNode to be the first.
		// Get the maskNode for the firstServoNode, and iterate to the front.
		MaskNode* maskNode = firstServoNode->maskNode;
		while (maskNode->previous != NULL)
		{
			maskNode = maskNode->previous;
		}
		nextInterruptMaskNode = maskNode;
		
		// Set the next interrupt for A to be the first interrupt mask node.
		OCR1A = nextInterruptMaskNode->time;
		OCR1B = TOP;
	}
	return;
}

static void initializeISR()
{  
	if(!initialized)
	{
		firstClearMaskNode = NULL;
		firstServoNode = NULL;
		nextInterruptMaskNode = NULL;
	
		cli();
		#if defined (_useTimer1)
			// Set the initial trigger times.
			OCR1A = 0;
			OCR1B = SERVO_MAX_PULSE;
		
			// Set the timers settings fields TCCR1A & TCCR1B
			// Set WGM (Waveform Generation Mode) to Normal Mode
			TCCR1A = 0;
			TCCR1B = 0;
			TCNT1 = 0;		// (Timer Counter) Clear the current timers count.
			
			// ICR depends on the clock speed F_CPU and the PRESCALER CS.
			// It needs to be set so that the total time of 1 timer cycle is 20 ms = 50 Hz 
			// Hence the magic 50 in the equation TOP = (F_CPU / (PRESCALER * 50)) - 1;
			
			TOP = (F_CPU / (PRESCALER * 50)) - 1;
			TCCR1B |= _BV(CS11); // Set PRESCALER to 8 (010)
			TIFR1 |= _BV(OCF1A) | _BV(OCF1B);						// (Timer Interrupt Flag Register) Clear any existing interrupts for comparator A and B.
			TIMSK1 |=  _BV(OCIE1A) | _BV(OCIE1B);					// (Timer Interrupt Mask Register) Watch comparator A and B.
		#endif 
		sei();
		initialized = true;
	} 
} 

void attach(Servo *servo, volatile uint8_t *port, uint8_t mask)
{
	initializeISR();

	MaskNode* maskNode = new MaskNode(port, mask, servo->nextPulse(), NULL, NULL);
	ServoNode* servoNode = new ServoNode(port, mask, maskNode, servo, NULL, NULL);
	
	// Add the mask - port combination to the clear mask list.
	MaskNode* currentClearMaskNode = firstClearMaskNode;
	bool added = false;
	while(!added)
	{
		if(currentClearMaskNode == NULL)
		{
			firstClearMaskNode = new MaskNode(port, mask, servo->nextPulse(), NULL, NULL);
			added = true;
		}
		else if(currentClearMaskNode->port == maskNode->port)
		{
			currentClearMaskNode->mask |= maskNode->mask;
			added = true;
		}
		else if(currentClearMaskNode->next == NULL)
		{
			currentClearMaskNode->next = new MaskNode(port, mask, servo->nextPulse(), currentClearMaskNode, NULL);
			added = true;
		}
		else
		{
			currentClearMaskNode = currentClearMaskNode->next;	
		}
	}
		
	// Add the servo node the servo node list.
	if(firstServoNode == NULL)
		firstServoNode = servoNode;
	else
	{
		ServoNode* tempNode = firstServoNode;
		while(tempNode->next != NULL)
			tempNode = tempNode->next;
		tempNode->next = servoNode;
		servoNode->previous = tempNode;
		
		// Wire the servo's mask node to the end of the masknode list, then put it in place.
		// Unless this is the first servo node added.
		MaskNode* lastMaskNode = firstServoNode->maskNode;
		while (lastMaskNode->next != NULL)
		{
			lastMaskNode = lastMaskNode->next;
		}
		lastMaskNode->next = maskNode;
		maskNode->previous = lastMaskNode;
		
		// Move the mask node the the appropriate location.
		moveMaskNode(servoNode, servoNode->servo->currentPulse());
	}
}

void dettach(Servo* servo)
{
	// Remove the servo node from the servo node list.
	// Remove the clear mask for the servo node.
	// Remove the mask node for the servo node.
	
	if(firstServoNode != NULL)
	{
		ServoNode* tempNode = firstServoNode;
		while(tempNode != NULL)
		{
			if(tempNode->servo == servo)
			{
				// Remove the servoNode from it's linked list
				if(tempNode->previous != NULL)
					tempNode->previous->next = tempNode->next;
				if(tempNode->next != NULL)
					tempNode->next->previous = tempNode->previous;
				
				// Remove the mask from the clear mask list
				MaskNode* currentClearMaskNode = firstClearMaskNode;
				while(currentClearMaskNode != NULL)
				{
					if(currentClearMaskNode->port == tempNode->port)
					{
						currentClearMaskNode->mask &= ~tempNode->mask;
						if(currentClearMaskNode->mask == 0)
						{
							if(currentClearMaskNode->previous != NULL)
								currentClearMaskNode->previous->next = currentClearMaskNode->next;
							if(currentClearMaskNode->next != NULL)
								currentClearMaskNode->next->previous = currentClearMaskNode->previous;
							delete currentClearMaskNode;
						}
						break;
					}						
					currentClearMaskNode = currentClearMaskNode->next;
				}
				
				// Remove the mask node for the servo
				if((tempNode->maskNode->mask &= ~tempNode->mask) == 0)
				{
					if(tempNode->maskNode->previous != NULL)
						tempNode->maskNode->previous->next = tempNode->maskNode->next;
					if(tempNode->maskNode->next != NULL)
						tempNode->maskNode->next->previous = tempNode->maskNode->previous;
					delete tempNode->maskNode;
				}
				
				delete tempNode;
				break;
			}
			tempNode = tempNode->next;
		}
	}
}