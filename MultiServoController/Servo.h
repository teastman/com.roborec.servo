/*
 * Servo.h
 *
 * Created: 07/05/2012 10:17:54 AM
 *  Author: teastman
 */ 

#ifndef SERVO_H_
#define SERVO_H_

#include <inttypes.h>

// us = micro seconds.

class Servo
{
public:
	Servo();
	~Servo();
  
	void		rotateTo(uint8_t angle);			// Set the destination angle in degrees. (0-180)
	void		rotateTo(uint8_t angle, uint8_t velocity);	// Set the destination angle in degrees. (0-180), and the velocity 0-100%
	
	// WARNING: THIS METHOD IS VERY THREAD UNSAFE
	uint16_t	calculateNextPulse();				// Calculates and saves the next angle in the iteration (in us),	
	 
	// Accessors 
	uint8_t		currentAngle() const;				// Get the current angle in degrees. (0-180)
	uint8_t		destinationAngle() const;			// Get the destination angle in degrees. (0-180)
	uint8_t		nextAngle() const;					// Returns the stored next angle in degrees. (0-180)
	
	uint16_t	currentPulse() const;				// Get the current angle in micro seconds. 
	uint16_t	destinationPulse() const;			// Get the destination angle micro seconds. 
	uint16_t	nextPulse() const;					// Returns the stored next angle micro seconds
	
	uint16_t	minPulseWidth();					// Get the min pulse width of the servo. 
	void		minPulseWidth(uint16_t pulse);		// Set the min pulse width of the servo.
	uint16_t	maxPulseWidth();					// Get the max pulse width of the servo.
	void		maxPulseWidth(uint16_t pulse);		// Set the max pulse width of the servo.
	
	uint16_t	maxVelocity() const;				// Get the max velocity. (degrees per second)
	void		maxVelocity(uint16_t maxVelocity);	// Set the max velocity.
	uint8_t		limitVelocity() const;				// Get the velocity limiter. (0-100%)
	void		limitVelocity(uint8_t velocity);	// Set the velocity limiter. (0-100%)
	void		unlimitVelocity();					// clear the velocity limiter, sets the applyVelocityLimit = false, which saves on calculations.
	bool		isVelocityLimited();				// Returns true if limiting is being used.
	
	void		limitAngle(uint8_t minAngle, uint8_t maxAngle);	// Limit the angle between min and max. (0-180)
	uint8_t		minAngle() const;					// Get the minimum angle limit. (0-180)
	void		minAngle(uint8_t minAngle);			// Set the minimum angle limit. (0-180)
	uint8_t		maxAngle() const;					// Get the maximum angle limit. (0-180)
	void		maxAngle(uint8_t maxAngle);			// Set the maximum angle limit. (0-180)
	
	bool		invert() const;						// Get whether the angle is to be inverted.
	void		invert(bool invert);				// Set whether the angle is to be inverted.	
	bool		normalize() const;					// Get whether the angle is normalized across the min-max limit range.
	void		normalize(bool normalize);			// Set whether the angle is normalized across the min-max limit range.
	
	uint16_t	degreesToMicros(uint8_t val) const;		// Convert degrees into micro seconds.
	uint8_t		microsToDegrees(uint16_t val) const;	// Convert micro seconds into degrees.
	
private:
	uint16_t	pulseVelocity(uint8_t velocity) const; // Calculate the current velocity in pulse time per update (20 milli seconds). based on a passed limiting velocity 0-100%
	
	uint8_t		normalizeAngle(uint8_t angle) const;		// Convert angle from 0-180 onto minAngle-maxAngle.
	uint8_t		deNormalizeAngle(uint8_t angle) const;	// Convert angle from minAngle-maxAngle onto 0-180.

	uint16_t	m_minPulseWidth;		// The minimum amount of pulse time (us) to set the servo to position 0, Dependent on the servo spec.  (Default : 600)
	uint16_t	m_maxPulseWidth;		// The maximum amount of pulse time (us) to set the servo to position 180, Dependent on the servo spec.  (Default : 2400)

	uint16_t	m_currentPulse;			// The current estimated angle of the servo based on the last pwm signal sent. (Default : 1500)
	uint16_t	m_nextPulse;			// The angle that will be sent on the next pwm update cycle. (Default : 1500)
	uint16_t	m_destinationPulse;		// The final destination goal angle. (Default : 1500)
	
	uint16_t	m_maxVelocity;			// The max velocity of the servo as described by the manufacturer measured in degrees per second. (Default : 500)
	uint8_t		m_limitVelocity;		// The user defined velocity limit (0-100%) for maxVelocity. (Default : 100)
	uint16_t	m_pulseVelocity;		// The function defined velocity measured in pulse time per update (20 milli seconds).
	bool		m_applyVelocityLimit;	// Boolean, if false servo will move at full speed and not do the calculations for limiting.
	
	uint16_t	m_minPulse;				// Minimum angle limit. (Default : 0)
	uint16_t	m_maxPulse;				// Maximum angle limit. (Default : 180)

	// TODO: add functionality for invert and normalize
	bool		m_invert;				// If true, map 0-180 onto 180-0. (Default : false)
	bool		m_normalize;			// If true, normalizes the angle from 0-180 across minAngle-maxAngle. (Default : false)
};

#endif /* SERVO_H_ */

