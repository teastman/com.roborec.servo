/*
 * Servo.cpp
 *
 * Created: 07/05/2012 12:46:20 PM
 *  Author: teastman
 */ 

#include "Servo.h"

Servo::Servo()
{
	m_minPulseWidth = 600;
	m_maxPulseWidth = 2400;
	m_maxVelocity = 500;
	m_limitVelocity = 100;
	m_pulseVelocity = pulseVelocity(100);
	m_currentPulse = 1500;
	m_destinationPulse = 1500;
	m_nextPulse = 1500;
	m_minPulse = 600;
	m_maxPulse = 2400;
	m_invert = false;
	m_normalize = false;
	m_applyVelocityLimit = false;
}

Servo::~Servo()
{
	// Destroy things
}

void Servo::rotateTo(uint8_t angle)
{
	if(angle > 180) 
		angle = 180;
		
	if(m_invert)
		angle = 180 - angle;
		
	if(m_normalize)
		angle = normalizeAngle(angle);
	
	m_destinationPulse = degreesToMicros(angle);
	
	if(m_destinationPulse > m_maxPulse)
		m_destinationPulse = m_maxPulse;
		
	if(m_destinationPulse<m_minPulse)
		m_destinationPulse = m_minPulse;
}

void Servo::rotateTo(uint8_t angle, uint8_t velocity)
{
	rotateTo(angle);
	m_pulseVelocity = pulseVelocity(velocity);
	m_applyVelocityLimit = true;
}

uint16_t Servo::calculateNextPulse()
{
	m_currentPulse = m_nextPulse;
	if(m_applyVelocityLimit)
	{
		if(m_currentPulse == m_destinationPulse)
		{
			// Calculated first because it is the most common scenario.
			// do nothing;
		}
		else if(m_destinationPulse > m_currentPulse)
		{
			m_nextPulse = m_pulseVelocity + m_currentPulse;
			if(m_nextPulse > m_destinationPulse)
				m_nextPulse = m_destinationPulse;
		}
		else
		{
			m_nextPulse = m_currentPulse - m_pulseVelocity;
			if(m_nextPulse < m_destinationPulse)
				m_nextPulse = m_destinationPulse;
		}
	}
	else
	{
		m_nextPulse = m_destinationPulse;
	}
	return m_nextPulse;
}	
	 
uint8_t Servo::currentAngle() const
{
	uint8_t result = microsToDegrees(m_currentPulse);
	
	if(m_invert) 
		result = 180 - result;
	
	if(m_normalize)
		result = deNormalizeAngle(result);
	
	return result;
}
	
uint8_t Servo::destinationAngle() const
{
	uint8_t result =  microsToDegrees(m_destinationPulse);
	
	if(m_invert)
		result = 180 - result;
	
	if(m_normalize)
		result = deNormalizeAngle(result);
	
	return result;
}

uint8_t Servo::nextAngle() const
{
	uint8_t result =  microsToDegrees(m_nextPulse);
	
	if(m_invert)
		result = 180 - result;
	
	if(m_normalize)
		result = deNormalizeAngle(result);
	
	return result;
}

uint16_t Servo::currentPulse() const
{
	return m_currentPulse;
}

uint16_t Servo::destinationPulse() const
{
	return m_destinationPulse;
}

uint16_t Servo::nextPulse() const
{
	return m_nextPulse;
}	

uint16_t Servo::minPulseWidth()
{
	return m_minPulseWidth;
}

void Servo::minPulseWidth(uint16_t pulse)
{
	m_minPulseWidth = pulse;
}

uint16_t Servo::maxPulseWidth()
{
	return m_maxPulseWidth;
}

void Servo::maxPulseWidth(uint16_t pulse)
{
	m_maxPulseWidth = pulse;
}	

uint16_t Servo::maxVelocity() const
{
	return m_maxVelocity;
}
	
void Servo::maxVelocity(uint16_t maxVelocity)
{
	m_maxVelocity = maxVelocity;
}
	
uint8_t	 Servo::limitVelocity() const
{
	return m_limitVelocity;
}	

void Servo::limitVelocity(uint8_t velocity)
{	
	if(velocity > 100)
		velocity = 100;
		
	m_limitVelocity = velocity;
	m_pulseVelocity = pulseVelocity(velocity);
	m_applyVelocityLimit = true;
}	

void Servo::unlimitVelocity()
{
	m_applyVelocityLimit = false;
}	

bool Servo::isVelocityLimited()
{
	return m_applyVelocityLimit;
}	
	
void Servo::limitAngle(uint8_t minAngle, uint8_t maxAngle)
{
	if(m_invert)
	{
		uint8_t minAng = minAngle;
		minAngle = 180 - maxAngle;
		maxAngle = 180 - minAng;
	}		
	
	m_minPulse = degreesToMicros(minAngle);
	m_maxPulse = degreesToMicros(maxAngle);
}	

uint8_t Servo::minAngle() const
{
	uint8_t result =  (m_invert) ? microsToDegrees(m_maxPulse) : microsToDegrees(m_minPulse);
	return (m_invert) ? 180-result : result;
}
	
void Servo::minAngle(uint8_t minAngle)
{
	if(m_invert)
		m_maxPulse = degreesToMicros(180 - minAngle);
	else
		m_minPulse = degreesToMicros(minAngle);
}
	
uint8_t Servo::maxAngle() const
{
	uint8_t result =  (m_invert) ? microsToDegrees(m_minPulse) : microsToDegrees(m_maxPulse);
	return (m_invert) ? 180-result : result;
}
	
void Servo::maxAngle(uint8_t maxAngle)
{
	if(m_invert)
		m_minPulse = degreesToMicros(180 - m_maxPulse);
	else
		m_maxPulse = degreesToMicros(m_maxPulse);
}	
	
bool Servo::invert() const
{
	return m_invert;
}
	
void Servo::invert(bool invert)
{
	if(invert != m_invert)
	{
		uint8_t minAng = minAngle();
		uint8_t maxAng = maxAngle();
		m_invert = invert;
		limitAngle(minAng, maxAng);
	}
	else
	{
		m_invert = invert;
	}		
}
	
bool Servo::normalize() const
{
	return m_normalize;
}
	
void Servo::normalize(bool normalize)
{
	m_normalize = normalize;
}

// the / 50 is to translate the max velocity from per second to per update.
// The updates are being sent out at a rate of 50 Hz or once every 20us
uint16_t Servo::pulseVelocity(uint8_t velocity) const
{
	if(velocity > 100)
		velocity = 100;
		
	return (uint16_t)(((double)m_maxVelocity / 50.0) * ((double)velocity / 100.0) * ((double)(m_maxPulseWidth - m_minPulseWidth) / 180.0));
}

// Converts a give angle to the appropriate microsecond burst.
uint16_t Servo::degreesToMicros(uint8_t val) const
{
	return (m_maxPulseWidth - m_minPulseWidth) / 180 * val + m_minPulseWidth;
}

// Converts a microsecond burst into an angle.
uint8_t Servo::microsToDegrees(uint16_t val) const
{
	return (uint8_t)(180.0 / (double)(m_maxPulseWidth - m_minPulseWidth) * (val - m_minPulseWidth));
}

// Convert angle from 0-180 onto minAngle-maxAngle.
uint8_t Servo::normalizeAngle(uint8_t angle) const
{
	uint8_t minAng = minAngle();
	return (uint8_t)((double)angle / 180.0 * (maxAngle() - minAng) + minAng);
}

// Convert angle from minAngle-maxAngle onto 0-180.
uint8_t Servo::deNormalizeAngle(uint8_t angle) const
{
	uint8_t minAng = minAngle();
	return (uint8_t)(((double)(angle - minAng))/((double)(maxAngle() - minAng)) * 180.0);
}