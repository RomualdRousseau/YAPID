/**************************************************************************/
/*!
    @file     PID.cpp
    @author   Romuald Rousseau
    @license  BSD (see license.txt)

    Driver for the CT8Z transmiter

    This is a library for the CT8Z transmiter

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
#include "Arduino.h"

#include "YAPID.h"

int YAPIDController::compute(const int error)
{
	unsigned long now = millis();
	float dt = (now - m_last_time) / 1000.0;
	m_last_time = now;
	
	if(m_last_error == 0 && error == 0)
	{
		m_integral_error *= m_decay;
	}
	else
	{
		m_integral_error = constrain(m_integral_error + m_Ki * error * dt, m_intMin, m_intMax);
	}

	float derivative_error = (error - m_last_error) / dt;
	m_last_error = error;

	return constrain(m_Kp * error + m_integral_error + m_Kd * derivative_error, m_outMin, m_outMax);
}

int YAPIDController::compute(const int error, const float derivative_error)
{
	unsigned long now = millis();
	float dt = (now - m_last_time) / 1000.0;
	m_last_time = now;
	
	if(m_last_error == 0 && error == 0)
	{
		m_integral_error *= m_decay;
	}
	else
	{
		m_integral_error = constrain(m_integral_error + m_Ki * error * dt, m_intMin, m_intMax);
	}

	return constrain(m_Kp * error + m_integral_error + m_Kd * derivative_error, m_outMin, m_outMax);
}

float YAPIDController::compute(const float error)
{
	unsigned long now = millis();
	float dt = (now - m_last_time) / 1000.0;
	m_last_time = now;
	
	if(m_last_error == 0 && error == 0)
	{
		m_integral_error *= m_decay;
	}
	else
	{
		m_integral_error = constrain(m_integral_error + m_Ki * error * dt, (float) m_intMin, (float) m_intMax);
	}

	float derivative_error = (error - m_last_error) / dt;
	m_last_error = error;

	return constrain(m_Kp * error + m_integral_error + m_Kd * derivative_error, (float) m_outMin, (float) m_outMax);
}

float YAPIDController::compute(const float error, const float derivative_error)
{
	unsigned long now = millis();
	float dt = (now - m_last_time) / 1000.0;
	m_last_time = now;
	
	if(m_last_error == 0 && error == 0)
	{
		m_integral_error *= m_decay;
	}
	else
	{
		m_integral_error = constrain(m_integral_error + m_Ki * error * dt, (float) m_intMin, (float) m_intMax);
	}

	return constrain(m_Kp * error + m_integral_error + m_Kd * derivative_error, (float) m_outMin, (float) m_outMax);
}

int YAPIDAutotuner::runtime(int input, int output)
{
	if(m_sample >= m_sampleCount)
	{
		return 0;
	}
	
	if(((m_last_input < 0) && (0 <= input)) || ((input <= 0) && (0 < m_last_input)))
	{
		unsigned long now = millis();
		
		switch(m_state)
		{
		case 0:
			m_last_time = now;
			m_peak_max = input;
			m_peak_min = input;
			m_state = 1;
			break;
		case 1:
			m_state = 2;
			break;
		case 2:
			m_Ku = 8.0 * abs(output) / ((m_peak_max - m_peak_min) * 3.141592654);
			m_Tu = (now - m_last_time) / 1000.0;
			m_last_time = now;
			m_peak_max = input;
			m_peak_min = input;
			m_state = 1;
			break;
		}
	}
	else if(m_state == 1 || m_state == 2)
	{
		m_peak_max = max(input, m_peak_max);
		m_peak_min = min(input, m_peak_min);
	}

	m_last_input = input;
	m_sample++;
	return 1;
}

void YAPIDAutotuner::end(float* Kp, float* Ki, float* Kd)
{
	if(m_type == YAPID_P)
	{
		*Kp = 0.5 * m_Ku;
		*Ki = 0.0;
		*Kd = 0.0;
	}
	else if(m_type == YAPID_PI)
	{
		*Kp = 0.4 * m_Ku;
		*Ki = 0.48 * m_Ku / m_Tu;
		*Kd = 0.0;
	}
	else if(m_type == YAPID_PID)
	{
		*Kp = 0.6 * m_Ku;
		*Ki = 1.2 * m_Ku / m_Tu;
		*Kd = 0.075 * m_Ku * m_Tu;
	}
}
