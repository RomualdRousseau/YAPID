/**************************************************************************/
/*!
	@file     PID.h
	@author   Romuald Rousseau
	@license  BSD (see license.txt)

	Driver for the CT8Z transmiter

	This is a library for the PID controller

	@section  HISTORY

	v1.0  - First release
*/
/**************************************************************************/
#ifndef YAPID_h
#define YAPID_h

#define YAPID_P		0
#define	YAPID_PI	1
#define YAPID_PID	2

#define DECAY_NONE		1.0
#define DECAY_QUADRATIC	0.5
#define DECAY_RESET		0.0


class YAPIDController
{
public:
	YAPIDController(const int outMin, const int outMax, const float decay):
		m_outMin(outMin),
		m_outMax(outMax),
		m_decay(decay)
	{
		reset();
	}

	void setConservativeTunings()
	{
		setTunings(1.0, 0.05, 0.25);
	}

	void setAggresiveTunings()
	{
		setTunings(4.0, 0.2, 1.0);
	}

	void setTunings(const float Kp, const float Ki, const float Kd)
	{
		m_Kp = Kp;
		m_Ki = Ki;
		m_Kd = Kd;
	}

	void getTunings(float* Kp, float* Ki, float* Kd)
	{
		*Kp = m_Kp;
		*Ki = m_Ki;
		*Kd = m_Kd;
	}

	void reset()
	{
		m_integral_error = 0;
		m_last_error = 0;
		m_last_time = millis();
	}

	int compute(const int error);

private:
	const int m_outMin;
	const int m_outMax;
	const float m_decay;
	float m_Kp;
	float m_Ki;
	float m_Kd;
	float m_integral_error;
	float m_last_error;
	unsigned long m_last_time;
};

class YAPIDAutotuner
{
public:
	YAPIDAutotuner(const int sampleCount, const int type):
		m_sampleCount(sampleCount),
		m_type(type)
	{
		begin();
	}
	
	void begin()
	{
		m_sample = 0;
		m_last_input = 0;
		m_state = 0;
	}
	
	int runtime(int input, int output);
	
	void end(float* Kp, float* Ki, float* Kd);

private:
	const int m_sampleCount;
	const int m_type;
	unsigned long m_last_time;
	int m_sample;
	int m_last_input;
	int m_state;
	int m_peak_max;
	int m_peak_min;
	float m_Ku;
	float m_Tu;
};

#endif
