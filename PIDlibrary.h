#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#ifndef PIDlibrary_h
#define PIDlibrary_h

template<class T>
class basePID
{
public:

    void setCoefficients(float, float, float);

    void setTimeDelta(unsigned long);

    void integralSat(float, float);

    void outputSat(float, float);

    void output();

    float getPmode() const;
    float getImode() const;
    float getDmode() const;
    float getKp() const;
    float getKi() const;
    float getKd() const;

    unsigned long getTimeDelta() const;

protected:

    float Kp;
    float Ki;
    float Kd;
    float Pmode;
    float Imode;
    float Dmode;

    float error;
    float error_dt;
    float error_prev;
    float feedback_prev;
    float integral;

    float control;
    float* reference;
    float* feedback;
    float* controlout;

    unsigned long int time_prev;
    unsigned long int time_delta;

    float i_limit_high;
    float i_limit_low;
    float output_limit_high;
    float output_limit_low;

    basePID();
};

class PID : public basePID<PID>
{
public:

    PID(float, float, float, unsigned long int,
        float*, float*, float*);

    void output();
};

class PI_D : public basePID<PI_D>
{
public:

    PI_D(float, float, float, unsigned long int,
        float*, float*, float*);

    void output();
};

class I_PD : public basePID<I_PD>
{
public:

    I_PD(float, float, float, unsigned long int,
        float*, float*, float*);

    void output();
};

template<class T>
basePID<T>::basePID() {}

template<class T>
void basePID<T>::setCoefficients(float p, float i, float d)
{
    this->Kp = p;
    this->Ki = i * float(time_delta);
    this->Kd = d / float(time_delta);
}

template<class T>
void basePID<T>::setTimeDelta(unsigned long time_delta)
{
    Ki = Ki * float(time_delta / this->time_delta);
    Kd = Kd * float(time_delta / this->time_delta);
    this->time_delta = time_delta;
}

template<class T>
void basePID<T>::integralSat(float i_limit_low, float i_limit_high)
{
    this->i_limit_low = i_limit_low;
    this->i_limit_high = i_limit_high;
}

template<class T>
void basePID<T>::outputSat(float output_limit_low, float output_limit_high)
{
    this->output_limit_high = output_limit_high;
    this->output_limit_low = output_limit_low;
}

template<class T>
void basePID<T>::output()
{
    static_cast<T*>(this)->output();
};

template<class T>
float basePID<T>::getPmode() const { return Pmode; }

template<class T>
float basePID<T>::getImode() const { return Imode; }

template<class T>
float basePID<T>::getDmode() const { return Dmode; }

template<class T>
float basePID<T>::getKp() const { return Kp; }

template<class T>
float basePID<T>::getKi() const { return Ki; }

template<class T>
float basePID<T>::getKd() const { return Kd; }

template<class T>
unsigned long  basePID<T>::getTimeDelta() const { return time_delta; }

//proportional, integral and derivative all on error.
PID::PID(float kp, float ki, float kd, unsigned long int time_delta,
    float* reference, float* feedback, float* controlout)
{
    this->time_delta = time_delta;
    this->reference = reference;
    this->feedback = feedback;
    this->controlout = controlout;

    setCoefficients(kp, ki, kd);
    setTimeDelta(time_delta);
    integralSat(1000, 2000);
    outputSat(1000, 1000);
    error_prev = 0;
    time_prev = millis() - time_delta;
}

void PID::output()
{
    if (millis() - time_prev >= time_delta)
    {
        time_prev = millis();
        error = *reference - *feedback;

        Pmode = Kp * error;
        Imode += Ki * error;

        if (Imode > i_limit_high) Imode = i_limit_high;
        if (Imode < i_limit_low) Imode = i_limit_low;

        Dmode = Kd * (error - error_prev);
        control = Pmode + Imode + Dmode;

        if (control > output_limit_high) *controlout = output_limit_high;
        else if (control < output_limit_low) *controlout = output_limit_low;
        else *controlout = control;

        error_prev = error;

    }
}

//proportional integral on error, derivative on feedback
PI_D::PI_D(float kp, float ki, float kd, unsigned long int time_delta,
    float* reference, float* feedback, float* controlout)
{
    this->time_delta = time_delta;
    this->reference = reference;
    this->feedback = feedback;
    this->controlout = controlout;

    setCoefficients(kp, ki, kd);
    setTimeDelta(time_delta);
    integralSat(1000, 2000);
    outputSat(1000, 1000);
    feedback_prev = 0;
    time_prev = millis() - time_delta;
}

void PI_D::output()
{
    if (millis() - time_prev >= time_delta)
    {
        time_prev = millis();
        error = *reference - *feedback;

        Pmode = Kp * error;

        Imode += Ki * error;
        if (Imode > i_limit_high) Imode = i_limit_high;
        if (Imode < i_limit_low) Imode = i_limit_low;

        Dmode = Kd * (*feedback - feedback_prev);
        control = Pmode + Imode + Dmode;

        if (control > output_limit_high) *controlout = output_limit_high;
        else if (control < output_limit_low) *controlout = output_limit_low;
        else *controlout = control;

        feedback_prev = *feedback;
    }
}

//proportional and derivative on feedback, Integral on error
I_PD::I_PD(float kp, float ki, float kd, unsigned long int time_delta,
    float* reference, float* feedback, float* controlout)
{
    this->time_delta = time_delta;
    this->reference = reference;
    this->feedback = feedback;
    this->controlout = controlout;

    setCoefficients(kp, ki, kd);
    setTimeDelta(time_delta);
    integralSat(1000, 2000);
    outputSat(1000, 1000);
    feedback_prev = 0;
    time_prev = millis() - time_delta;
}

void I_PD::output()
{
    if (millis() - time_prev >= time_delta)
    {
        time_prev = millis();
        error = *reference - *feedback;
        Pmode = Kp * (*feedback - feedback_prev);

        Imode += Ki * error;
        if (Imode > i_limit_high) Imode = i_limit_high;
        if (Imode < i_limit_low) Imode = i_limit_low;

        Dmode = Kd * (*feedback - feedback_prev);
        control = Pmode + Imode + Dmode;

        if (control > output_limit_high) *controlout = output_limit_high;
        else if (control < output_limit_low) *controlout = output_limit_low;
        else *controlout = control;

        feedback_prev = *feedback;
    }
}

#endif
