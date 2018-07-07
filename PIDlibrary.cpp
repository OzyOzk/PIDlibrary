#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PIDlibrary.h"

basePID::basePID(){}

void basePID::setCoefficients(float p, float i, float d)
{
  this->Kp=p;
  this->Ki=i*float(time_delta);
  this->Kd=d/float(time_delta);
}

void basePID::setTimeDelta(unsigned long time_delta)
{
  Ki = Ki * float(time_delta / this->time_delta);
  Kd = Kd * float(time_delta / this->time_delta);
  this->time_delta = time_delta;
}

void basePID::integralSat(float ilimitlow, float ilitmithigh)
{
  this->ilimithigh = ilimithigh;
  this->ilimitlow = ilimitlow;
}

void basePID::outputSat(float outputlimitlow, float outputlimithigh)
{
  this->outputlimithigh = outputlimithigh;
  this->outputlimitlow = outputlimitlow;
}

void basePID::output(){};

float basePID:: getPmode() const {return Pmode;}
float basePID:: getImode() const {return Imode;}
float basePID:: getDmode() const {return Dmode;}
float basePID:: getKp() const {return Kp;}
float basePID:: getKi() const {return Ki;}
float basePID:: getKd() const {return Kd;}
unsigned long  basePID::getTimeDelta() {return time_delta;}

//proportional, integral and derivative all on error.
PID::PID(float kp, float ki, float kd, unsigned long int time_delta,
    float* reference, float* feedback, float* controlout)
    {
      this->time_delta = time_delta;
      this->reference = reference;
      this->feedback = feedback;
      this->controlout = controlout;
      
      setCoefficients(kp,ki,kd);
      setTimeDelta(time_delta);
      integralSat(1000,2000);
      outputSat(1000,1000);
      error_prev = 0;
      time_prev = millis()-time_delta;
    }

void PID::output()
{
  if(millis()-time_prev >= time_delta)
  {
    time_prev = millis();
    error = *reference-*feedback;

    Pmode = Kp*error;
    Imode += Ki*error;
    Dmode = Kd*(error-error_prev);
    control = Pmode+Imode+Dmode;
    
    if(Imode > ilimithigh) Imode = ilimithigh;
    if(Imode < ilimitlow) Imode = ilimitlow;
    
    if(control > outputlimithigh) *controlout = outputlimithigh;
    else if( control < outputlimitlow) *controlout = outputlimitlow;
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
      
      setCoefficients(kp,ki,kd);
      setTimeDelta(time_delta);
      integralSat(1000,2000);
      outputSat(1000,1000);
      feedback_prev = 0;
      time_prev = millis()-time_delta;
    }

void PI_D::output()
{
  if(millis()-time_prev >= time_delta)
  {
    time_prev = millis();
    error = *reference-*feedback;

    Pmode = Kp*error;
    Imode += Ki*error;
    Dmode = Kd*(*feedback-feedback_prev);
    control = Pmode+Imode+Dmode;
    
    if(Imode > ilimithigh) Imode = ilimithigh;
    if(Imode < ilimitlow) Imode = ilimitlow;
    
    if(control > outputlimithigh) *controlout = outputlimithigh;
    else if( control < outputlimitlow) *controlout = outputlimitlow;
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
    Dmode = Kd * (*feedback - feedback_prev);
    control = Pmode + Imode + Dmode;

    if (Imode > ilimithigh) Imode = ilimithigh;
    if (Imode < ilimitlow) Imode = ilimitlow;

    if (control > outputlimithigh) *controlout = outputlimithigh;
    else if (control < outputlimitlow) *controlout = outputlimitlow;
    else *controlout = control;

    feedback_prev = *feedback;
  }
}
