#ifndef PIDlibrary_h
#define PIDlibrary_h

class basePID
{
  public:

    virtual void setCoefficients(float, float, float);
    
    virtual void setTimeDelta(unsigned long);

    virtual void integralSat(float, float);

    virtual void outputSat(float, float);
    
    virtual void output()=0;

    virtual float getPmode () const;
    virtual float getImode () const;
    virtual float getDmode () const;
    virtual float getKp () const;
    virtual float getKi () const;
    virtual float getKd () const;
    
    virtual unsigned long getTimeDelta() const;
  
  protected:
  
    float Kp;
    float Ki;
    float Kd;
    float Pmode;
    float Imode;
    float Dmode;
    
    float error;
    float errorDt;
    float error_prev;
    float feedback_prev;
    float integral;

    float control;
    float* reference;
    float* feedback;
    float* controlout;
    
    unsigned long int time_prev;
    unsigned long int time_delta;

    float ilimithigh;
    float ilimitlow;
    float outputlimithigh;
    float outputlimitlow;
    
    basePID();
};

class PID : public basePID
{
  public:

    PID(float, float, float, unsigned long int,
        float*, float*, float*);
    
    void output();
};

class PI_D : public basePID
{
  public:

    PI_D(float, float, float, unsigned long int,
        float*, float*, float*);
    
    void output();
};

class I_PD : public basePID
{
  public:

    I_PD(float, float, float, unsigned long int,
	    float*, float*, float*);

    void output();
};

#endif
