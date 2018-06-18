class basePID
{
  public:

    virtual void setCoefficients(float, float, float);
    
    virtual void setTimeDelta(unsigned long);

    virtual void integralSat(float, float);

    virtual void outputSat(float, float);
    
    virtual void output()=0;

    virtual float getPmode();
    virtual float getImode();
    virtual float getDmode();
    virtual float getKp();
    virtual float getKi();
    virtual float getKd();
    
    virtual unsigned long getTimeDelta();
  
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

class PI_D : public basePID
{
public:

	I_PD(float, float, float, unsigned long int,
		float*, float*, float*);

	void output();
};
#endif
