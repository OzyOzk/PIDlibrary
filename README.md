# PIDlibrary
PID libary for Arduino boards (work in progress). Started off as a modification of the arduino PID library. Currently rewriting entire library, adding a number of extra functions as well as a few more types of PID controllers.

## Tasks ##

* add auto, manual function
* check all users defined variables for validity (sanity checks)
* Upload test results on Quadcopter, differential drive

## Timing ##

The code takes 112 micro seconds to run with every available function in void loop(). That is ~8928Hz. This was tested on an Arduino Uno R3 using the micros() function. 

```c++
  timepre = micros();
  myPID.setCoefficients(1,1,1);
  myPID.setTimeDelta(2000);
  myPID.integralSat(-2,2);
  myPID.outputSat(-200,200);
  Serial.println(micros()-timepre);
```
