# PIDlibrary
PID library for Arduino boards (work in progress). Started off as a modification of the arduino PID library. Currently rewriting entire library, adding a number of extra functions as well as a few more types of PID controllers. 

The only part of the code that depends on the Arduino libraries is the ```millis()``` function. If you replace this with it's equivalent on the hardware you're using, then the code will work on your hardware. ```Millis()``` is a function that returns the time elapsted in milliseconds since the microcontroller was turned on. It returns the time elapsed as an ```unsigned long```.

## Tasks ##

* add auto, manual function
* check all users defined variables for validity (sanity checks)
* Upload test results on Quadcopter, differential drive robot

## Timing ##

The code takes 112 micro seconds to run with every available function in void loop(). That is ~8928Hz. This was tested on an Arduino Uno R3 using the micros() function. 

```cpp
  timepre = micros();
  myPID.setCoefficients(1,1,1);
  myPID.setTimeDelta(2000);
  myPID.integralSat(-2,2);
  myPID.outputSat(-200,200);
  Serial.println(micros()-timepre);
```

## Timing (Static Polymorphism)

The code now takes 104 micro seconds to run with every available function in void loop(). This is a mere 8 microsecond improvement over the original polymorphic version of the codebase which is allot less than what I thought it would be. I'll be investigating this further to see if there are any implementation issues. The same test as above was run to test this code's speed, shown below.

```cpp
  timepre = micros();
  myPID.setCoefficients(1,1,1);
  myPID.setTimeDelta(2000);
  myPID.integralSat(-2,2);
  myPID.outputSat(-200,200);
  Serial.println(micros()-timepre);
