/*
  TeemonLib.cpp - heavily modified teensy-specific variant of emonlib:
  (previouly EmonLib.cpp] - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
*/

//#include "WProgram.h" un-comment for use on older versions of Arduino IDE
#include "TeemonLib.h"

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

// to-do, turn NOISEFLOOR into a per-channel configurable
#define NOISEFLOOR 3.4

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(int _inPinV, double _VCAL, double _PHASECAL)
{
   inPinV = _inPinV;
   VCAL = _VCAL;
   PHASECAL = _PHASECAL;
}

void EnergyMonitor::current(int _inPinI, double _ICAL)
{
   inPinI = _inPinI;
   ICAL = _ICAL;
}

// void/placeholder... old "calcVI" did a lot more than calcuale VI! Keep the method as a wrapper or discard?
void EnergyMonitor::calcVI(int crossings, int timeout)
{

}

void EnergyMonitor::waitForCross(int timeout) 
{
  boolean st=false;                                  //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(st==false)                                   //the while loop...
  {
     startV = analogRead(inPinV);                    //using the voltage waveform
     if ((startV < (ADC_COUNTS/2+50)) && (startV > (ADC_COUNTS/2-50))) st=true;  //check its within range
     if ((millis()-start)>timeout) st = true;
  } 
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kwh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
#define CALC_STATE_PENDING 0
#define CALC_STATE_COMPLETE  1
#define CALC_STATE_TIMEDOUT -1
int EnergyMonitor::calcAllNonblock(int crossings, int timeout, int init)
{

	int SUPPLYVOLTAGE = readVcc();

  int numberOfSamples = 0;                        //This is now incremented  

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (500 adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  
  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurment loop
  //------------------------------------------------------------------------------------------------------------------------- 

  if (init) {
    lastStartMillis = millis(); 
    crossCount = 0;
  }
  if ((crossCount < crossings) && ((millis()-lastStartMillis)<timeout)) 
  {
    numberOfSamples++;                            //Count number of times looped.

    lastSampleV=sampleV;                          //Used for digital high pass filter
    lastSampleI=sampleI;                          //Used for digital high pass filter
    
    lastFilteredV = filteredV;                    //Used for offset removal
    lastFilteredI = filteredI;                    //Used for offset removal   
    
    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = analogRead(inPinV);                 //Read in raw voltage signal
    sampleI = analogRead(inPinI);                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
    //-----------------------------------------------------------------------------
    filteredV = 0.996*(lastFilteredV+(sampleV-lastSampleV));
    filteredI = 0.996*(lastFilteredI+(sampleI-lastSampleI));
   
    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum
    
    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------   
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum 
    
    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV); 
    
    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------   
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP +=instP;                               //Sum  
    
    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------       
    lastVCross = checkVCross;                     
    if (sampleV > startV) checkVCross = true; 
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;                  
                     
    if (lastVCross != checkVCross) crossCount++;
    
    return CALC_STATE_PENDING;
  } 
  else
  {
    // either we hit the calc timeout or the desired number of crossings. Time to update the
    // metrics
  
    double V_RATIO = VCAL *((SUPPLYVOLTAGE/1000.0) / (ADC_COUNTS));
    Vrms = V_RATIO * sqrt(sumV / numberOfSamples); 
    
    double I_RATIO = ICAL *((SUPPLYVOLTAGE/1000.0) / (ADC_COUNTS));
    Irms = I_RATIO * sqrt(sumI / numberOfSamples); 

    //Calculation power values
    realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
    apparentPower = Vrms * Irms;
    powerFactor=realPower / apparentPower;
    
    
    // handle the wattseconds counter 
    uint32_t incr, carry;
    if (filterReadyIn > 0) {
      // first few times calcVI is called, the offset filter is still not settled and the
      // counter deltas are worthless. tune the initial value of filterReadyIn to prevent giant inital counter values.
      filterReadyIn--;
      wsLastCalcTime = 0;
      ws = 0;
    } else {
      incr = wsLastCalcTime / 1000;
      if (incr) {
        carry = wsLastCalcTime % 1000;
        
        if (realPower < 20) {
          // no decrementing supported (for now?). Additionally, anything below 60% of the noise "floor" is ignored.
          if (realPower > NOISEFLOOR*0.6) {
            ws += incr*(map((long)(realPower*1000), (NOISEFLOOR*0.6)*1000, 20000, 0, 20000)/1000);
          }
        } else {
          ws += (uint64_t) ((double)incr*realPower);
        }
        wsLastCalcTime = carry;
      }
    }
    //Reset accumulators
    sumV = 0;
    sumI = 0;
    sumP = 0;
    
    return (crossCount >= crossings) ? CALC_STATE_COMPLETE : CALC_STATE_TIMEDOUT;
    
  }
 

}


void EnergyMonitor::serialprint()
{
    Serial.print(realPower);
    Serial.print(' ');
    Serial.print(apparentPower);
    Serial.print(' ');
    Serial.print(Vrms);
    Serial.print(' ');
    Serial.print(Irms);
    Serial.print(' ');
    Serial.print(powerFactor);
    Serial.println(' ');
    delay(100); 
}

//thanks to http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino
//and Jérôme who alerted us to http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

long EnergyMonitor::readVcc() {
  long result;

//check for Teensy3.1 (note, this may work for teensy3.0 as well... if so, check CORE_TEENSY to match teensy 3.X instead)
#if defined(__MK20DX256__)
  // analogReadAveraging() with 8 or higher should have been called in the main sketch setup for this to prove most reliable
  result = 1195 * ADC_COUNTS /analogRead(39);
  delay(1);
  result = (result + (1195 * ADC_COUNTS /analogRead(39)))/2;
  return result;
#endif

  return 3300;
}

