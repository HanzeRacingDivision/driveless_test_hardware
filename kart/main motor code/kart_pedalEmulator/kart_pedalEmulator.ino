/*
this ESP32 will handle emulating the pedal, to make the kart-specific ESC controllable electronically.
(this basically only requires a DAC (and an ADC if you want to use the existing pedal as well))
This ESP will also do the wheel speed measurements (and communicate them to where they're needed),
To make things easier (hopefully) i'll try to implement a constant velocity control loop on this thing,
 but i can't promise it'll work well.

The kart has a big'ol switch for reverse 'gear'. This switch uses (for some reason) 60V signals.
Furthermore, on the kart's dashboard there is a speed select (key)switch.
Both of these things can be replaced by a series of mosfets/relays/Solid-State-Relays (triac?),
 if i want to. I'm not sure i want to. It seems prudent to have this ESP check whether the reverse switch is on,
 but it's not a true necessity either. We'll see how much time/energy i have to spare
That being said, a little resistor-divider to measure battery voltage seems like a quick and useful feature. IDK.

the brakes on the kart are hydraulic, so i have no intention of using those.
Instead, i'll try to get a little motor-breaking to work.
I don't know if the ESC will let me, but i'm damn well gonna try.
Once motor braking works, the control loop may become a tad more complex... :)

The pedal in the kart appears to be some kind of active component, not just a potentiometer.
It still functions like a pot (providing a 0~5v output voltage based on how far the pedal is pressed),
 but it seems to have some deadzone, and to let the output pin float (instead of pulling it) to 5V.
This ESP does not NEED to mimic this wacky behavior, unless testing suggests otherwise.
The oscilloscope showed some high frequency (and very short) voltage drops on the output signal,
 i'm guessing this is some kind of bad feedback from the ESC, rather than something the pedal does on purpose.
In any case, the voltage (and current) required to drive the pedal input on the ESC are more than the ESP's DAC can handle,
 so i'm using a Rail-To-Rail opAmp to multiply the voltage by a factor of 5/3.3=1.51515. (google 'voltage follower' or 'buffer amplifier')
 The opAmp math works out to: Vout = Vin * (1 + (R1 / R2))
 so i'll be using 34.8k and 68k resistors and an MCP6004 RTR opAmp to get this done.
The pedal is at 5V when idle, and 0+? volt when pressed

as for the wheel speed sensors...
*/
#define abs(x) ((x)>0?(x):-(x)) //makes abs() work with floats
#include "thijsFIFO.h"   //a (temporary???) little class library for First In First Out arrays (should be semi-efficient)

//#define realPedalPassthrough   //allows pedal to detemine motor power (again). Use realPedalAsTargetSpeed if you want to pedal to determine targetSpeed
//#define noInstructStop //stop driving if no instruction has been sent for a while    TBD!!!

#include <driver/dac.h>
// usefull macros: DAC_CHANNEL_1   DAC_GPIO25_CHANNEL   DAC_CHANNEL_1_GPIO_NUM
const dac_channel_t DACchannel = DAC_GPIO25_CHANNEL;  //ch1 is GPIO25, ch2 is GPIO26
#define    DACinvert  //invert DAC output, to make 0V == 255 and vice versa.  Useful if ESC is active LOW

const uint8_t DACmotorIdle = 0;
const uint8_t DACmotorMax = 200; //limit motor power here (lower value == higher power)

const uint32_t motorControlInterval = 100; //time between control system updates in millis. The shorter this is, the more aggresive motorKpAdj will be used
uint32_t motorControlTimer;

// output = (targetSpeed * motorKpBase) + ((targetSpeed - mesSpeed) * motorKpAdj) + some other things
const float motorKpBase = 1.0; //motorKpBase will be based on rudimentary calibration (hopefully linear)
//const float motorKpAdj = 30.0; //motorKpAdj should be small to reduce overshoot, but big enough to prevent underperforming steady-states (at very low/high speeds)
//const uint8_t motorKpAdjAvgDepth = 4; //number of points to consider when calculating average speed (MUST BE <= speedLogFIFOsize)
//const uint8_t motorKpSteerAvgDepth = 4; //number of points to consider when calculating average steering angle (MUST BE <= servoLogFIFOsize)
//const float motorKpSteerPolynomial[3] = {0.2877, 0.0387, 0.0156}; //format: {c, b, a}, asin ax^2 + bx + c
//const float motorKpVoltSlope = ; //adjustment based on battery voltage, lower voltage -> higher motor power (compensating)
//const float motorKpVoltOffset = ; //like y=ax+b, where y=KpVolt, a==slope and b==offset

float targetSpeed; //target speed in m/s
const float targetSpeedLimits[2] = {-0.01, 1.5}; //(target) speed limits in m/s
const float minTargetSpeed = 0.05; //minimum target speed for the motor to function (in m/s)

//FIFO<float> speedHistFIFO(10); //a First-In-First-Out array to store (unfiltered) speed history

//////////////////////////////// speed sensors: ////////////////////////////////
//volatile uint32_t encoCount = 0;
//uint32_t encoCountLast;
//uint32_t encoTimer; //stores timestamp associated with encoCountLast

#ifdef realPedalPassthrough
  #warning("realPedalPassthrough is UNTESTED!")
  //#define realPedalAsTargetSpeed  //uses the pedal to set targetSpeed (not motor power directly).
  const uint8_t realPedalADCpin = 33; //using an ADC1 (not ADC2) pin means it can function alongside wifi/BLE (when ADC2 is unavailable)
  //FIFO<uint16_t> realPedalFIFO(10); //a First-In-First-Out array to store (unfiltered) realPedalADC history
  const uint16_t realPedalADCthreshold = 3600; //if the (ADC value of the) pedal drops BELOW this threshold, start listening to it.
#endif

#ifdef DACinvert
  #define DACwrite(val)  dac_output_voltage(DACchannel, 255-val);
#else
  #define DACwrite(val)  dac_output_voltage(DACchannel, val);
#endif

//IRAM_ATTR void encoISR() {
//  encoCount += 1;
//}

//float calcSpeed() {
//  float sum = 0;
//  for(uint8_t i=0; i<speedHistFIFO.size; i++) {
//    sum += speedHistFIFO._data[i]; //the order does not matter, so directly interfacing with the (unordered) FIFO data may be faster
//  }
//  return(sum / speedHistFIFO.size);
//}


void setup() {
  Serial.begin(115200);

  //pinMode(DACpin, OUTPUT); //does not seem to be needed
  dac_output_enable(DACchannel);
  DACwrite(DACmotorIdle);
  
  #ifdef realPedalPassthrough
    pinMode(realPedalADCpin, INPUT);
  #endif
}

void loop() {
  
  #ifdef realPedalPassthrough
    uint16_t realPedalADCval = analogRead(realPedalADCpin);
    if(realPedalADCval <= realPedalADCthreshold) {
      ESP_LOGD(,"real pedal is in control");
      #ifdef realPedalAsTargetSpeed
        targetSpeed = (targetSpeedLimits[1] * (4095-realPedalADCval))/4095; //convert the pedal's (inverted) ADC value to a targetSpeed
      #else
        //DACwrite(DACmotorIdle >> 4); //convert 12bit ADC value into 8bit DAC value directly
        dac_output_voltage(DACchannel, DACmotorIdle >> 4); //convert 12bit ADC value into 8bit DAC value directly  (ignoring DACinvert option!)
        motorControlTimer = millis() + motorControlInterval; //stall the motor control loop (because the pedal determines power)
      #endif
    }
  #endif
  
  if(millis() >= motorControlTimer) {
    motorControlTimer = millis() + motorControlInterval;
    uint8_t newDACval = DACmotorIdle; //does not need to initialized to DACmotorIdle per-se
    if(targetSpeed > minTargetSpeed) {
      float output = (targetSpeed * motorKpBase); //base multiplier
      //float avgMesSpeed = avgSpeed(motorKpAdjAvgDepth); //get average speed (default is full depth, otherwise: avgSpeed(depth))
      //output += ((targetSpeed - avgMesSpeed) * motorKpAdj); //speed error multiplier    TBD!!!
      //output += motorKpSteer(motorKpSteerAvgDepth) * targetSpeed; //steering adds resistance, this counteracts that    TBD!!!
      //output += motorKpVolt() * targetSpeed; //lower battery voltage -> less power, this counteracts that     TBD!!!
      newDACval = constrain(output, DACmotorIdle, DACmotorMax);
    } else if(targetSpeed < -0.01) {
      //attempt motor braking here maybe?
      newDACval = DACmotorIdle;  //driving in reverse is TBD!!!
    } else {
      //attempt motor braking here???
      newDACval = DACmotorIdle;
    }
    DACwrite(newDACval);
  }
}
