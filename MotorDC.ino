/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/*
 * CODE Control Driver DC
 Author: Minh Tri Knight.
 */

#include <Hercules.h>
#include <math.h>
#include "DifferentialSteering.h"
int fPivYLimit = 150;
DifferentialSteering DiffSteer(fPivYLimit);


/*-----------------------------------------------
 * Code receiver RC
 --------------------------------------------------*/
const int pwmPIN[]={2,3}; // an array to identify the PWM input pins (the array can be any length) 
                                  // first pin is channel 1, second is channel 2...etc

int RC_inputs = 0;                // The number of pins in pwmPIN that are connected to an RC receiver. Addition pins not connected to an RC receiver could be used for any other purpose i.e. detecting the echo pulse on an HC-SR04 ultrasonic distance sensor
                                  // When 0, it will automatically update to the number of pins specified in pwmPIN[] after calling setup_pwmRead().                                                
// Calibration of each RC channel:
 
// The arrays below are used to calibrate each RC channel into the range -1 to +1 so that servo direction, mixing, rates, sub trims...etc can be applied in sketch depending on application. 
// The arrays should be modified in order to calibrate the min, middle and max pulse durations to suit your transmitter (use max rates to get the best resolution). 
// FYI: the function print_PWM() will print the raw pulse width data for all the RC channels to the serial port.
// if the RC_min[], RC_mid[], RC_max[] are empty or have missing data the calibration will default to min 1000us, mid 1500us and max 2000us.

//SANWA 6CH 40MHz with corona RP6D1  
//                THR     RUD    
int RC_min[6] = { 1000,   1000};
int RC_mid[6] = { 1500,   1500};
int RC_max[6] = { 2000,   2000};
//          MAX    MAX

// fail safe positions

float RC_failsafe[] = {0.00, 0.00};
   
// enter a failsafe position (in the range of -+1) for each RC channel in case radio signal is lost
// if the array is the incorrect length for the number of RC channels, the failsafe will default to neutral i.e. 0. 
// The failsafe tolerances are: 10-330Hz & 500-2500us

/*
 *    GLOBAL PWM DECODE VARIABLES
 */

const int num_ch = sizeof(pwmPIN)/sizeof(int);  // calculate the number of input pins (or channels)
volatile int PW[num_ch];                        // an array to store pulsewidth measurements
volatile int PWM_in[num_ch];
volatile boolean prev_pinState[num_ch];         // an array used to determine whether a pin has gone low-high or high-low
volatile unsigned long pciTime;                 // the time of the current pin change interrupt
volatile unsigned long pwmTimer[num_ch];        // an array to store the start time of each PWM pulse

volatile boolean pwmFlag[num_ch];               // flag whenever new data is available on each pin
volatile boolean RC_data_rdy;                   // flag when all RC receiver channels have received a new pulse
unsigned long pwmPeriod[num_ch];                 // period, mirco sec, between two pulses on each pin

byte pwmPIN_reg[num_ch];                        // each of the input pins expressed as a position on it's associated port register
byte pwmPIN_port[num_ch];                       // identify which port each input pin belongs to (0 = PORTB, 1 = PORTC, 2 = PORTD)

const int size_RC_min = sizeof(RC_min) / sizeof(int);           // measure the size of the calibration and failsafe arrays
const int size_RC_mid = sizeof(RC_mid) / sizeof(int);
const int size_RC_max = sizeof(RC_max) / sizeof(int);
const int size_RC_failsafe = sizeof(RC_failsafe) / sizeof(float);

// FUNCTION USED TO TURN ON THE INTERRUPTS ON THE RELEVANT PINS
// code from http://playground.arduino.cc/Main/PinChangeInterrupt

void pciSetup(byte pin){
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

// FUNCTION USED TO FIND THE PIN POSITION ON EACH PORT REGISTER: helps the interrupt service routines, ISR, run faster

void pwmPIN_to_port(){
  for (int i = 0; i < num_ch; i++){

    // determine which port and therefore ISR (PCINT0_vect, PCINT1_vect or PCINT2_vect) each pwmPIN belongs to.
                                                                  pwmPIN_port[i] = 1;    // pin belongs to PCINT1_vect (PORT C)
    if (pwmPIN[i] >= 0 && pwmPIN[i] <= 7)                         pwmPIN_port[i] = 2;    // pin belongs to PCINT2_vect (PORT D)
    else if (pwmPIN[i] >= 8 && pwmPIN[i] <= 13)                   pwmPIN_port[i] = 0;    // pin belongs to PCINT0_vect (PORT B)

    // covert the pin number (i.e. pin 11 or pin A0) to the pin position in the port register. There is most likely a better way of doing this using a macro...
    // (Reading the pin state directly from the port registers speeds up the code in the ISR)
    
    if(pwmPIN[i] == 0 || pwmPIN[i] == A0 || pwmPIN[i] == 8)         pwmPIN_reg[i] = 0b00000001;
    else if(pwmPIN[i] == 1 || pwmPIN[i] == A1 || pwmPIN[i] == 9)    pwmPIN_reg[i] = 0b00000010;
    else if(pwmPIN[i] == 2 || pwmPIN[i] == A2 || pwmPIN[i] == 10)   pwmPIN_reg[i] = 0b00000100;
    else if(pwmPIN[i] == 3 || pwmPIN[i] == A3 || pwmPIN[i] == 11)   pwmPIN_reg[i] = 0b00001000;
    else if(pwmPIN[i] == 4 || pwmPIN[i] == A4 || pwmPIN[i] == 12)   pwmPIN_reg[i] = 0b00010000;
    else if(pwmPIN[i] == 5 || pwmPIN[i] == A5 || pwmPIN[i] == 13)   pwmPIN_reg[i] = 0b00100000;
    else if(pwmPIN[i] == 6)                                         pwmPIN_reg[i] = 0b01000000;
    else if(pwmPIN[i] == 7)                                         pwmPIN_reg[i] = 0b10000000;
    
  }
}

// SETUP OF PIN CHANGE INTERRUPTS

void setup_pwmRead(){
  
  for(int i = 0; i < num_ch; i++){              // run through each input pin
    pciSetup(pwmPIN[i]);                        // enable pinchange interrupt for pin
  }
  pwmPIN_to_port();                             // determines the port for each input pin
                                                // pwmPIN_to_port() also coverts the pin number in pwmPIN[] (i.e. pin 11 or pin A0) to the pin position in the port register (i.e. 0b00000001) for use in the ISR.
  
  if(RC_inputs == 0 || RC_inputs > num_ch) RC_inputs = num_ch;    // define the number of pins connected to an RC receiver.                                          
} 

// INTERRUPT SERVICE ROUTINES (ISR) USED TO READ PWM INPUT

// the PCINT0_vect (B port register) reacts to any changes on pins D8-13.
// the PCINT1_vect (C port register)          ""        ""         A0-A5.
// the PCINT2_vect (D port register)          ""        ""         D0-7.

// port registers are used to speed up if statements in ISR code:



// READ INTERRUPTS ON PINS D8-D13: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT0_vect){                                                 // this function will run if a pin change is detected on portB
  
  pciTime = micros();                                             // Record the time of the PIN change in microseconds

  for (int i = 0; i < num_ch; i++){                               // run through each of the channels
    if (pwmPIN_port[i] == 0){                                     // if the current channel belongs to portB
      
      if(prev_pinState[i] == 0 && PINB & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PINB & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                        // flag that new data is available
        if(i+1 == RC_inputs) RC_data_rdy = HIGH;                  
      }
    }
  }
}

// READ INTERRUPTS ON PINS A0-A5: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT1_vect){                                                 // this function will run if a pin change is detected on portC

  pciTime = micros();                                             // Record the time of the PIN change in microseconds

  for (int i = 0; i < num_ch; i++){                               // run through each of the channels
    if (pwmPIN_port[i] == 1){                                     // if the current channel belongs to portC
      
      if(prev_pinState[i] == 0 && PINC & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PINC & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                             // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                         // flag that new data is available
        if(i+1 == RC_inputs) RC_data_rdy = HIGH;
      }
    }
  }
}

// READ INTERRUPTS ON PINS D0-7: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT2_vect){                                                 // this function will run if a pin change is detected on portD

  pciTime = micros();                                             // Record the time of the PIN change in microseconds

  for (int i = 0; i < num_ch; i++){                               // run through each of the channels
    if (pwmPIN_port[i] == 2){                                     // if the current channel belongs to portD
      
      if(prev_pinState[i] == 0 && PIND & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PIND & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                        // flag that new data is available
        if(i+1 == RC_inputs) RC_data_rdy = HIGH;
      }
    }
  }
}
float calibrate(float Rx, int Min, int Mid, int Max){
   float calibrated;
   if (Rx >= Mid)
   {
    calibrated = map(Rx, Mid, Max, 0, 1000);  // map from 0% to 100% in one direction
   }
   else if (Rx == 0)
   {
    calibrated = 0;                           // neutral
   }
   else
   {
    calibrated = map(Rx, Min, Mid, -1000, 0); // map from 0% to -100% in the other direction
   }
  return calibrated * 0.001;
}

// Basic Receiver FAIL SAFE
// check for 500-2500us and 10-330Hz (same limits as pololu)

boolean FAILSAFE(int CH){

   int i = CH-1;
   boolean failsafe_flag = LOW;
        
       if(pwmFlag[i] == 1)                             // if a new pulse has been measured.
         {
            pwmFlag[i] = 0;                            // set flag to zero
      
            if(pwmPeriod[i] > 100000)                  // if time between pulses indicates a pulse rate of less than 10Hz   
            {
              failsafe_flag = HIGH;                       
            }
            else if(pwmPeriod[i] < 3000)               // or if time between pulses indicates a pulse rate greater than 330Hz   
            {
              failsafe_flag = HIGH;                             
            }

            if(PW[i] < 500 || PW[i] > 2500)           // if pulswidth is outside of the range 500-2500ms
            {
              failsafe_flag = HIGH;                        
            }   
         }
        else if (micros() - pwmTimer[i] > 100000)     // if there is no new pulswidth measurement within 100ms (10hz)
        {
          failsafe_flag = HIGH;                      
        }

    return failsafe_flag;   
}

/*
 *  RC OUTPUT FUNCTIONS
 */

 boolean RC_avail(){
    boolean avail = RC_data_rdy;
    RC_data_rdy = LOW;                          // reset the flag
    return avail;
    }

  float RC_decode(int CH){
  
  if(CH < 1 || CH > RC_inputs) return 0;     // if channel number is out of bounds return zero.
  
  int i = CH - 1;                     

  // determine the pulse width calibration for the RC channel. The default is 1000, 1500 and 2000us.
  
  int Min;
  if(CH <= size_RC_min) Min = RC_min[CH-1]; else Min = 1000;
  
  int Mid;
  if(CH <= size_RC_mid) Mid = RC_mid[CH-1]; else Mid = 1500;
  
  int Max;
  if(CH <= size_RC_max) Max = RC_max[CH-1]; else Max = 2000;

  float CH_output;
      
  if(FAILSAFE(CH) == HIGH){                         // If the RC channel is outside of failsafe tolerances (10-330hz and 500-2500uS)
      if(CH > size_RC_failsafe) CH_output = 0;      // and if no failsafe position has been defined, set output to neutral
      else CH_output = RC_failsafe[i];              // or if defined set the failsafe position 
  }
  else{                                             // If the RC signal is valid
    CH_output = calibrate(PW[i],Min,Mid,Max);       // calibrate the pulse width to the range -1 to 1.
  }
  return CH_output;                                 

  // The signal is mapped from a pulsewidth into the range of -1 to +1, using the user defined calibrate() function in this code. 

  // 0 represents neutral or center stick on the transmitter
  // 1 is full displacement of a control input is one direction (i.e full left rudder)
  // -1 is full displacement of the control input in the other direction (i.e. full right rudder)
}

/*
 *  Receiver Calibration
 */

 // NEED TO SPEED UP


/* Interface code using the PWM signal
*/
int PWM_signal(int i){
  i=i-1;
  PWM_in[i]= PW[i];
  return PWM_in[i];
  
}
/*
 *  Quick print function of Rx channel input
 */

void print_RCpwm(){                             // display the raw RC Channel PWM Inputs
  for (int i = 0; i < RC_inputs; i++){
    //Serial.print(" ch");Serial.print(i+1);
    Serial.print("  ");
    if(PW[i] < 1000) Serial.print(" ");
    Serial.print(PW[i]);
  }
  Serial.println("");
}


void print_decimal2percentage(float dec){
  int pc = dec*100;
  // the number and text will take up 6 charactors i.e ___3%_ or -100%_
  if (pc >= 0) Serial.print(" ");
  if (abs(pc) < 100) Serial.print(" ");
  if (abs(pc) < 10) Serial.print(" ");
  Serial.print(" ");Serial.print(pc);Serial.print("% ");
}

/*
 * GENERIC PWM FUNCTIONS
 */

unsigned long pin_time;
float pin_pwm;
float pin_period;

boolean PWM_read(int CH){
  if(CH < 1 && CH > num_ch) return false;
  int i = CH-1;
  boolean avail = pwmFlag[i];
  if (avail == HIGH){
    pwmFlag[i] = LOW;
    noInterrupts();
    pin_time = pwmTimer[i];
    pin_pwm = PW[i];
    pin_period = pwmPeriod[i];
    interrupts();
  }
  return avail;
}

unsigned long PWM_time(){return pin_time;}
float PWM_period(){return pin_period;}
float PWM(){return pin_pwm;}

float PWM_freq(){
  float freq;
  return freq = 1000000 / pin_period;  // frequency Hz
}

float PWM_duty(){
  float duty;
  duty = pin_pwm/pin_period;
  return duty;
}

/*-----------------------------------------------
 * END Code receiver RC
 --------------------------------------------------*/
///
//#include <Servo.h>
//Beginning of Auto generated function prototypes by Atmel Studio
void pciSetup(byte pin);
void pwmPIN_to_port();
void setup_pwmRead();
ISR(PCINT0_vect );
ISR(PCINT1_vect );
ISR(PCINT2_vect );
float calibrate(float Rx, int Min, int Mid, int Max);
boolean FAILSAFE(int CH);
boolean RC_avail();
float RC_decode(int CH);
int PWM_signal(int i);
void print_RCpwm();
void print_decimal2percentage(float dec);
boolean PWM_read(int CH);
unsigned long PWM_time();
float PWM_period();
float PWM();
float PWM_freq();
float PWM_duty();
void RC_input();
void Battle_input();
void Debug_signal();
//End of Auto generated function prototypes by Atmel Studio


//int servoPin=9;
//int servoPin2=10;
//Servo Servo1;
//Servo Servo2;

int PWM_in1;
int PWM_in2;
int PWM_out1;
int PWM_out2;

int Battle_mode = A1;
int Output_0V = A0;
int Battle=0;

unsigned long now;                        // timing variables to update data at a regular interval                  
unsigned long rc_update;
const int channels = 2;                   // specify the number of receiver channels
float RC_in[channels];                    // an array to store the calibrated input from receiver 
boolean motor_control= true;

void setup()
{
    MOTOR.begin();                      // initialize
    setup_pwmRead();                      
    Serial.begin(9600);
    pinMode(Battle_mode,INPUT);
    pinMode(Output_0V,OUTPUT);
  MOTOR.setStop1();
  MOTOR.setStop2();

}

void loop()
{
  // Trigger Battle Mode or RC Mode
   Battle = digitalRead(Battle_mode);
   digitalWrite(Output_0V, LOW);
    
  
    now = millis();
    if(RC_avail() && now - rc_update > 50){   // if RC data is available AND 25ms has passed since last update (adjust to be equal or greater than the frame rate of receiver)
     
   rc_update = now;  // Update time
     PWM_in1 = PWM_signal(1); // get input PWM signal 1 (pin D2)
     PWM_in2 = PWM_signal(2); // get input PWM signal 1 (pin D3)
//----------------------------
    //Call function RC_input
    if (Battle==0){
        RC_input();
      }
    else if (Battle==1){
       Battle_input();
    
      }
//   Serial.println(PWM_in1);
//  Serial.println(PWM_in2);
    
    }
  
}

// This code for RC input
void RC_input(){
  //////------------------Change Speed of the Motor 1----------
     if (PWM_in1 < 1080){
      PWM_out1 = 100;
      MOTOR.setSpeedDir1(PWM_out1, DIRF);
      Serial.println("PWM output to Motor 1");
      Serial.println(PWM_out1); 
      } 
     else if( (1080 <= PWM_in1) && (1480 > PWM_in1)){
      PWM_out1 = round((1480-PWM_in1)/4);
      MOTOR.setSpeedDir1(PWM_out1, DIRF);
      Serial.println("PWM output to Motor 1");
      Serial.println(PWM_out1);
     }
     else if( (1480<= PWM_in1) &&  (1520> PWM_in1)){
      PWM_out1 = 0;
      MOTOR.setStop1();
      Serial.println("PWM output to Motor 1");
      Serial.println(PWM_out1);
     }
     else if( (1520<= PWM_in1) &&  (1920 > PWM_in1)){
      PWM_out1 = round((PWM_in1-1520)/4);
      MOTOR.setSpeedDir1(PWM_out1, DIRR);
      Serial.println("PWM output to Motor 1");
      Serial.println(PWM_out1);
     }
     else if( 1920<= PWM_in1 ){
      PWM_out1 = 100;
      MOTOR.setSpeedDir1(PWM_out1, DIRR);
      Serial.println("PWM output to Motor 1");
      Serial.println(PWM_out1);
     }
//
////////------------------Change Speed of the Motor 2----------
     if (PWM_in2 < 1080){
      PWM_out2 = 100;
      MOTOR.setSpeedDir2(PWM_out2, DIRF);
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_out2); 
      } 
     else if( (1080 <= PWM_in2) && (1480 > PWM_in2)){
      PWM_out2 = round((1480-PWM_in2)/4);
      MOTOR.setSpeedDir2(PWM_out2, DIRF);
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_out2);
     }
     else if( (1480<= PWM_in2) &&  (1520> PWM_in2)){
      PWM_out2 = 0;
      MOTOR.setStop2();
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_out2);
     }
     else if( (1520<= PWM_in2) &&  (1920 > PWM_in2)){
      PWM_out2 = round((PWM_in2-1520)/4);
      MOTOR.setSpeedDir2(PWM_out2, DIRR);
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_out2);
     }
     else if( 1920<= PWM_in2 ){
      PWM_out2 = 100;
      MOTOR.setSpeedDir2(PWM_out2, DIRR);
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_out2);
     }
  }
//--------------------------------------
// This code for battle mode
void Battle_input(){
  int XValue = map(PWM_in1, 1000, 2000, -1000, 1000);
     int YValue = map(PWM_in2, 1000, 2000, -1000, 1000);
     int lowLimit = 1080;
     int highLimit = 1920;
     DiffSteer.computeMotors(XValue, YValue);
     int leftMotor = DiffSteer.computedLeftMotor();
     int rightMotor = DiffSteer.computedRightMotor();
     int PWM_1 = map(leftMotor, -1000,1000, -100,100);
     int PWM_2 = map(rightMotor, -1000,1000, -100,100);
      
      if ((PWM_1 >=0)  && (PWM_2  >=0)){
////    
       MOTOR.setSpeedDir1(PWM_1, DIRF);
       MOTOR.setSpeedDir2(PWM_2, DIRF);
      Serial.println("PWM output to Motor 1");
      Serial.println(PWM_1); 
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_2); 
     } 
       else if((PWM_1 <0)&& (PWM_2  < 0)){
       PWM_1 = abs(PWM_1);
       PWM_2 = abs(PWM_2);
       MOTOR.setSpeedDir1(PWM_1, DIRR);
       MOTOR.setSpeedDir2(PWM_2, DIRR);
        Serial.println("PWM output to Motor 1");
      Serial.println(PWM_1); 
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_2); 
       }
//     
       else if ((PWM_1 >=0)&& (PWM_2  <0)){
      PWM_1 = abs(PWM_1);
       PWM_2 = abs(PWM_2);
       MOTOR.setSpeedDir1(PWM_1, DIRF);
       MOTOR.setSpeedDir2(PWM_2, DIRR);
        Serial.println("PWM output to Motor 1");
      Serial.println(PWM_1); 
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_2); 
       }
       else if ((PWM_1 <0)&& (PWM_2  >=0)){
       PWM_1 = abs(PWM_1);
       PWM_2 = abs(PWM_2);
       MOTOR.setSpeedDir1(PWM_1, DIRR);
       MOTOR.setSpeedDir2(PWM_2, DIRF);
        Serial.println("PWM output to Motor 1");
      Serial.println(PWM_1); 
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_2); 
  }
   else if ((PWM_1 >130)&& (PWM_2  >130)){ // Safety Function if interupt
       MOTOR.setSpeedDir1(0, DIRR);
       MOTOR.setSpeedDir2(0, DIRF);
        Serial.println("PWM output to Motor 1");
      Serial.println(PWM_1); 
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_2); 
  }
    else if ((PWM_1 < -130)&& (PWM_2  < -130)){ // Safety Function if interupt
       MOTOR.setSpeedDir1(0, DIRR);
       MOTOR.setSpeedDir2(0, DIRF);
        Serial.println("PWM output to Motor 1");
      Serial.println(PWM_1); 
      Serial.println("PWM output to Motor 2");
      Serial.println(PWM_2); 
  }
}
  
void Debug_signal(){
        //--------------DEBUG CODE-----------------
      
//     Serial.println("--------------"); 
//     Serial.println("PWM Signal 1")  ;    // print PWM Signal 1                    
//     Serial.println(PWM_in1);   
//      Serial.println("PWM signal 2");      // print PWM Signal 2             
//      Serial.println(PWM_in2); 
      // This code for debug. Conver the RC signal 1000us-2000us to -> -100% to +100%
//  for (int i = 0; i<channels; i++){       // run through each RC channel
//        int CH = i+1; 
//        RC_in[i] = RC_decode(CH); 
//        print_decimal2percentage(RC_in[i]);   // uncomment to print calibrated receiver input (+-100%) to serial
//  }
      //-------------------------------
  }
