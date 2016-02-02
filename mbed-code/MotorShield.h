#ifndef _MotorShield_h
#define _MotorShield_h

#include "mbed.h"

/////////////
// Motor 1 //
/////////////
#define M1CS                p18 // current sense
#define M1EN                p24 // enable M1
#define M1INB               p23 // input A
#define M1INA               p22 // input B
#define M1PWM               p21 // PWM 


/////////////
// Motor 2 //
/////////////
#define M2EN                p8  // 
#define M2INB               p10
#define M2INA               p11
#define M2CS                p19
#define M2PWM               p25

class MotorShield
{
  public:   
    // CONSTRUCTORS
    MotorShield(); // Default pin selection.
    MotorShield(unsigned char INA1, unsigned char INB1, unsigned char EN1DIAG1, unsigned char CS1, 
                           unsigned char INA2, unsigned char INB2, unsigned char EN2DIAG2, unsigned char CS2); // User-defined pin selection. 
    
    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ. 
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    void setM1Brake(int brake); // Brake M1. 
    void setM2Brake(int brake); // Brake M2.
    void setBrakes(int m1Brake, int m2Brake); // Brake both M1 and M2.
    float getM1CurrentMilliamps(); // Get current reading for M1. 
    float getM2CurrentMilliamps(); // Get current reading for M2.
    unsigned char getM1Fault(); // Get fault reading from M1.
    unsigned char getM2Fault(); // Get fault reading from M2.
    
  private:
    // Digital Outputs
    DigitalOut _INA1;
    DigitalOut _INB1;
    PwmOut _PWM1;
    PwmOut _PWM2;
    DigitalOut _INA2;
    DigitalOut _INB2;
    // Digital Inputs
    DigitalIn _EN1DIAG1;
    DigitalIn _EN2DIAG2;
    // Analog Inputs
    AnalogIn _CS1;
    AnalogIn _CS2;
    
};

#endif