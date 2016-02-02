#include "MotorShield.h"
    // CONSTRUCTORS
MotorShield::MotorShield():_INA1(M1INA),_INB1(M1INB), _PWM1(M1PWM),
_PWM2(M2PWM),_INA2(M2INA),_INB2(M2INB), _EN1DIAG1(M1EN),_EN2DIAG2(M2EN),
_CS1(M1CS), _CS2(M2CS)
{    }

    // PUBLIC METHODS
void MotorShield::init() // Set the PWM to 20kHZ. 
{
    printf("Shield Initialized...\n");
    _PWM1.period(0.00005f);
    _PWM2.period(0.00005 f);
    
}

// number between -400 and 400
void MotorShield::setM1Speed(int speed){ // SET SPEED FOR M1
    unsigned char reverse = 0;

    if(speed < 0){
        speed = -speed; // make speed positive
        reverse = 1;  // preserve the direction
    }
    if(speed > 400) speed = 400;
    _PWM1.write(speed*1.0f/400);
    if(speed == 0){
        _INA1 = 0;
        _INB1 = 0;
    }else if(reverse){
        _INA1 = 0;
        _INB1 = 1;
    }else{
        _INA1 = 1;
        _INB1 = 0;
    }
} 
void MotorShield::setM2Speed(int speed){
    unsigned char reverse = 0;

    if(speed < 0){
        speed = -speed; // make speed positive
        reverse = 1;  // preserve the direction
    }
    if(speed > 400) speed = 400;
    _PWM2.write(speed*1.0f/400);
    if(speed == 0){
        _INA2 = 0;
        _INB2 = 0;
    }else if(reverse){
        _INA2 = 0;
        _INB2 = 1;
    }else{
        _INA2 = 1;
        _INB2 = 0;
    }
} // Set speed for M2.
void MotorShield::setSpeeds(int m1Speed, int m2Speed){
    setM1Speed(m1Speed);
    setM2Speed(m2Speed);
} // Set speed for both M1 and M2.
void MotorShield::setM1Brake(int brake){
    if(brake < 0){
        brake = -brake;
    }
    if(brake > 400) brake= 400;
    _INA1 = 0;
    _INB1 = 0;
    _PWM1.write(brake*1.0f/400);
} // Brake M1. 
void MotorShield::setM2Brake(int brake){
    if(brake < 0){
        brake = -brake;
    }
    if(brake > 400) brake= 400;
    _INA2 = 0;
    _INB2 = 0;
    _PWM2.write(brake*1.0f/400);
} // Brake M2.
void MotorShield::setBrakes(int m1Brake, int m2Brake){
    setM1Brake(m1Brake);
    setM2Brake(m2Brake);
} // Brake both M1 and M2.
float MotorShield::getM1CurrentMilliamps(){
    // 1.0f = 3.3V --- 140mV/A
    return (_CS1.read() * 3.3 * 1000/0.14);

} // Get current reading for M1. 
float MotorShield::getM2CurrentMilliamps(){
    return (_CS2.read() * 3.3 * 1000/0.14);
} // Get current reading for M2.
unsigned char MotorShield::getM1Fault(){
    return !_EN1DIAG1.read();
} // Get fault reading from M1.
unsigned char MotorShield::getM2Fault(){
    return !_EN2DIAG2.read();
} // Get fault reading from M2.
