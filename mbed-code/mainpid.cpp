// Copyright (C) 2015 Paulo Costa

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// Contact: pcesar222@gmail.com

#include "mbed.h"
#include "MPU6050.h"
#include "math.h"
#include "matrix.h"
#include "MotorShield.h"
#include "Kalman.h"
#include <string>

using namespace std;
/**********************************************
/*
/*      Hardware Pin Definitions
/*                       
/**********************************************/

///////////////
// Encoders  //
///////////////
#define ENC1A             p5
#define ENC1B             p6
#define ENC2A             p15
#define ENC2B             p16
    
// Value used for averaging encoder speed
#define SPEED_AVERAGE_FACTOR    8
#define WHEEL_RADIUS            0.03
#define ENC_TO_RAD              0.0033f  //(360.0f/64/30*PI/180)

//////////
// XBee //
//////////
#define MBED_TX             p13
#define MBED_RX             p14


#define PI 3.14159265358979

/**********************************************
/*
/*      Function prototypes 
/*                       
/**********************************************/

// encoder functions
void incrementEnc1a(void);
void incrementEnc1b(void);
void incrementEnc2a(void);
void incrementEnc2b(void);

void setEnc1aFall(void);
void setEnc1bFall(void);
void setEnc2aFall(void);
void setEnc2bFall(void);

void printEncoder(void);
void testEncoders(MotorShield);
//DigitalOut myled(LED1);

void assignGainsFromString(float*,float*,float*,float*,float*,string);
float constrain(float, float, float);


/**********************************************
/*
/*      Motor driver shield + XBee
/*                       
/**********************************************/


MotorShield shield;

Serial xBee(p13,p14); // tx, rx
Serial pc(USBTX, USBRX);
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);


/**********************************************
/*
/*      Encoder 
/*                       
/**********************************************/

InterruptIn enc1a(ENC1A);
InterruptIn enc1b(ENC1B);
InterruptIn enc2a(ENC2A);
InterruptIn enc2b(ENC2B);

 // t.read_us is a 32bit integer, max = ~2000s

int enc1aCount = 0;
int enc1bCount = 0;
int enc1total = 0;
int enc2aCount = 0;
int enc2bCount = 0;
int enc2total = 0;


// Timer variables
long float oldTime, newTime, lastEncTime;

// Speed 
float enc1Speed, enc2Speed;

// Let us now what direction the wheel is spinning
enum spinDirection {clockWise, counterClockWise} spinDirection1, spinDirection2;
enum lastTransition {rise, fall} enc1aLastTransition, enc1bLastTransition,
enc2aLastTransition, enc2bLastTransition; 



/**********************************************
/*
/*      Control loop constants
/*                       
/**********************************************/


float k[5] = {0,0,0,0,20};
//k[0] = Kp
//k[1] = Ki
//k[2] = Kd
//k[3] = thetaRef
//k[4] = sampleTime

// /**********************************************
// /*
// /*      Main Routine
// /*                       
// /**********************************************/


int main() {


    // MPU6050 object "MPU6050.h"
    MPU6050 accelgyro;
    // Kalman filter Object "Kalman.h"
    Kalman kalman;

    // Xbee serial baudrate
    xBee.baud(38400);
    pc.baud(9600);

    // Initialize timers
    Timer timer, code, encTimer;
    // Start timers
    timer.start(); 
    code.start();
    encTimer.start();
    // Reset timer values
    timer.reset();
    code.reset();
    encTimer.reset();

    //Encoder 1 interrupts
    enc1a.rise(&incrementEnc1a);
    enc1b.rise(&incrementEnc1b);
    enc1a.fall(&setEnc1aFall);
    enc1b.fall(&setEnc1bFall);

    enc2a.rise(&incrementEnc2a);
    enc2b.rise(&incrementEnc2b);
    enc2a.fall(&setEnc2aFall);
    enc2b.fall(&setEnc2bFall);

    // Debug leds
    myled1 = 0;
    myled2 = 0;
    myled3 = 0;

    // Encoder 1 initializations
    newTime =0; oldTime = 0; lastEncTime = 0;
    enc1Speed = 0; enc2Speed = 0;

    // MPU6050 initializations
    accelgyro.initialize();
    accelgyro.setFullScaleGyroRange(0);
    accelgyro.setFullScaleAccelRange(0);

    // Initialize MotorShield object
    shield.init();

    float measuredTheta , measuredRate, newTheta,calcRate;
    float position= 0, velocity = 0, lastPosition = 0, lastVelocity =0;
    float angleOffset = 0, positionOffset = 0,lastAngleoffset = 0;


    // Position control constants, if necessary
    float zoneA = 16000.0f, zoneB = 8000.0f, zoneC = 2000.0f;
    float multiplier = 1.0f;
    float zoneAscale = 600*2*multiplier, 
        zoneBscale = 800*2*multiplier, 
        zoneCscale = 1000*2*multiplier,
        zoneDscale =  1500*2*multiplier,
        velocityScale = 60*2*multiplier;

    // Serial communication buffer
    char buffer[40];
    int i, strSize;

    // Control parameters
    float k0,k1,k2,k3,tref;
    float x, dotx, deltaX, deltaT, lastX = 0;

    // Helper variables
    float waittime , u, diff, dt;

    float error = 0,lastError = 0;
    int counter = 0;
    float pTerm = 0,dTerm = 0,iTerm = 0;


    
    while(1) {

        ///////////////////////////////////////////
        // Read serial data and update constants //
        ///////////////////////////////////////////
        i = 0;
        char a;
        while(pc.readable()){
            a = pc.getc();
            buffer[i++] = a;
            printf("%c\n", a );
            myled1 = 1;
            wait(0.001);
         }

        strSize = i;
        string receive(buffer,strSize); // Changes from char to string
        if(strSize > 0){
            printf("%s\n", receive);
            assignGainsFromString(&k[0], &k[1], &k[2], &k[3], &k[4],receive);
        }

        // Below is an attempt to control the robot position, 
        // by dividing it into "zones" and changing the angle accordingly.
        // 
        /////////////////////////////
        // Generate encoder offset //
        /////////////////////////////

        // position = (float)enc2total;
        // positionOffset = position;

        // //if((encTimer.read_us() - lastEncTime) > 0.0f ) {   // every 100 ms
            
        //     float sign = 0;
        //     if(positionOffset > 0) sign= 1;
        //     else sign = -1;


        //     if(abs(positionOffset) > zoneA) angleOffset += 1.0f/zoneAscale*positionOffset;
        //     else if(abs(positionOffset) > zoneB) angleOffset += 1.0f/zoneBscale*positionOffset;
        //     else if(abs(positionOffset) > zoneC) angleOffset += 1.0f/zoneCscale*positionOffset;
        //     else angleOffset += positionOffset/zoneDscale;

        //     printf("angleOffset: %f, positionoffset: %f\n", angleOffset, positionOffset );
        //     // Estimate velocity
        //     // 
        //     velocity = (position - lastPosition);
        //     lastPosition = position;

        //     angleOffset += velocity/velocityScale;

        //     angleOffset = constrain(angleOffset,-10, 10);
            
        //     lastAngleoffset = angleOffset;
        // //}

        // angleOffset = constrain(angleOffset,lastAngleoffset - 1, lastAngleoffset + 1);
        
        timer.reset();
        ///////////////////////////
        // Get gyro/accel values //
        ///////////////////////////
        
        accelgyro.getAcceleration(&ax,&ay,&az);
        measuredRate = accelgyro.getRotationX()*1.0/131.0f;  // Units depend on config. Right now 250o/s
        measuredTheta = -atan(1.0*ay/az);   
        measuredTheta = measuredTheta*180/PI;  // measured in degrees
        
        ///////////////////
        // Kalman Filter //
        ///////////////////
        dt = (double)(code.read_us() - oldTime)/1000000.0f;
        newTheta = kalman.getAngle(measuredTheta,
            -measuredRate, dt);
        
        //DEBUG: printf("%g \n",  (double)(code.read_us() - oldTime)/1000000.0f);
        oldTime = code.read_us();

        //////////////////
        // Control loop //
        //////////////////

        // Set control variable to zero
        u = 0;
        
        // Extract constants from k vector, which has the serial readings.
        float kp = k[0];
        float ki = k[1];
        float kd = k[2];
        tref = k[3] - angleOffset;
        waittime = k[4];

        if(newTheta >= 50 || newTheta <= -50){
            u = 0;
        }else{

            // Define error term
            error = newTheta - tref;
            // Proportional term
            pTerm = kp*error;
            //Integral term
            iTerm += ki*error*dt*100.0f;
            // Prevent integration windup:
            if(iTerm >= 100) iTerm = 100; 
            else if (iTerm <= -100) iTerm = -100;

            u = -(iTerm + pTerm);
            
            // Calculated derivative based on smoothened angle.
            // There are two other sources for the angle here: kalman.getRate(); and measuredRate.
            calcRate = -(error-lastError)/dt;

            // Derivative term
            if(kd !=  0)
            dTerm = kd*calcRate/100.0f;
            lastError = error;

            u += dTerm;

            // Correct for dead zone --- Did not have successful results but I'll leave it here.
            // int deadzone = 20;
            // if(u < -deadzone) u = u + deadzone;
            // else if(u > deadzone) u = u - deadzone; 
            // else u = 0;
            
            // // Include saturation
            u = constrain(u,-400,400);
            
            // Flash LED to indicate loop
            if(counter % 50 == 0){
                myled3 = !myled3;    
                counter = 0;
            }
            counter++;
        }
        
        lastError = error; // update angle

        if(counter % 50 == 0){
            myled3 = !myled3;    
            // xBee.printf("%f,%f\n", newTheta,u);
            counter = 0;
        } 

        // Set motor speed
        shield.setM1Speed(-(int)u);
        shield.setM2Speed((int)u);

        // DEBUG over serial port. Use with MATLAB program "serialPlot.m" to plot these results.
        printf("%f,%f,%f\n", measuredTheta, newTheta , u);
        
        // Hold the timer until desired sampling time is reached.
        while(timer.read_us() < waittime*1000);

        // Turn off serial transmission LED
        myled1 = 0;

    }
}

/**********************************************
/*
/*      Encoder Interrupt Functions
/*                       
/**********************************************/

// Encoder 1
void incrementEnc1a(){

    // Establish the transition
    enc1aLastTransition = rise;
    enc1aCount++;

    // Debug
    // 32 so the readings don't come too often
    //if(enc1aCount%32 == 0) ;//printEncoder();

    // Determine direction
    enc1bLastTransition == fall ? spinDirection1 = clockWise : spinDirection1 = counterClockWise;

    // Update total count
    spinDirection1 == clockWise ? enc1total-- : enc1total++;
}
void incrementEnc1b(){
    // Establish the transition
    enc1bLastTransition = rise;
    enc1bCount++;

    // Determine direction
    enc1aLastTransition == rise ? spinDirection1 =clockWise : spinDirection1 = counterClockWise;

    // Update total count
    spinDirection1 == clockWise ? enc1total-- : enc1total++;
    
}

// Lets you know that it's fall
void setEnc1aFall(){
    // Establish transition
    enc1aLastTransition = fall;

    // Determine direction
    enc1bLastTransition == rise ? spinDirection1= clockWise : spinDirection1 = counterClockWise;

    // Update total count
    spinDirection1 == clockWise ? enc1total-- : enc1total++;
}

void setEnc1bFall(){
    // Establish transition
    enc1bLastTransition = fall;

    // Determine direction
    enc1aLastTransition == fall ? spinDirection1 = clockWise : spinDirection1 = counterClockWise;

    // Update total count
    spinDirection1 == clockWise ? enc1total-- : enc1total++;
}

// Encoder2
void incrementEnc2a(){
    // Establish transition
    enc2aLastTransition = rise;
    
    // Determine direction
    enc2bLastTransition == fall ? spinDirection2 = clockWise : spinDirection2 = counterClockWise;

    // Update total count
    spinDirection2 == clockWise ? enc2total++ : enc2total--;
}
void incrementEnc2b(){
    // Establish transition
    enc2bLastTransition = rise;
    enc2bCount++;

    // Determine direction
    enc2aLastTransition == rise? spinDirection2 = clockWise : spinDirection2 = counterClockWise;        
    
    // Update total count
    spinDirection2 == clockWise ? enc2total++ : enc2total--;
}

// Lets you know that it's fall
void setEnc2aFall(){
    // Establish transition
    enc2aLastTransition = fall;

    // Determine direction
    enc2bLastTransition == rise? spinDirection2 = clockWise : spinDirection2 = counterClockWise;
    
    // Update total count
    spinDirection2 == clockWise ? enc2total++ : enc2total--;
}

void setEnc2bFall(){
    // Establish transition
    enc2bLastTransition = fall;

    // Determine direction
    enc2aLastTransition == fall ? spinDirection2 = clockWise : spinDirection2 = counterClockWise;
    
    // Update total count
    spinDirection2 == clockWise ? enc2total++ : enc2total--;
}

// DEBUG
void printEncoder(){
    printf("\n************************\n");
    // print spin of wheel 1
    (spinDirection1 == clockWise )?
    printf("Wheel 1 direction: CLOCKWISE\n"):
    printf("Wheel 1 direction: COUNTER-CLOCKWISE\n");
    // info motor 1
    printf("enc1Count: %d\n", enc1total);

    // print spin of wheel 2
    (spinDirection2 == clockWise )?
    printf("Wheel 2 direction: CLOCKWISE\n"):
    printf("Wheel 2 direction: COUNTER-CLOCKWISE\n");
    // info motor 2
    printf("enc2Count: %d\n", enc2total);

    printf("************************\n");
}

/**********************************************
/*
/*      Control Functions
/*                       
/**********************************************/

// Gets a string of comma separated values (CSV) and assign them to the float arguments.
// This can be used with any serial program. I rather like the program "Realterm", for windows.
void assignGainsFromString(float * k0, 
    float * k1, 
    float * k2,
    float * thetaRef, 
    float * sampT,
    string receive){
    
    xBee.printf("String recebido: %s\n",receive );
    // 1,2,3,4,5,6
    myled2 = !myled2;
    string kk[5], holder;
    int index = receive.find(',', 0); // Find first comma
    int pos;
    if(index!=-1){
        kk[0] = receive.substr(0,index);
        holder = receive.substr(index+1,receive.length());
        index = holder.find(',',0);
        for(pos = 1; pos< 4 && (index!=-1 && (holder[0] != ',')); pos++){
            kk[pos] = holder.substr(0,index);
            holder = holder.substr(index+1,receive.length());
            index = holder.find(',',0);
        }
        wait(0.001);
        kk[pos]=holder;
        *k0 = atof(kk[0].c_str());
        *k1 = atof(kk[1].c_str());
        *k2 = atof(kk[2].c_str());
        *thetaRef = atof(kk[3].c_str());
        *sampT = atof(kk[4].c_str());

        printf("first: %f\n", *k0);
        printf("second: %f\n", *k1);
        printf("third: %f\n", *k2);
        printf("fifth: %f\n", *thetaRef);
        printf("sixth: %f\n", *sampT);

        
    }
    else{
        printf("Wrong format! try again \n");
    }
}

// Helper function that constrains the value to a specified interval.
float constrain(float value, float min, float max){
    float ans;
    if(value < min) ans = min;
    else if( value > max) ans = max;
    else ans = value;
    return ans;
}