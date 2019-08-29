// VEX V5 C++ Project
#include "vex.h"

using namespace vex;

const double pi = 3.14159;
const int gearRatio = 5;
const double fudgeFactorRotate =0.895;
const double fudgeFactorForward = 0.983;


//#region config_globals
vex::brain Brain;
vex::motor motorLeft(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor motorRight(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor motorArm(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::sonar mainSonar(Brain.ThreeWirePort.A);
//#endregion config_globals

void stop(){
    motorRight.stop(brakeType::hold);
    motorLeft.stop(brakeType::hold);
}

void goForward(double distance, double wheelDiameter){
    motorLeft.startRotateFor(directionType::fwd,fudgeFactorForward*distance/(pi*wheelDiameter)*gearRatio,rotationUnits::rev, 30, velocityUnits::pct);
    motorRight.rotateFor(directionType::fwd,fudgeFactorForward*distance/(pi*wheelDiameter)*gearRatio,rotationUnits::rev, 30, velocityUnits::pct);
    stop();
}
void rotate(double degrees, double wheelDiameter, double wheelTrack){
    motorLeft.startRotateFor(directionType::rev,fudgeFactorRotate*(wheelTrack*degrees*gearRatio)/(360*wheelDiameter),rotationUnits::rev, 30, velocityUnits::pct);
    motorRight.rotateFor(directionType::fwd,fudgeFactorRotate*(wheelTrack*degrees*gearRatio)/(360*wheelDiameter),rotationUnits::rev, 30, velocityUnits::pct);
    stop();
}

void circle(double wheelDiameter){
    motorLeft.startRotateFor(directionType::fwd,5*1888.097/(pi*wheelDiameter),rotationUnits::rev,11.25,velocityUnits::pct);
    motorRight.rotateFor(directionType::fwd,5*110.097/(pi*wheelDiameter), rotationUnits::rev,29 , velocityUnits::pct);
    stop();
}

void circleTurn(double wheelDiameter, double outerDiameter, double wheelTrack){
    double turnRatio = outerDiameter/(outerDiameter - wheelTrack);
    motorRight.startRotateFor(directionType::fwd,5*99999/(pi*wheelDiameter), rotationUnits::rev, (30/turnRatio)*1.65 , velocityUnits::pct);
    motorLeft.rotateFor(directionType::fwd,(gearRatio*outerDiameter)/(2.2*wheelDiameter),rotationUnits::rev,30,velocityUnits::pct);
    
    stop();
}

void waitMSec(double timeValue){
    Brain.resetTimer();
    while(Brain.timer(timeUnits::msec) < timeValue){
        
    }
}
void star(double degrees, double distance, double wheelDiameter, double wheelTrack){
   for(int i = 0; i < 5; i++){
        goForward(distance, wheelDiameter);
        waitMSec(300);
        rotate(degrees, wheelDiameter, wheelTrack);
        waitMSec(300);
    }
}
void maze(double wheelDiameter, double wheelTrack){
    goForward(30, wheelDiameter);
    rotate(90, wheelDiameter, wheelTrack);
    waitMSec(300);
    
    goForward(20.5, wheelDiameter);
    waitMSec(300);
    
    // rotate(-90, wheelDiameter, wheelTrack);
    // goForward(15, wheelDiameter);
    //waitMSec(300);
    circleTurn(4, 13, 12);
    goForward(2.5, 4);
    circleTurn(4, 13, 12);
    goForward(3, 4);
    // rotate(-90, wheelDiameter, wheelTrack);
    // waitMSec(300);
    
    // goForward(8, wheelDiameter);
}




void square(){
    waitMSec(3000);
    for(int i = 0; i < 4; i++){
        goForward(24, 4);
        waitMSec(1000.0);
        rotate(90, 4, 12);
        waitMSec(1000.0);
    }
}

int main(void) {
    motorArm.stop(brakeType::hold);
    //star(144, 24, 4, 12);
    //square();
    //maze(4,12);
    Brain.Screen.setFillColor(white);
    Brain.Screen.setPenColor(black);
    Brain.Screen.setCursor(1,1);
    motorLeft.resetRotation();
    Brain.Screen.printLine(1,"Left Enc: %f",motorLeft.rotation(rotationUnits::rev));
    waitMSec(1000);
    for(int i = 0; i < 20; i++){
        Brain.Screen.clearScreen();
        Brain.Screen.printLine(1,"Sonar: %f",mainSonar.distance(distanceUnits::mm)); 
        waitMSec(300);
        Brain.Screen.clearScreen();
        waitMSec(300);
    }
    waitMSec(2000);
    goForward(79,4);
    waitMSec(2000);
    Brain.Screen.printLine(1,"Left Enc: %f",motorLeft.rotation(rotationUnits::rev));
    waitMSec(1000);
    for(int i = 0; i < 20; i++){
        Brain.Screen.clearScreen();
        Brain.Screen.printLine(1,"Sonar: %f",mainSonar.distance(distanceUnits::mm)); 
        waitMSec(300);
        Brain.Screen.clearScreen();
        waitMSec(300);
    }
    waitMSec(20000);
}
