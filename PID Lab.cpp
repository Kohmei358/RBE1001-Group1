// VEX V5 C++ Project
#include "vex.h"
using namespace vex;
const double kC = 2048.0;
const double pC = 1.8;
const double kP = 85;
const double kI = 0.03*(2*kP/pC);
const double kD = (0.185*kP*pC);
const double target = 45;
const int leftThresh = 1550;
const int rightThresh = 2075;
const double timeConst = 0.1;
double e;



//#region config_globals
vex::brain Brain;
vex::motor motorLeft(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor motorRight(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor motorArm(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::sonar mainSonar(Brain.ThreeWirePort.A);
vex::line leftLight(Brain.ThreeWirePort.E);
vex::line rightLight(Brain.ThreeWirePort.D);
vex::controller radio = vex::controller();
//#endregion config_globals

void driveAtSpeed(double speed){
    motorLeft.spin(directionType::fwd, speed, voltageUnits::volt);
    motorRight.spin(directionType::fwd, speed, voltageUnits::volt);
}

double map(double darkVolts,double lightVolts , double darkPct, double lightPct, double pct){
    double temp = ((((pct - darkPct)*(lightVolts - darkVolts))/(lightPct - darkPct)) + darkVolts);
    return temp;
}

int main(void) {
    Brain.Timer.clear();
    //Brain.Screen.clearScreen();
    double intergral = 0;
    double prevError = 0;
    double error = 0;
    double steering = 0;
    while(true){
        if(Brain.Timer.time(timeUnits::sec) > 3){
            Brain.Timer.clear();
            intergral = 0;
        }
        //motorLeft.spin(directionType::fwd,map(0.3,2,64,33,leftLight.value(percentUnits::pct)),voltageUnits::volt);
        //motorRight.spin(directionType::fwd,map(0.3,2,63,8,rightLight.value(percentUnits::pct)),voltageUnits::volt);
        
        error = map(0,1,58,3,leftLight.value(percentUnits::pct)) - map(0,1,63,3,rightLight.value(percentUnits::pct));
        error = error*0.1;
        Brain.Screen.printLine(1,"Error: %f, Steeing: %f",error,steering);
        Brain.Screen.printLine(2,"T: %f",Brain.Timer.time(timeUnits::sec));
        
        intergral += error * timeConst;
        double derivative = (error-prevError)/timeConst;
        steering = kP*error + kI*intergral + kD*derivative;
        // if(steering > 4){
        //     steering = 4;
        // }else if(steering < -4){
        //     steering = -4;
        // }
        prevError = error;
        sleepMs(10);
        if(steering > 0){
            motorLeft.spin(directionType::rev,2.6+steering,voltageUnits::volt);
            motorRight.spin(directionType::rev,2.6-(0.8*steering),voltageUnits::volt);
        }else{
            motorLeft.spin(directionType::rev,2.6+(0.8*steering),voltageUnits::volt);
            motorRight.spin(directionType::rev,2.6-steering,voltageUnits::volt);
        }

        
    }
}
