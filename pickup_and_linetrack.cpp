// VEX V5 C++ Project
#include "vex.h"
using namespace vex;

int centerX;

//#region config_globals
vex::brain             Brain;
vex::motor             motorLeft(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor             motorRight(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor             motorArm(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::vision            visonMain(vex::PORT12);
vex::limit             limitMain(Brain.ThreeWirePort.F);
vex::vision::signature sig_STOP_SIGN(1,10219,10813,10516,-1111,-803,-957,3,0);
vex::vision::signature sig_TARGET(2,553,883,718,-4005,-3753,-3879,3,0);
vex::sonar mainSonar(Brain.ThreeWirePort.A);
vex::line leftLight(Brain.ThreeWirePort.E);
vex::line rightLight(Brain.ThreeWirePort.D);
//#endregion config_globals
const double kC = 2048.0;
const double pC = 1.8;
const double kP = 85;
const double kI = 0.03*(2*kP/pC);
const double kD = (0.185*kP*pC);
const double target = 45;
const int leftThresh = 1550;
const int rightThresh = 2075;
const double timeConst = 0.1;
const double collectWasteAngle = 125.31;
const int resetAngle = 73;
const double travellingAngle = collectWasteAngle - (resetAngle + 90);
const int wheelRad = 2;
double e;

void pickUp(){
    bool lastPressed = limitMain.pressing();
	//reset arm rotaion
	while(!lastPressed){
		motorArm.spin(directionType::rev, 5, voltageUnits::volt);
		lastPressed = limitMain.pressing();
	}
		if(limitMain.pressing())
	{
	motorArm.resetRotation();
	motorArm.rotateTo(resetAngle*5, rotationUnits::deg, 50, velocityUnits::pct);
	}
    //reset arm rotaion after clicking limit switch
    //move to pickup angle
	motorArm.rotateTo(collectWasteAngle*5, rotationUnits::deg, 50, velocityUnits::pct);
    //move back
	move(-2);
    //move arm up
	motorArm.rotateTo(10, rotationUnits::deg, 50, velocityUnits::pct);
}
void driveAtSpeed(double speed){
    motorLeft.spin(directionType::fwd, speed, voltageUnits::volt);
    motorRight.spin(directionType::fwd, speed, voltageUnits::volt);
}

double map(double darkVolts,double lightVolts , double darkPct, double lightPct, double pct){
    double temp = ((((pct - darkPct)*(lightVolts - darkVolts))/(lightPct - darkPct)) + darkVolts);
    return temp;
}
void lineTrack(){
Brain.Timer.clear();
    double intergral = 0;
    double prevError = 0;
    double error = 0;
    double steering = 0;
    while(true){
        if(Brain.Timer.time(timeUnits::sec) > 3){
            Brain.Timer.clear();
            intergral = 0;
        }        
        error = map(0,1,58,3,leftLight.value(percentUnits::pct)) - map(0,1,63,3,rightLight.value(percentUnits::pct));
        error = error*0.1;
        Brain.Screen.printLine(1,"Error: %f, Steering: %f",error,steering);
        Brain.Screen.printLine(2,"T: %f",Brain.Timer.time(timeUnits::sec));
        
        intergral += error * timeConst;
        double derivative = (error-prevError)/timeConst;
        steering = kP*error + kI*intergral + kD*derivative;
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
void dropOff(){
    //90 deg point turn
    //move arm down
    //move forward
}
void goToGoal(){
    //use vison or ultrasonic to move from dropoff
    //to goal
}

bool noStopSign(){
    //return true is noStopSign
    //false if 5-20inches from stopSign
}
bool noStopLine(){
    //return true is no stop line
    //false if on stop line
}

int main(void) {
	//#region config_init
	visonMain.setBrightness(55);
	visonMain.setSignature(sig_STOP_SIGN);
	visonMain.setSignature(sig_TARGET);
	//#endregion config_init
	pickUp();
	turn(180);
	while(noStopSign() && noStopLine){
	    lineTrack();
	}
	dropOff();				
    goToGoal();
}
