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
const int light_threshold = 50;
double e;

void pickUp(){
    //reset arm rotaion after clicking limit switch
	vex::limit.pressing();
    //move to pickup angle
    //move back
    //move arm up
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
	float sensorValue = mainSonar.distance(distanceUnits::in);
 	if((max_dis > sensorValue) and (min_dis < sensorValue))// and ((targetArea + range) > area) and ((targetArea - range) < area)))
	{
		return false;
	 }
	 else{ //outside 5-20in
	 	return true;
	 }
}
bool noStopLine(){
    int Lreading = leftLight.value(percentUnits::pct);
    int Rreading = rightLight.value(percentUnits::pct);
    if((Lreading < light_threshold) and (Rreading < light_threshold)){
        return true;
    }
    else{
        return false;
    }
}
}

int main(void) {
	//#region config_init
	visonMain.setBrightness(55);
	visonMain.setSignature(sig_STOP_SIGN);
	visonMain.setSignature(sig_TARGET);
	//#endregion config_init
	pickUp();
	while(noStopSign()){
	    //line track
	}
	while(noStopLine()){
	    //line track
	}
	dropOff();				
    goToGoal();
}

    
    // motorLeft.spin(fwd,30,percentUnits::pct);
    // motorRight.spin(fwd,-30,percentUnits::pct);
    // sleepMs(1000);
    // motorLeft.stop();
    // motorRight.stop();
    // sleepMs(1000);
    // motorArm.startRotateFor(-100,rotationUnits::deg);
    // motorLeft.spin(fwd,30,percentUnits::pct);
    // motorRight.spin(fwd,30,percentUnits::pct);
    // sleepMs(300);
    // motorLeft.stop();
    // motorRight.stop();
    // sleepMs(1000);
    // while(centerX-190 > 5 || centerX-190 < -5){
    //     if(centerX-190 < 5){
    //         motorLeft.spin(fwd,1.5,voltageUnits::volt);
    //         motorRight.spin(fwd,-1.5,voltageUnits::volt);
    //     }
    //     else if(centerX-190 > -5){
    //         motorLeft.spin(fwd,-1.5,voltageUnits::volt);
    //         motorRight.spin(fwd,1.5,voltageUnits::volt);
    //     }
    //     visonMain.takeSnapshot(sig_TARGET);
    //     centerX = visonMain.largestObject.centerX;
    //     Brain.Screen.printLine(1,"x: %d, y:%d\n", visonMain.largestObject.centerX, visonMain.largestObject.centerY);
    // }
    // motorLeft.stop();
    // motorRight.stop();
    // sleepMs(1000);
    // motorLeft.spin(fwd,3,voltageUnits::volt);
    // motorRight.spin(fwd,3,voltageUnits::volt);
    // sleepMs(1700);
    // motorLeft.stop();
    // motorRight.stop();
    // while(centerX-134 > 5 || centerX-134 < -5){
    //     if(centerX-134 < 5){
    //         motorLeft.spin(fwd,1.0,voltageUnits::volt);
    //         motorRight.spin(fwd,-1.0,voltageUnits::volt);
    //     }
    //     else if(centerX-134 > -5){
    //         motorLeft.spin(fwd,-1.0,voltageUnits::volt);
    //         motorRight.spin(fwd,1.0,voltageUnits::volt);
    //     }
    //     visonMain.takeSnapshot(sig_TARGET);
    //     centerX = visonMain.largestObject.centerX;
    //     Brain.Screen.printLine(1,"x: %d, y:%d\n", visonMain.largestObject.centerX, visonMain.largestObject.centerY);
    // }
    // motorLeft.stop();
    // motorRight.stop();
    // motorLeft.spin(fwd,5,voltageUnits::volt);
    // motorRight.spin(fwd,5,voltageUnits::volt);
    // sleepMs(300);
    // motorLeft.stop(brakeType::hold);
    // motorRight.stop(brakeType::hold);
    // while(true){
    //     visonMain.takeSnapshot(sig_TARGET);
    //     centerX = visonMain.largestObject.centerX;
    //     Brain.Screen.printLine(1,"x: %d, y:%d\n", visonMain.largestObject.centerX, visonMain.largestObject.centerY);
    // }
