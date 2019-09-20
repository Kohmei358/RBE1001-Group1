// VEX V5 C++ Project
#include "vex.h"
#include "math.h"
using namespace vex;

const double kP = 0.001;
const double kI = 0.01;


//#region config_globals
vex::brain             Brain;
vex::motor             motorLeft(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor             motorRight(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor             motorArm(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::vision            visionMain(vex::PORT12);
vex::vision::signature sig_TARGET(1,293,585,439,-3999,-3717,-3858,3,0);
//#endregion config_globals

double degToRad(double deg){
    return (deg*2*3.14)/360;
}

void stop(){
    motorLeft.stop(brakeType::hold);
    motorRight.stop(brakeType::hold);
}

void moveForwards(int power, int distance){
    motorLeft.rotateFor(directionType::fwd,distance,rotationUnits::deg,power,velocityUnits::pct,false);
    motorRight.rotateFor(directionType::fwd,distance,rotationUnits::deg,power,velocityUnits::pct);
    stop();
}

void turnLeft(int power, int distance){
    motorLeft.rotateFor(directionType::rev,distance,rotationUnits::deg,power,velocityUnits::pct,false);
    motorRight.rotateFor(directionType::fwd,distance,rotationUnits::deg,power,velocityUnits::pct);
    stop();
}

void pickUp(){
    //reset arm rotaion after clicking limit switch
    //move to pickup angle
    //move back
    //move arm up
}
void dropOff(){
    //90 deg point turn
    //move arm down
    //move forward
}

double alignGoalTo(int xTarget){
    Brain.Screen.printLine(8,"Align: %d",xTarget);
    double error = 5;
    double totalError = 0;
    motorLeft.resetRotation();
    motorRight.resetRotation();
    while(abs(error) > 0.00001){
        visionMain.takeSnapshot(sig_TARGET);
        Brain.Screen.printLine(9,"x: %d \t y: %d \t width: %d",visionMain.largestObject.centerX,visionMain.largestObject.centerY,visionMain.largestObject.width);
        if(visionMain.largestObject.width > 10){
            error = (xTarget - visionMain.largestObject.centerX)/10;
            totalError += error;
            Brain.Screen.printLine(10,"E: %f, TE: %f",error,totalError);
            motorLeft.spin(directionType::fwd,error*kP+totalError*kI,percentUnits::pct);
            motorRight.spin(directionType::rev,error*kP+totalError*kI,percentUnits::pct);
        }else{
            Brain.Screen.printLine(10,"No Data");
            //totalError = 0;
            stop();
        }
        sleepMs(10);
    }
    stop();
    double angleMoved = abs((motorLeft.rotation(rotationUnits::deg)-motorRight.rotation(rotationUnits::deg))/(2));
    return angleMoved;
}

void goToGoal(){
    Brain.Screen.printLine(8,"GoToGoal");
    double rot = alignGoalTo(160);
    Brain.Screen.printLine(1,"Rot: %f", rot);
    double angle = (rot/6);
    Brain.Screen.printLine(2,"Ang: %f",angle);
    double cose = cos(degToRad(angle));
    Brain.Screen.printLine(3,"Cos: %f",cose);
    Brain.Screen.printLine(4,"Done");
    sleepMs(100000);
    moveForwards(30,300.0/cose);
    alignGoalTo(130);
    moveForwards(50,250);
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
	visionMain.setBrightness(55);
	visionMain.setSignature(sig_TARGET);
	//#endregion config_init
							
		// 	pickUp();
// 	while(noStopSign()){
// 	    //line track
// 	}
// 	while(noStopLine()){
// 	    //line track
// 	}
// 	dropOff();				
    goToGoal();
}
