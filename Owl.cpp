// VEX V5 C++ Project
#include "vex.h"
#include "math.h"
using namespace vex;

double kP = 0.06;
double kI = 0.00001;
double kP2 = 0.3;
double kI2 = 0.001;


//#region config_globals
vex::brain             Brain;
vex::motor             motorLeft(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor             motorRight(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor             motorArm(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::vision            visionMain(vex::PORT12);
vex::vision::signature sig_TARGET(1,-307,-127,-217,-3539,-3317,-3428,3,0);
vex::sonar  mainSonar(Brain.ThreeWirePort.A);
vex::line   rightLight(Brain.ThreeWirePort.D);
vex::line   leftLight(Brain.ThreeWirePort.E);
vex::bumper limitMain(Brain.ThreeWirePort.F);
//#endregion config_globals
float min_dis = 5.0;
float max_dis = 20.0;
const double kC = 2048.0;
const double pC = 1.8;
//const double kP = 85;
//const double kI = 0.03*(2*kP/pC);
const double kD = (0.185*kP*pC);
const double target = 45;
const int leftThresh = 1550;
const int rightThresh = 2075;
const double timeConst = 0.1;
const int wheelRad = 2;
double e;
const int resetAngle = -930;
const int backupdist = -80;
double derivativeError = 0;
double intergralError = 0;
double prevError = 0;
double error = 0;
double steering = 0;
const int light_threshold = 50;

double degToRad(double deg){
    return (deg*2*3.14)/360;
}

void stop(){
    motorLeft.stop(brakeType::brake);
    motorRight.stop(brakeType::brake);
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
	//reset arm rotaion
	motorArm.spin(directionType::fwd, 12, voltageUnits::volt);
	while(!limitMain.pressing()){
	    Brain.Screen.printLine(1,"Wait for btn");
	}
	Brain.Screen.clearScreen();
    //reset arm rotation after clicking limit switch
    //move to pickup angle
    motorArm.resetRotation();
	motorArm.rotateTo(resetAngle, rotationUnits::deg, 100, velocityUnits::pct);
    sleepMs(350);
	moveForwards(20, backupdist);
	sleepMs(350);
    //move arm up
	motorArm.rotateTo(-250, rotationUnits::deg, 50, velocityUnits::pct);
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
    prevError = error;
    error = (32-rightLight.value(percentUnits::pct))*kP;
    error = error*error*error;
    intergralError += error;
    derivativeError = prevError-error;
    Brain.Screen.printLine(1,"Intgreal: %f     D: %f",intergralError,derivativeError);
    if(Brain.Timer.time(timeUnits::sec) > 3){
        kP = 0.05;
        motorLeft.spin(directionType::rev,2.1-error-intergralError*kI,voltageUnits::volt);
        motorRight.spin(directionType::rev,2.1+error+intergralError*kI,voltageUnits::volt);
    }else{
        motorLeft.spin(directionType::rev,4.1-error-intergralError*kI,voltageUnits::volt);
        motorRight.spin(directionType::rev,4.1+error+intergralError*kI,voltageUnits::volt);
    }
    
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
            motorLeft.spin(directionType::fwd,error*kP2+totalError*kI2,percentUnits::pct);
            motorRight.spin(directionType::rev,error*kP2+totalError*kI2,percentUnits::pct);
        }else{
            Brain.Screen.printLine(10,"No Data");
            //totalError = 0;
            stop();
        }
        sleepMs(10);
    }
    stop();
    double angleMoved = (motorLeft.rotation(rotationUnits::deg)-motorRight.rotation(rotationUnits::deg))/(2);
    turnLeft(5,-0.2*angleMoved);
    angleMoved = (motorLeft.rotation(rotationUnits::deg)-motorRight.rotation(rotationUnits::deg))/(2);
    return angleMoved;
}

void goToGoal(){
    Brain.Screen.printLine(8,"GoToGoal");
    double rot = alignGoalTo(130);
    Brain.Screen.printLine(1,"Rot: %f", rot);
    double angle = (rot/6);
    Brain.Screen.printLine(2,"Ang: %f",angle);
    double cose = cos(degToRad(angle));
    Brain.Screen.printLine(3,"Cos: %f",cose);
    Brain.Screen.printLine(4,"Done");
    moveForwards(30,500.0/cose);
    turnLeft(10,rot);
    moveForwards(50,250);
}

bool noStopSign(){
    float sensorValue = mainSonar.distance(distanceUnits::in);
    Brain.Screen.printLine(3,"Distance: %f",sensorValue);
    if(20 > sensorValue && 2 < sensorValue)
    {
       return false;
    }
    else{ //outside 5-20in
       return true;
    }
}
bool noStopLine(){
    //Brain.Screen.printLine(3,"L: %f R: %f",leftLight.value(percentUnits::pct) ,rightLight.value(percentUnits::pct));
    if(leftLight.value(percentUnits::pct) > 30 && rightLight.value(percentUnits::pct) > 30){
        return false;
    }
    else{
        return true;
    }
}

int main(void) {
	//#region config_init
	visionMain.setBrightness(55);
	visionMain.setSignature(sig_TARGET);
	//#endregion config_init
	
	pickUp();
 	turnLeft(25, -180*2.95);
 	sleepMs(350);
 	moveForwards(20, -300);
    stop();
    sleepMs(200);
    
 	//line track
 	Brain.Timer.clear();
	while(noStopSign()){
        lineTrack();
        sleepMs(5);
    }
    Brain.Screen.clearScreen();
    stop();
    sleepMs(1000);
    intergralError = 0;
    double sonarVal = mainSonar.distance(distanceUnits::in);
    while(sonarVal < 23 && sonarVal > 2){
        sleepMs(20);
        sonarVal = mainSonar.distance(distanceUnits::in);
    }
    Brain.Screen.clearScreen();
    while(leftLight.value(percentUnits::pct) < 30 || rightLight.value(percentUnits::pct) < 30){
        Brain.Screen.printLine(3,"L: %f R: %f",leftLight.value(percentUnits::pct) ,rightLight.value(percentUnits::pct));
        lineTrack();
        sleepMs(10);
    }
    stop();
    turnLeft(20,90*2.7);
    sleepMs(350);
    moveForwards(-20,-170);
    motorArm.rotateTo(resetAngle, rotationUnits::deg, 100, velocityUnits::pct);
    moveForwards(20,170);
    
	dropOff();				
    goToGoal();
    
}
