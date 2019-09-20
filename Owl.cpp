// VEX V5 C++ Project
#include "vex.h"
#include "math.h"
using namespace vex;

const double kP = 0.001;
const double kI = 0.01;

vex::brain  Brain;
vex::motor  motorLeft(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor  motorRight(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor  motorArm(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::vision visionMain(vex::PORT12);
vex::sonar  mainSonar(Brain.ThreeWirePort.A);
vex::line   rightLight(Brain.ThreeWirePort.D);
vex::line   leftLight(Brain.ThreeWirePort.E);
vex::bumper limitMain(Brain.ThreeWirePort.F);

vex::vision::signature sig_TARGET(1,293,585,439,-3999,-3717,-3858,3,0);
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
const int resetAngle = -960;
const int backupdist = -90;



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
	motorArm.rotateTo(-800, rotationUnits::deg, 50, velocityUnits::pct);
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
    float sensorValue = mainSonar.distance(distanceUnits::in);

   if((max_dis > sensorValue) and (min_dis < sensorValue))
   {
       return false;
   }
   else{ //outside 5-20in
       return true;
   }
}
bool noStopLine(){
    //return true is no stop line
    //false if on stop line
}

int main(void) {
	
// 	pickUp();
// 	sleepMs(500);
//  	turnLeft(15, -180*2.81);
//  	sleepMs(500);
//  	moveForwards(20, -400);
	while(noStopSign() /*&& noStopLine()*/){
	    lineTrack();
	}
// 	dropOff();				

//     goToGoal();
}
