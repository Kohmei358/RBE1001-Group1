/**
 * GRADING SECTION:
 * Line Tracking - drive->followLine()
 * Distance Measurement - sonarTrigger()
 * Encoder Measurement - all motors use rotate to, which uses encoder values
 *      all motors except drive use hard coded encoder targets
 * Sensor Assited Delivery - built in PID on every motor, line tracking, touch sensors
 * PID on Drive - drive.spin() uses a built in PID loop
 * PID on Control on arm - lift->setPos() uses lift.rotateTo() this uses built in PID loops
 * Assisted Remote Controll - check AutoSequence class
 * Effective State Machine - enums for robot state
 * Custom Display - check setUpScreen class
 * Signle Upload for autons - check setUpScreen class
 * Other demoed competency - singleton classes and multithreading
 **/

#include "vex.h"
using namespace vex;

vex::brain      Brain;
vex::motor      motorBackLeft(vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor      motorFrontLeft(vex::PORT12, vex::gearSetting::ratio18_1, false);
vex::motor      motorBackRight(vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor      motorFrontRight(vex::PORT11, vex::gearSetting::ratio18_1, false);
vex::motor      motorLeftLift(vex::PORT10, vex::gearSetting::ratio18_1, true);
vex::motor      motorRightLift(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::motor      motorPusher(vex::PORT13, vex::gearSetting::ratio18_1, true);
vex::motor      motorGrabber(vex::PORT9, vex::gearSetting::ratio18_1, false);
vex::controller con(vex::controllerType::primary);

vex::sonar  mainSonar(Brain.ThreeWirePort.A);
vex::line   mainLight(Brain.ThreeWirePort.D);
vex::line   stopLight(Brain.ThreeWirePort.E);
vex::bumper pizzaLimit(Brain.ThreeWirePort.F);
vex::bumper liftLimit(Brain.ThreeWirePort.C);
vex::bumper frontLimit(Brain.ThreeWirePort.G);

// Creates a competition object that allows access to Competition methods.
vex::competition Competition;

enum AUTOMODE{RED1,RED2,BLUE1,BLUE2,NONE}; //set by SetUpScreen - defaults to NONE
enum GAMEMODE{DRIVE,AUTO,COMP,NA}; //set by SetUpScreen - defaults to NA
//Robot does not run when NA is selected

AUTOMODE autonMode;
GAMEMODE robotMode;

const int LIFT_POS_COUNT = 5; //# of possible encoder positions for the lift
const int PUSHER_POS_COUNT = 2; //# of possible encoder positions for the pusher
const int GRABBER_POS_COUNT = 2; //# of possible encoder positions for the grabber

const int LINE_THRESH = 30; //if vlaue is below this line tracker is on line
const float LINE_KP = 0.2; //Line tracking KP
const float LINE_KI = 0.1; //Line tracking Ki
const float LINE_KD = 0.02; //line tracking kd
const int TURN_90 = 350; //constatnt motor roatation value for turing the robot 
//90 degrees measured in motor rotations
const int SONARDISTANCE = 15; //if value is less, a haptic rumble is triggered

class DriveTrain{
    private:
        DriveTrain(); //Singleton setup
    public:
        static DriveTrain* getInstance();
        void setDrivePower(int lPower, int rPower); //power in %
        void turnLeft(int deg);  //in motor rotaion deg
        void stop(); //stop breakType is break
        void followLine(); //follows line untill stopLine sees a line
        void forwardsUntillWall(); //moves forwards until front touch sensor hits
};

DriveTrain::DriveTrain(){}

DriveTrain* DriveTrain::getInstance(){                                                                                
    static DriveTrain instance;
    return &instance; //returns a pointer to the single instance                                                            
}       

void DriveTrain::forwardsUntillWall(){ //blocking function
    setDrivePower(40,40);
    while(!frontLimit.pressing()){}
    stop();
}

void DriveTrain::setDrivePower(int lPower, int rPower){
    motorBackLeft.spin(directionType::fwd,lPower,velocityUnits::pct);
    motorFrontLeft.spin(directionType::fwd,lPower,velocityUnits::pct);
    motorBackRight.spin(directionType::fwd,rPower,velocityUnits::pct);
    motorFrontRight.spin(directionType::fwd,rPower,velocityUnits::pct);
}

void DriveTrain::turnLeft(int deg){
    motorFrontLeft.rotateFor(-1*deg,vex::rotationUnits::deg,false);
    motorFrontRight.rotateFor(deg,vex::rotationUnits::deg,true);
}

void DriveTrain::stop(){
    motorFrontLeft.stop();
    motorFrontRight.stop();
}

void DriveTrain::followLine(){ //blocking follow line funciton
    double derivativeError = 0;
    double intergralError = 0;
    double prevError = 0;
    double error = 0;
    double result = 0;
    DriveTrain* drive = DriveTrain::getInstance();
    while(stopLight.value(vex::percentUnits::pct) < LINE_THRESH){
        error = (32 - mainLight.value(percentUnits::pct))*LINE_KP;
        intergralError += error;
        derivativeError = prevError-error;
        result = error*LINE_KP + intergralError*LINE_KI + derivativeError*LINE_KD;
        drive->setDrivePower(30-result, 30+result); //base speed of 30%
        prevError = error;
    }
    drive->stop();//the line tracker on the side has hit a perpendicular lien
}

class Lift{
    private:
        Lift();
        int levels[LIFT_POS_COUNT] = {0,137,277,373,522}; //encouner positions
        //the lift to go to
        int currPosIndex = 0; //which level is the lift on currernly?
    public:
        static Lift* getInstance(); //singleton again
        void setPos(int deg); //set lift motor position
        void resetRotation(); //set rotation to 0, triggered when limit switch pushed
        void nextPos(); //move up a position
        void prevPos(); //move down a position
        void updateLevel(); //goto the nearest preset position above the current lift positon
};

Lift::Lift(){ 
}

Lift* Lift::getInstance(){                                                                                
    static Lift instance;
    return &instance;                                                            
}       

void Lift::setPos(int deg){
    motorLeftLift.rotateTo(deg,vex::rotationUnits::deg,false);
    motorRightLift.rotateTo(deg,vex::rotationUnits::deg,false);
}

void Lift::nextPos(){
    currPosIndex++;
    if(currPosIndex >= LIFT_POS_COUNT) currPosIndex = 0; //postion past max level
    setPos(levels[currPosIndex]);
}
void Lift::prevPos(){
    currPosIndex--;
    if(currPosIndex <= -1) currPosIndex = LIFT_POS_COUNT-1; //postion below max level
    setPos(levels[currPosIndex]);
}

void Lift::resetRotation(){ //triggered by limit swtich
    motorLeftLift.resetRotation();
    motorRightLift.resetRotation();
}

void Lift::updateLevel(){
    int aveRotation = 0.5*(motorLeftLift.rotation(vex::rotationUnits::deg) + motorRightLift.rotation(vex::rotationUnits::deg));
    for(int i = 0; i < sizeof(levels); i++){
        if(aveRotation > levels[i]){
            //find nearest position above currernt lift pos and move to said level
            currPosIndex = i;
            setPos(levels[currPosIndex]);
            return;
        }
    }
}

class Pusher{ 
    //similar to lift class, onlt 2 poses
    private:
        Pusher();
        int currPosIndex = 0;
        int levels[PUSHER_POS_COUNT] = {0,-181};
    public:
        static Pusher* getInstance();
        void setPos(int deg);
        void resetRotation();
        void nextPos();
        void prevPos(); //for comments see lift class
        //this probably should have been a parent / child class
        
};

Pusher::Pusher(){
}

Pusher* Pusher::getInstance(){                                                                                
    static Pusher instance;
    return &instance;                                                            
}       

void Pusher::setPos(int deg){
    motorPusher.rotateTo(deg,vex::rotationUnits::deg,false);
}

void Pusher::nextPos(){
    currPosIndex++;
    if(currPosIndex >= PUSHER_POS_COUNT) currPosIndex = 0;
    setPos(levels[currPosIndex]);
}
void Pusher::prevPos(){
    currPosIndex--;
    if(currPosIndex <= -1) currPosIndex = PUSHER_POS_COUNT-1;
    setPos(levels[currPosIndex]);
}

void Pusher::resetRotation(){
    motorPusher.resetRotation();
}

class Grabber{
    //similar to lift class, onlt 2 poses
    //probably should have been parent / child class
    //for comments see lift class
    private:
        Grabber();
        int currPosIndex = 0;
        int levels[GRABBER_POS_COUNT] = {0,86};
    public:
        static Grabber* getInstance();
        void setPos(int deg);
        void resetRotation();
        void nextPos();
        void prevPos();
        
};

Grabber::Grabber(){
}

Grabber* Grabber::getInstance(){                                                                                
    static Grabber instance;
    return &instance;                                                            
}       

void Grabber::setPos(int deg){
    motorGrabber.rotateTo(deg,vex::rotationUnits::deg,false);
}

void Grabber::nextPos(){
    currPosIndex++;
    if(currPosIndex >= GRABBER_POS_COUNT) currPosIndex = 0;
    setPos(levels[currPosIndex]);
}
void Grabber::prevPos(){
    currPosIndex--;
    if(currPosIndex <= -1) currPosIndex = GRABBER_POS_COUNT-1;
    setPos(levels[currPosIndex]);
}

void Grabber::resetRotation(){
    motorGrabber.resetRotation();
}

//setup screen to choose auton and robot mode
class SetUpScreen{
    public:
        SetUpScreen();
        void displayMain(); //main screen with 4 buttons (Screen 0)
        void displayAuton(); //auton selection screen (Screen 1)
        void waitForInput(int screen); //call with screen number when user inputed needed
        void displayFinalScreen(); //confirmation screen (Screen 2)
};

SetUpScreen::SetUpScreen(){
    autonMode = NONE;
    robotMode = NA;
}

//1st Screen
void SetUpScreen::displayMain(){
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(blue);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(5,60,230,80);
    Brain.Screen.drawRectangle(5,160,230,80);
    Brain.Screen.drawRectangle(250,60,230,80);
    Brain.Screen.drawRectangle(250,160,230,80);
    Brain.Screen.setFont(fontType::mono30);
    Brain.Screen.printAt(80,90,"Select");
    Brain.Screen.printAt(90,120,"Auton");
    Brain.Screen.printAt(70,210,"Driver");
    Brain.Screen.printAt(340,110,"Auton");
    Brain.Screen.printAt(300,210,"Competition");
    Brain.Screen.setFont(fontType::prop40);
    Brain.Screen.setCursor(1, 0);
    switch(autonMode){
        case RED1: 
            Brain.Screen.setFillColor(red); 
            Brain.Screen.print("Current Auton: Red Front"); 
            break;
        case RED2: 
            Brain.Screen.setFillColor(red); 
            Brain.Screen.print("Current Auton: Red Back"); 
            break;
        case BLUE1: Brain.Screen.print("Current Auton: Blue Front"); break;
        case BLUE2:  Brain.Screen.print("Current Auton: Blue Back"); break;
        case NONE:          
            Brain.Screen.setFillColor(yellow);
            Brain.Screen.printAt(10,37,"Current Auton: EMPTY");
            break;
        default: Brain.Screen.print("Error!"); break;
    }
    Brain.Screen.render();
    
    waitForInput(0);
}

void SetUpScreen::waitForInput(int screen){
    while(!Brain.Screen.pressing()){}
    int x = Brain.Screen.xPosition();
    int y = Brain.Screen.yPosition();
    while(Brain.Screen.pressing()){}
    if(screen == 0){ //Main Screen
        if(x < 235 && y < 140){
                displayAuton();
        }
        else if(x < 235 && y > 160){
            robotMode = DRIVE;
           // Competition.drivercontrol(drivercontrol);
           SetUpScreen::displayFinalScreen();
        } 
        else if(x > 250 && y < 140){
            robotMode = AUTO;
           //Competition.autonomous(autonomous);
           SetUpScreen::displayFinalScreen();
        } 
        else if(x > 250 && y > 160){
            robotMode = COMP;
            competition();
            SetUpScreen::displayFinalScreen();
        }
    }else if(screen == 1){ //Auton Screen
        if(x < 240 && y < 120) {
            autonMode = RED1;
        } 
        else if(x < 240 && y > 120) {
            autonMode = RED2;
        } 
        else if(x > 240 && y < 120) {
            autonMode = BLUE1;
        } 
        else if(x > 240 && y > 120) {
            autonMode = BLUE2;
        }
        SetUpScreen::displayMain();
    }else if(screen == 2){
        SetUpScreen::displayMain();
    }
}

//Select Auton Screen
void SetUpScreen::displayAuton(){
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(fontType::mono20);
    Brain.Screen.drawRectangle(5, 122, 232, 112);
    Brain.Screen.drawRectangle(5, 5, 232, 112);
    Brain.Screen.printAt(85,60,"Red (LF)");
    Brain.Screen.printAt(95,80,"Front");
    Brain.Screen.printAt(85,177,"Red (LB)");
    Brain.Screen.printAt(105,197,"Back");
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(242, 5, 232, 112);
    Brain.Screen.drawRectangle(242, 122, 232, 112);
    Brain.Screen.printAt(312,60,"Blue (RF)");
    Brain.Screen.printAt(327,80,"Front");
    Brain.Screen.printAt(312,177,"Blue (RB)");
    Brain.Screen.printAt(334,197,"Back");
    Brain.Screen.render();
    
    waitForInput(1);
}

void SetUpScreen::displayFinalScreen(){
    Brain.Screen.clearScreen(green);
    Brain.Screen.setFillColor(green);
    Brain.Screen.setPenColor(black);
    Brain.Screen.setFont(fontType::prop40);
    Brain.Screen.setCursor(1, 0);
    Brain.Screen.print("Current Auton:");
    Brain.Screen.setCursor(2, 0);
    switch(autonMode){
        case RED1: Brain.Screen.print("RED1"); break;
        case RED2: Brain.Screen.print("RED2"); break;
        case BLUE1: Brain.Screen.print("BLUE1"); break;
        case BLUE2: Brain.Screen.print("BLUE2"); break;
        case NONE: Brain.Screen.print("NONE"); break;
        default: Brain.Screen.print("Error!"); break;
    }
    Brain.Screen.setCursor(3, 0);
    Brain.Screen.print("Robot Mode:");
    Brain.Screen.setCursor(4, 0);
    switch(robotMode){
        case AUTO: Brain.Screen.print("AUTO"); break;
        case DRIVE: Brain.Screen.print("DRIVE"); break;
        case COMP: Brain.Screen.print("COMP"); break;
        case NA: Brain.Screen.print("Error"); break;
        default: Brain.Screen.print("Error!"); break;
    }
    Brain.Screen.setCursor(5, 0);
    Brain.Screen.print("Touch Screen to change");
    Brain.Screen.render();
    SetUpScreen::waitForInput(2);
}

//Not a super necessary class but oh well
class AutoSequence{
    private:
        AutoSequence();
    public:
        static AutoSequence* getInstance();
        
        //call when touch sensor is pressed
        //grabs pizza and lifts to the nearest floor automatically
        void grabAndLiftPizza(); 
        
        //when at the right floor, pushes pizza into dorm
        void scorePizza();
        
};

AutoSequence::AutoSequence(){
}

AutoSequence* AutoSequence::getInstance(){                                                                                
    static AutoSequence instance;
    return &instance;                                                            
}       

void AutoSequence::grabAndLiftPizza(){
    Grabber::getInstance()->nextPos();
    sleepMs(350);
    Lift::getInstance()->updateLevel();
}

void AutoSequence::scorePizza(){
    Pusher::getInstance()->nextPos();
    Grabber::getInstance()->nextPos();
    sleepMs(550);
    Grabber::getInstance()->prevPos();
    sleepMs(300);
    Pusher::getInstance()->prevPos();
}

class Joystick{
    private:
        Joystick();
    public:
        //input scales for all axies
        float lXScale;
        float lYScale;
        float rXScale;
        float rYScale;
        static Joystick* getInstance();
        float lY();
        float rY();
};

Joystick::Joystick()
{
    lXScale = 1;
    lYScale = 1;
    rXScale = 1;
    rYScale = 1;
}

Joystick* Joystick::getInstance(){                                                                                
    static Joystick instance;
    return &instance;                                                            
} 
float Joystick::lY(){                                                                                
    return lYScale * con.Axis3.position(percentUnits::pct);                                              
}       

float Joystick::rY(){
    return rYScale * con.Axis2.position(percentUnits::pct);                                              
}       

/**
 * This section contains all the threads for button and switch detection
 * It uses very similar concepts to event driven programming so I hope
 * it counts for that. Most of these are trigged when button is relased
 * while some have actions for both.
 **/

void btnR1 () {
    static bool lastPressed = con.ButtonR1.pressing();
    while (true) {
        if (con.ButtonR1.pressing()) lastPressed = true;
        else if (!con.ButtonR1.pressing() && lastPressed) {
            lastPressed = false;
            Lift::getInstance()->nextPos();
        }
        this_thread::yield();
    }
}
void btnL1 () {
    static bool lastPressed = con.ButtonL1.pressing();
    while (true) {
        if (con.ButtonL1.pressing()) lastPressed = true;
        else if (!con.ButtonL1.pressing() && lastPressed) {
            Grabber::getInstance()->prevPos();
        }
        this_thread::yield();
    }
}
void btnR2 () {
    static bool lastPressed = con.ButtonR2.pressing();
    while (true) {
        if (con.ButtonR2.pressing()) lastPressed = true;
        else if (!con.ButtonR2.pressing() && lastPressed) {
            lastPressed = false;
            Lift::getInstance()->prevPos();
        }
        this_thread::yield();
    }
}
void btnL2 () {
    static bool lastPressed = con.ButtonL2.pressing();
    while (true) {
        if (con.ButtonL2.pressing()) lastPressed = true;
        else if (!con.ButtonL2.pressing() && lastPressed) {
            lastPressed = false;
            Grabber::getInstance()->nextPos();
        }
        this_thread::yield();
    }
}

void btnUP () {
    static bool lastPressed = con.ButtonUp.pressing();
    while (true) {
        if (con.ButtonUp.pressing()) lastPressed = true;
        else if (!con.ButtonUp.pressing() && lastPressed) {
            lastPressed = false;
            Pusher::getInstance()->nextPos();
        }
        this_thread::yield();
    }
}

void btnDN () {
    static bool lastPressed = con.ButtonDown.pressing();
    while (true) {
        if (con.ButtonDown.pressing()) lastPressed = true;
        else if (!con.ButtonDown.pressing() && lastPressed) {
            lastPressed = false;
            Pusher::getInstance()->prevPos();
        }
        this_thread::yield();
    }
}

void btnA () {
    static bool lastPressed = con.ButtonA.pressing();
    while (true) {
        if (con.ButtonA.pressing()) lastPressed = true;
        else if (!con.ButtonA.pressing() && lastPressed) {
            lastPressed = false;
            AutoSequence::getInstance()->grabAndLiftPizza();
        }
        this_thread::yield();
    }
}

void btnB () {
    static bool lastPressed = con.ButtonB.pressing();
    while (true) {
        if (con.ButtonB.pressing()) lastPressed = true;
        else if (!con.ButtonB.pressing() && lastPressed) {
            lastPressed = false;
            AutoSequence::getInstance()->scorePizza();
        }
        this_thread::yield();
    }
}

void limitLift () {
    static bool lastPressed = con.ButtonDown.pressing();
    while (true) {
        if (con.ButtonDown.pressing()) lastPressed = true;
        else if (!con.ButtonDown.pressing() && lastPressed) {
            lastPressed = false;
            Lift::getInstance()->resetRotation();
        }
        this_thread::yield();
    }
}

void limitGrabber () {
    static bool lastPressed = con.ButtonDown.pressing();
    while (true) {
        if (con.ButtonDown.pressing()) lastPressed = true;
        else if (!con.ButtonDown.pressing() && lastPressed) {
            lastPressed = false;
            AutoSequence::getInstance()->grabAndLiftPizza();
        }
        this_thread::yield();
    }
}

void sonarTrigger() {
    static bool lastTriggered = false;
    while (true) {
        int sonarValue = mainSonar.distance(vex::distanceUnits::cm);
        if (sonarValue > SONARDISTANCE && !lastTriggered){
            lastTriggered = true;
            con.rumble("*-*-*");
        } 
        else if (sonarValue <= SONARDISTANCE && lastTriggered) {
            lastTriggered = false;
            con.rumble("***");
        }
        this_thread::yield();
    }
}

//starts all the threads to handle events
void startThreads(){
    thread R1 = thread(btnR1);
    thread R2 = thread(btnR2);
    thread L1 = thread(btnL1);
    thread L2 = thread(btnL2);
    thread UP = thread(btnUP);
    thread A = thread(btnA);
    thread B = thread(btnB);
    thread DOWN = thread(btnDN);
    thread sona = thread(sonarTrigger);
    thread limit1 = thread(limitGrabber);
    thread limit2 = thread(limitLift);
}


void pre_auton() {
    SetUpScreen setup = SetUpScreen();
    setup.displayMain();
}

void autonomous() {
    
    //do not move unless user selected right mode
    while(robotMode !=  AUTO  && robotMode != COMP){
        this_thread::yield();
    }
    switch(autonMode){
        case RED1: //go over bump
            Brain.Screen.print("RED1");
            DriveTrain::getInstance()->setDrivePower(100,100);
            sleepMs(6000);
            DriveTrain::getInstance()->stop();
            break;
        case RED2: //user loads a pizza and scores on top floor
            Brain.Screen.print("RED2");
            Lift::getInstance()->nextPos();
            Lift::getInstance()->nextPos();
            Grabber::getInstance()->nextPos();
            sleepMs(1500);
            Grabber::getInstance()->prevPos();
            DriveTrain::getInstance()->turnLeft(TURN_90*2);
            DriveTrain::getInstance()->followLine();
            DriveTrain::getInstance()->turnLeft(TURN_90);
            DriveTrain::getInstance()->forwardsUntillWall();
            Lift::getInstance()->nextPos();
            sleepMs(1500);
            AutoSequence::getInstance()->scorePizza();
            break;
        case BLUE1: //over bump
            Brain.Screen.print("BLUE1");
            Brain.Screen.print("RED1");
            DriveTrain::getInstance()->setDrivePower(100,100);
            sleepMs(6000);
            DriveTrain::getInstance()->stop();
            break;
        case BLUE2: //user loads a pizza and scores on top floor
            Brain.Screen.print("BLUE2");
            Lift::getInstance()->nextPos();
            Lift::getInstance()->nextPos();
            Grabber::getInstance()->nextPos();
            sleepMs(1500);
            Grabber::getInstance()->prevPos();
            DriveTrain::getInstance()->turnLeft(TURN_90*-2);
            DriveTrain::getInstance()->followLine();
            DriveTrain::getInstance()->turnLeft(TURN_90*-1);
            DriveTrain::getInstance()->forwardsUntillWall();
            Lift::getInstance()->nextPos();
            sleepMs(1500);
            AutoSequence::getInstance()->scorePizza();
            break;
        default: Brain.Screen.print("Error!"); break; //no auton
    }
}

void drivercontrol() {
    DriveTrain* drive = DriveTrain::getInstance();
    Joystick *joy = Joystick::Joystick::getInstance();
    joy->lYScale = 0.5;
    joy->rYScale = 0.5;
    
    startThreads();

    //do not move unless user selects right mode through screen
    while(robotMode !=  DRIVE  && robotMode != COMP){
        this_thread::yield();
    }
    
    while (true) {
        drive->setDrivePower(joy->lY(),joy->rY());
        this_thread::yield();
    }
}

int main() {
    // Do not adjust the lines below
    
    // Set up (but don't start) callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(drivercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Robot Mesh Studio runtime continues to run until all threads and
    // competition callbacks are finished.
}
