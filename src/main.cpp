/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Joel D.                                                   */
/*    Created:      10/24/2024, 11:18:06 PM                                   */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "motors.h"
#include "PID.h"
#include "turnPID.h"

using namespace vex;
//using namespace _PID;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

PID pid;
turnPID turnPid;
int autonNum;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  Left.resetPosition();
  Right.resetPosition();
  InertialSensor.calibrate();
  InertialSensor.resetHeading();

  //Set auton number for each time you upload the program
  autonNum = 1;
}

void spin_For(double time, motor Motor) {
    double timePassed = 0;
    while (timePassed < time) {
      Motor.spin(forward, 80, pct);
      timePassed += time / 1000;
      wait(time, msec);
    }
    Motor.stop(brake);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  clampPneumatics.set(false);

  /*InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
      wait(10, vex::msec);
  }
  InertialSensor.resetHeading();*/

  wait(3000, msec);
  Brain.Screen.print(autonNum);
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  if (autonNum == 1) {
    // Red left

    pid.runPID(36, 2);
    turnPid.runTurnPID(180);
    clampPneumatics.set(true);
    spin_For(2, belt);
    turnPid.runTurnPID(-90/*.690067526*/);
    pid.runPID(24, 2);
    turnPid.runTurnPID(90);
    pid.runPID(24, 2);
    spin_For(2, belt);
    turnPid.runTurnPID(135);
    pid.runPID(24 * sqrt(2), 2);
    // pid.runPID(2 * sqrt(10), 2);
    Brain.Screen.print("Auton 1 (Red Left) completed.");

  } else if (autonNum == 2) {
    // Red right
    // turnPid.runTurnPID(33.690067526);
    // pid.runPID(43.2/2);
    // belt.spinFor(2, sec);
    
    pid.runPID(36, 2);
    turnPid.runTurnPID(180);
    clampPneumatics.set(true);
    spin_For(2, belt);
    turnPid.runTurnPID(90/*.690067526*/);
    pid.runPID(24, 2);
    turnPid.runTurnPID(-90);
    pid.runPID(24, 2);
    spin_For(2, belt);
    turnPid.runTurnPID(-135);
    pid.runPID(24 * sqrt(2), 2);
    Brain.Screen.print("Auton 2 (Red Right) completed.");
    

  } if (autonNum == 3) {
    // Blue left
    // turnPid.runTurnPID(-33.690067526);
    // pid.runPID(43.2);
    
    pid.runPID(36, 2);
    turnPid.runTurnPID(180);
    clampPneumatics.set(true);
    spin_For(2, belt);
    turnPid.runTurnPID(-90/*.690067526*/);
    pid.runPID(24, 2);
    turnPid.runTurnPID(90);
    pid.runPID(24, 2);
    spin_For(2, belt);
    turnPid.runTurnPID(135);
    pid.runPID(24 * sqrt(2), 2);

    Brain.Screen.print("Auton 3 (Blue Left) completed.");

  } else if (autonNum == 4) {
    // Blue right
    // turnPid.runTurnPID(33.690067526);
    // pid.runPID(43.2/2);
    // belt.spinFor(2, sec);
    pid.runPID(36, 2);
    turnPid.runTurnPID(180);
    clampPneumatics.set(true);
    spin_For(2, belt);
    turnPid.runTurnPID(90/*.690067526*/);
    pid.runPID(24, 2);
    turnPid.runTurnPID(-90);
    pid.runPID(24, 2);
    spin_For(2, belt);
    turnPid.runTurnPID(-135);
    pid.runPID(24 * sqrt(2), 2);

    // turnPid.runTurnPID(153.4349488);
    // pid.runPID(2 * sqrt(10));
    Brain.Screen.print("Auton 4 (Blue Right) completed.");
  } else if (autonNum == 10) {
    pid.runPID(-6, 2);
    clampPneumatics.set(true);
    pid.runPID(-6, 2);
    turnPid.runTurnPID(180);
    pid.runPID(24, 2);
    // belt.spinFor(2,sec);
    spin_For(2, belt);
    turnPid.runTurnPID(-90);
    pid.runPID(24, 2);
    turnPid.runTurnPID(0);


    //TEST THIS TO SEE IF IT DOES THEM AT THE SAME TIME
    pid.runPID(36, 2);
    spin_For(10, belt);


    // belt.spinFor(10,sec);
    pid.runPID(-12, 2);
    turnPid.runTurnPID(-90);
    pid.runPID(12, 2);
    turnPid.runTurnPID(180+26.5650511771);
    pid.runPID(-12, 2);
    clampPneumatics.set(false);
  } else if (autonNum == 5) {
    // wait(4, sec);
    // Left.spinFor(1, sec);
    // Right.spinFor(1, sec);
    pid.runPID(3, 2);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                             */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

//in main.cpp
void usercontrol(void) {
  // User control code here, inside the loop
  // simga 🗣️🗣️🔥🔥

  double slow = 1;
  bool pneumaticsBool = false;
  bool pressingBool = false;
  int distance = 0;
  while (1) {

    //PID test controls
    if (Controller1.ButtonDown.pressing()) {
      pid.stopPID();
    }
    
    if (!Controller1.ButtonX.pressing() && !Controller1.ButtonY.pressing()) {
      if (Controller1.ButtonB.pressing()) {
        printToConsole("PID running");
        pid.runPID(distance, 10);
        wait(5, sec);
      } else if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()) {
        distance = 0;
        printToConsole(distance);
        wait(0.1, sec);
      }  else if (Controller1.ButtonRight.pressing()) {
        distance++;
        printToConsole(distance);
        wait(0.1, sec);
      } else if (Controller1.ButtonLeft.pressing()) {
        distance--;
        printToConsole(distance);
        wait(0.1, sec);
      }
    } else if (Controller1.ButtonX.pressing()){
      if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()) {
        pid.kpUpdate(0-pid.kpUpdate(0));
        pid.kpUpdate(0.5);
        printToConsole(pid.kpUpdate(0));
        wait(0.1, sec);
      }  else if (Controller1.ButtonRight.pressing()) {
        pid.kpUpdate(0.1);
        printToConsole(pid.kpUpdate(0));
        wait(0.1, sec);
      } else if (Controller1.ButtonLeft.pressing()) {
        pid.kpUpdate(-0.1);
        printToConsole(pid.kpUpdate(0));
        wait(0.1, sec);
      }
    } else if (Controller1.ButtonY.pressing()){
      if (Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()) {
        pid.kdUpdate(0-pid.kdUpdate(0));
        pid.kdUpdate(0.1);
        printToConsole(pid.kdUpdate(0));
        wait(0.1, sec);
      }  else if (Controller1.ButtonRight.pressing()) {
        pid.kdUpdate(0.1);
        wait(0.01, sec);
        printToConsole(pid.kdUpdate(0));
        wait(0.1, sec);
      } else if (Controller1.ButtonLeft.pressing()) {
        pid.kdUpdate(-0.1);
        wait(0.01, sec);
        printToConsole(pid.kdUpdate(0));
        wait(0.1, sec);
      }
    }
  
    // printToConsole(((fabs(Left.position(vex::turns)) + fabs(Right.position(vex::turns))) / 2.0));

    if (Controller1.ButtonUp.pressing()){
      slow = 0.4;
    } else {
      slow = 1;
    }

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based  on feedback from the joysticks.
    Left.spin(forward, (Controller1.Axis3.position()+ Controller1.Axis1.position()) * slow * 0.8, pct);
    Right.spin(forward, (Controller1.Axis3.position() - Controller1.Axis1.position()) * slow * 0.8, pct);

    /*Brain.screen.print(Controller1.Axis3.position());
    Brain.screen.print(Controller1.Axis3.position() + Controller1.Axis1.position(), pct);
    Brain.screen.clear();*/ 
    //Ml.spin(forward, Controller1.Axis3.position() + Controller1.Axis1.position(), pct);
    //Mr.spin(forward, Controller1.Axis3.position() - Controller1.Axis1.position(), pct);
    //Bl.spin(forward, Controller1.Axis3.position() + Controller1.Axis1.position(), pct);
    //Br.spin(forward, Controller1.Axis3.position() - Controller1.Axis1.position(), pct);
    
    

    if (Controller1.ButtonDown.pressing()) {
      //from the screen
      Left.stop(hold);
      Right.stop(hold);
    } else {
      //to the ring
      Left.setStopping(coast);
      Right.setStopping(coast);
    }

    if (Controller1.ButtonR1.pressing()) {
      //to the pen
      belt.spin(forward, 100, pct);
    } else if (Controller1.ButtonR2.pressing()) {
      //to the king
      belt.spin(reverse, 100, pct);
    } else {
      //where's my crown
      belt.stop(brake);
    }
    
    if (Controller1.ButtonA.pressing()) {
      //that's my bling
      if (pressingBool == false) {
        pneumaticsBool = !pneumaticsBool;
        clampPneumatics.set(pneumaticsBool);
      }
      pressingBool = true;
    } else {
      //always drama when I ring
      pressingBool = false;
    }

    if (Controller1.ButtonL1.pressing()) {
      ladyBrown.spin(forward, 100, pct);
    } else if (Controller1.ButtonL2.pressing()) {
      ladyBrown.spin(reverse, 100, pct);
    } else {
      ladyBrown.stop(brake);
    }
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
