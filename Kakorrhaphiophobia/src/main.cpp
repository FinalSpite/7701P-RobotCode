/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       njwolff                                                   */
/*    Created:      12/7/2024, 10:05:06 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <string.h>

using namespace vex;
using signature = vision::signature;
using code = vision::code;
// A global instance of competition
competition Competition;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

bool running = true;

int autonchoice = 0;
//when auton = 0 is left side code when = 1 its right side code and when =2 its practice driving.

// define your global instances of motors and other devices here
// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT2, ratio6_1, false);
motor leftMotorB = motor(PORT1, ratio6_1, true);
motor leftMotorC = motor(PORT3, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT11, ratio6_1, false);
motor rightMotorB = motor(PORT13, ratio6_1, true);
motor rightMotorC = motor(PORT12, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial DrivetrainInertial = inertial(PORT7);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 219.44, 320, 40, mm, 0.6666666666666666);
digital_out Sol1 = digital_out(Brain.ThreeWirePort.B);
motor pickupmotor = motor(PORT10, ratio18_1, false);
digital_out Sol2 = digital_out(Brain.ThreeWirePort.D);


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
  
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  Sol1.set(false);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  wait(1000, msec);
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.clearScreen();
  running = false;
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

void autonomous_right(void) {
  Brain.Screen.print("AutonRight");
  wait(1000, msec);
  turn_to_angle(3600);
}
void autonomous_left(void){
  Brain.Screen.print("AutonLeft");
  wait(1000, msec);
  turn_to_angle(3600);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/ 

void user(void) {
  // User control code here, inside the loop
    Brain.Screen.setFillColor(black);
    Brain.Screen.print("User Code");
    thread rc_auto_loop_controller(rc_auto_loop_function_Controller1);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.

  // Run the pre-autonomous function.
  pre_auton();

  run();
  
  Brain.Screen.setFillColor(black);

  if (autonchoice == 0){
    Competition.autonomous(autonomous_left);
    Competition.drivercontrol(user);
  } else if (autonchoice == 1){
    Competition.autonomous(autonomous_right);
    Competition.drivercontrol(user);
  } else if (autonchoice == 2){
    user();
  } else if(autonchoice == 3){
    wait(1500,msec);
    autonomous_right();
  }

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
