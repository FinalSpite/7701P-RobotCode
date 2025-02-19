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

//since the pre-auton function is treaded we need to make sure that it is done before the buttons are placed on the screen
bool running = true;

//if the auton is on practice mode it will wait a second before running.
bool practice = false;

int autonchoice = 0;
//when auton = 0 is left side code when = 1 its right side code and when =2 its practice driving.

// define your global instances of motors and other devices here
// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT2, ratio6_1, false);
motor leftMotorB = motor(PORT1, ratio6_1, true);
motor leftMotorC = motor(PORT7, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT13, ratio6_1, false);
motor rightMotorB = motor(PORT12, ratio6_1, true);
motor rightMotorC = motor(PORT14, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial DrivetrainInertial = inertial(PORT11);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 219.44, 320, 40, mm, 0.6666666666666666);
digital_out Sol1 = digital_out(Brain.ThreeWirePort.B);
motor pickupmotor = motor(PORT10, ratio18_1, false);
motor intakemotor = motor(PORT9, ratio6_1, false);
motor_group pickup = motor_group(pickupmotor, intakemotor);
motor lady_brown = motor(PORT19, ratio18_1, true);





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
  lady_brown.setStopping(hold);
  lady_brown.setVelocity(80, percent);
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  Sol1.set(false);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  lady_brown.resetPosition();
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  wait(250, msec);
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.clearScreen();
  running = false;
  lady_brown.spinToPosition(60, degrees);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control the robot during the autonomous phase of    */
/*  the VEX Competition.                                                     */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/ 

// Code for left side autonomous (basic)
void autonomous_left(void){
  if(practice == true){
    wait(1500, msec);
  }
  
  Brain.Screen.clearScreen();
  Brain.Screen.print("AutonLeft");
  Drivetrain.drive(reverse);
  wait(1450, msec);
  Drivetrain.stop();
  Sol1.set(true);
  wait(250, msec);
  pickup.spin(forward, 100, percent);
  wait(1300, msec);
  turn_to_angle(65.0);
  wait(500, msec);
  Drivetrain.setDriveVelocity(34, percent);
  Drivetrain.drive(forward);
  wait(725, msec);
  Drivetrain.stop();
  wait(400, msec);
  DrivetrainInertial.resetRotation();
  turn_to_angle(180.0);
  wait(250, msec);
  Drivetrain.drive(forward);
  wait(2000, msec);
  Drivetrain.stop();
}
// Code for right side autonomous (basic)
void autonomous_right(void) {
  if(practice == true){
    wait(1500, msec);
  }

  Brain.Screen.clearScreen();
  Brain.Screen.print("AutonRight");
  Drivetrain.drive(reverse);
  wait(1400, msec);
  Drivetrain.stop();
  Sol1.set(true);
  wait(250, msec);
  pickup.spin(forward, 100, percent);
  wait(1300, msec);
  turn_to_angle(315.0);
  wait(500, msec);
  Drivetrain.setDriveVelocity(34, percent);
  Drivetrain.drive(forward);
  wait(725, msec);
  Drivetrain.stop();
  wait(400, msec);
  DrivetrainInertial.resetRotation();
  turn_to_angle(180.0);
  wait(100, msec);
  Drivetrain.drive(forward);
  wait(2000, msec);
  Drivetrain.stop();
}

void autonomous_start( void ){
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(0,0,230,272);
  Brain.Screen.printAt(10,45,"LEFT");

  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(250,0,230,272);
  Brain.Screen.printAt(260,45, "RIGHT");


  while(true){
    if (Brain.Screen.pressing() == true){
      int x = Brain.Screen.xPosition();
      
      if ((x<=230)){
        practice = true;
        autonomous_left();
        break;
      }else if((x>=250)){
        practice = true;
        autonomous_right();
        break;
      }
    }
  }
} 

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*                                                                           */
/*      The Code For the driver control is the in the robot-config file      */
/*           under the function rc_auto_loop_function_Controller1            */
/*-------------------------------------\/------------------------------------*/ 

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

 // Set up callbacks for autonomous and driver control periods based on button presses on the run screen.
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
    autonomous_start();
  }

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
