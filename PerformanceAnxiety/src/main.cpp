/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       njwolff                                                   */
/*    Created:      12/7/2024, 10:07:47 PM                                    */
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


// define your global instances of motors and other devices here
// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT2, ratio6_1, false);
motor leftMotorB = motor(PORT1, ratio6_1, true);
motor leftMotorC = motor(PORT7, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT11, ratio6_1, false);
motor rightMotorB = motor(PORT12, ratio6_1, true);
motor rightMotorC = motor(PORT14, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial DrivetrainInertial = inertial(PORT16);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 219.44, 320, 40, mm, 0.6666666666666666);
digital_out Sol1 = digital_out(Brain.ThreeWirePort.B);
motor pickupmotor = motor(PORT10, ratio18_1, false);
motor intakemotor = motor(PORT9, ratio6_1, false);
motor_group pickup = motor_group(pickupmotor, intakemotor);


bool RemoteControlCodeEnabled = true;

// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;


//These booleans are used so only one button has to be pressed to do an action
//This is because button presses are detected many times in a second, so it would just go on and off without.
bool solenoid_toggle = true;
bool solenoid_last_toggle = false;
bool solenoid_toggle2 = true;
bool solenoid_last_toggle2 = false;
bool conveyer_Toggle = true;
bool conveyer_last_toggle = false;
bool conveyer_Toggle1 = true;
bool conveyer_last_toggle1 = false;




float Kp = 0.5;   // Proportional gain for PID
float Ki = 0.0;   // Integral gain for PID
float Kd = 0.1;   // Derivative gain for PID


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  Perform some actions before the competition starts.      */
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


/// PID definition
void turn_to_angle(float targetAngle) {
    float error;            // Difference between target and current angle
    float previousError = 0;
    float integral = 0;
    float derivative;
    float motorPower;

    while (true) {
      
        // Calculate the current error
        error = targetAngle - DrivetrainInertial.rotation();
        
        // Proportional term
        float P = Kp * error;
        
        // Integral term
        integral += error;
        float I = Ki * integral;
        
        // Derivative term
        derivative = error - previousError;
        float D = Kd * derivative;
        
        float KL = 1.2;

        // PID output
        motorPower = P + I + D;

        // Set motor powers (adjust this for left and right turns)
        LeftDriveSmart.spin(directionType::fwd, motorPower, velocityUnits::pct);
        RightDriveSmart.spin(directionType::rev, motorPower, velocityUnits::pct);
        

        // Update previous error
        previousError = error;
        
        // Break loop if error is within a small threshold
          if (fabs(error) < KL) {
            break;
          }
          
        
        // Small delay to avoid overloading the CPU
        task::sleep(20);
    }
    
    // Stop motors once the target angle is reached
    LeftDriveSmart.stop();
    RightDriveSmart.stop();
}


/*------------------------------------------------*/
/*                                                */
/*               UserDrive Code                   */
/*                    ||||                        */
/*                                                */
/*---------------------\/-------------------------*/


int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  pickupmotor.stop();
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 3.5 && drivetrainLeftSideSpeed > -3.5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 3 && drivetrainRightSideSpeed > -3) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity((drivetrainLeftSideSpeed-2), percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }


      //When the controller button r1 is pressed the pickup belt will move in the direction for pickup
      if ((Controller1.ButtonL1.pressing() == true)){
        if(conveyer_last_toggle == false){
          if (conveyer_Toggle == true)
          {
            pickup.spin(reverse, 100, percent);
            conveyer_Toggle = false;
          }
          else if (conveyer_Toggle == false){
            pickup.stop();
            conveyer_Toggle = true;
          }
        conveyer_last_toggle = true;
        wait(10, msec);
        }
      }
      if ((Controller1.ButtonL1.pressing() == false)){
        conveyer_last_toggle = false;
        wait(10, msec);
      }




      //When the controller button L1 is pressed then the pickup belt spins in the opposite direction
      //just in case something gets stuck or for further upgrades in the future.
      if ((Controller1.ButtonR1.pressing() == true)){
        if(conveyer_last_toggle1 == false){
          if (conveyer_Toggle1 == true)
          {
            pickup.spin(forward, 100, percent);
            conveyer_Toggle1 = false;
          }
          else if (conveyer_Toggle1 == false){
            pickup.stop();
            conveyer_Toggle1 = true;
          }
        conveyer_last_toggle1 = true;
        }
        wait(10, msec);
      }
      if ((Controller1.ButtonR1.pressing() == false)){
        conveyer_last_toggle1 = false;
        wait(10, msec);
      }


      // Sets the grabber down in the back of the robot
      if ((Controller1.ButtonA.pressing() == true)){
        if (solenoid_last_toggle == false){
          if (solenoid_toggle == true){
            Sol1.set(true); 
            solenoid_toggle = false;
          }
          else if (solenoid_toggle == false)
          {
            Sol1.set(false);
            solenoid_toggle = true;
          } 
          solenoid_last_toggle = true;
        }
      }
      if (Controller1.ButtonA.pressing() == false){
        solenoid_last_toggle = false;
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}
/*----------------------/\------------------------*/
/*                                                */
/*               UserDrive Code                   */
/*                    ||||                        */
/*                                                */
/*------------------------------------------------*/



void user_control(void){
  // User control code here, inside the loop
    Brain.Screen.setFillColor(black);
    Brain.Screen.print("User Code");
    thread rc_auto_loop_controller(rc_auto_loop_function_Controller1);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}

void autonomous(void) {
  wait(1000, msec);
  turn_to_angle(3600);
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(user_control);
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(50, msec);
  }
}
