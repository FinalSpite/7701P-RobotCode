#include "vex.h"
#include <string.h>
#include <iostream>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

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
            pickupmotor.spin(reverse, 100, percent);
            intakemotor.spin(forward, 100, percent);
            conveyer_Toggle = false;
          }
          else if (conveyer_Toggle == false){
            pickupmotor.stop();
            intakemotor.stop();
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
            pickupmotor.spin(forward, 100, percent);
            intakemotor.spin(reverse,100, percent);
            conveyer_Toggle1 = false;
          }
          else if (conveyer_Toggle1 == false){
            pickupmotor.stop();
            intakemotor.stop();
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


      //For future use of a second solenoid
      if ((Controller1.ButtonB.pressing() == true)){
        if (solenoid_last_toggle2 == false){
          if (solenoid_toggle2 == true){
            Sol2.set(true); 
            solenoid_toggle2 = false;
          }
          else if (solenoid_toggle2 == false)
          {
            Sol2.set(false);
            solenoid_toggle2 = true;
          } 
          solenoid_last_toggle2 = true;
        }
      }
      if (Controller1.ButtonB.pressing() == false){
        solenoid_last_toggle2 = false;
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


/// PID code
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
/*           Buttons on the Screeen               */
/*                    ||||                        */
/*                                                */
/*---------------------\/-------------------------*/


void run( void ) {
  if(running != true){
  Drivetrain.setStopping(coast);
  RightDriveSmart.setVelocity(30, percentUnits::pct);
  LeftDriveSmart.setVelocity(30, percentUnits::pct);
  wait(500,msec);
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(0,0,230,140);
    Brain.Screen.setFont(prop30);
    Brain.Screen.printAt(30,77,"LEFT AUTON");
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(250,0,230, 140);
    Brain.Screen.printAt(276,77,"RIGHT AUTON");
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0,150, 230,110);
    Brain.Screen.printAt(40, 200, "USER CODE");
    Brain.Screen.setFillColor(orange);
    Brain.Screen.drawRectangle(250,150, 230, 110);
    Brain.Screen.printAt(270, 200, "AUTON PRACT");
  }
  while (1)
  {
    if (Brain.Screen.pressing()){
      int x = Brain.Screen.xPosition();
      int y = Brain.Screen.yPosition();

      if ((x<=230)&&(y<=140)){
        autonchoice = 0;
        break;
      }else if((y<=140)&&(x>=250)){
        autonchoice = 1;
        break;
      }else if((y>=150)&&(x<=230)){
        autonchoice = 2;
        break;
      }else if((y>=150)&&(x>=250)){
        autonchoice = 3;
        break;
      }
    }
  }
  Brain.Screen.clearScreen();

  wait(20, msec);
}