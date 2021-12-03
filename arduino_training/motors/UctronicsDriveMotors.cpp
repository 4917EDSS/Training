#include "UctronicsDriveMotors.h"

UctronicsDriveMotors::UctronicsDriveMotors() :
  leftMotor(3, MOTOR34_64KHZ),
  rightMotor(4, MOTOR34_64KHZ) {
  
}

void UctronicsDriveMotors::drive(int leftPower, int rightPower) {
  // Need to set the mode of the motors before setting the power
  if( (leftPower > 0) && (rightPower > 0) ) {
    leftMotor.run(FORWARD);
    rightMotor.run(FORWARD);
  }
  else if( (leftPower < 0) && (rightPower < 0) ) {
    leftMotor.run(BACKWARD);
    rightMotor.run(BACKWARD);
  }
  else if( (leftPower > 0) && (rightPower < 0) ) {
    leftMotor.run(RIGHT);
    rightMotor.run(RIGHT);
  }
  else if( (leftPower < 0) && (rightPower > 0) ) {
    leftMotor.run(LEFT);
    rightMotor.run(LEFT);
  }
  else {
    leftMotor.run(STOP);
    rightMotor.run(STOP);
  }

  // Now set the absolute power (we took care of the sign/direction above)
  if(leftPower < 0) {
    leftPower = -leftPower;
  }
  if(rightPower < 0) {
    rightPower = -rightPower;
  }
  leftMotor.setSpeed(leftPower*255/100);
  rightMotor.setSpeed(rightPower*255/100);
}
