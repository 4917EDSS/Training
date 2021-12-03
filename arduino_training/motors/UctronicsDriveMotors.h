#include "UCMotor.h"

class UctronicsDriveMotors {
 public:
  UctronicsDriveMotors();
  // Power is 0..100 for forward and 0..-100 for reverse
  void drive(int leftPower, int rightPower);
  
 private:
  UC_DCMotor leftMotor;
  UC_DCMotor rightMotor;
};
