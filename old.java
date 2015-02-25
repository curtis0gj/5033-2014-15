public void turn(double degree) {
  reset();
  while(true) {
  if(!isAutonomous() || !isEnabled()); return;
  double angle = gyro.getAngle();
  
  chassis.arcadeDrive(-0.25, -1);
  
  if(angle > degree) {
    chassis.arcadeDrive(0, 0);
  }
  Timer.delay(0.02);
  }
