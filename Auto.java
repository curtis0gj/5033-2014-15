package org.usfirst.frc.team5033.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

public class Auto {
	Robot robot;
	RobotDrive chassis;
	DigitalInput limit;
	DigitalInput limit2;
	DigitalInput limit3;
	Gyro gyro;
	Encoder encoder;
	Relay leftArmWheel;
	Relay rightArmWheel;
	Victor screwMotor1;
	Victor screwMotor2;
	Victor armMotor;
	public Auto(Robot r){
		this.robot = r;
		this.chassis = r.chassis;
		this.gyro = r.gyro;
		this.encoder = r.encoder;
		this.limit = r.limit;
		this.limit2 = r.limit2;
		this.limit3 = r.limit3;
		this.leftArmWheel = r.leftArmWheel;
		this.rightArmWheel = r.rightArmWheel;
		this.screwMotor1 = r.screwMotor1;
		this.screwMotor2 = r.screwMotor2;
		this.armMotor = r.armMotor;
	}
	private void wait(double waitTime) {
		Timer.delay(waitTime); 
	}
	private void reset() {
		chassis.arcadeDrive(0, 0);
		gyro.reset();
		encoder.reset();
		screwMotor1.set(Defines.SCREW_OFF);
		screwMotor2.set(Defines.SCREW_OFF);
	}
	private void angleError(double setpointDegressZeroToThreeSixty, double experimentalDegrees) {
		double err = setpointDegressZeroToThreeSixty - experimentalDegrees; // 0 TO 360!
		if(err < -180) {
			err += 360;
		} else if(err > 180) {
			err -= 360;
		}
		return err;
	}
	double kp_rotate = 0.01;
	double MAX_ERROR = 5;
	private void turn(double degree) {
		reset();
		while(true) {
			double deltaAngle = angleError(deg, gyro.getAngle());
			if(Math.abs(degree - deltaAngle) < MAX_ERROR) {
				break;
			} else {
				chassis.arcadeDrive(0, deltaAngle * kp_rotate);
			}
			Timer.delay(0.02);
		}
	}
	double kP = 0.035;
	private void forwardDrive(double distanceToGo) {
		reset();
		while (true) {
			double distance = encoder.get();
			double angle = gyro.getAngle();
			if (!isAutonomous() || !isEnabled()) return;
			chassis.arcadeDrive(-0.25, angle * kP);
			//chassis.drive(-0.40, 0);
			if (distance < -distanceToGo) {
				reset();
				return;
			}
			Timer.delay(0.02);
		}
	}
	private void liftBin(double second) {
		double screwTime = Timer.getFPGATimestamp();
		while (true) {
			if (!isAutonomous() || !isEnabled()) return;
			if (screwTime + second > Timer.getFPGATimestamp()) {
				screwMotor1.set(Defines.SCREW_SPEED);
				screwMotor2.set(Defines.SCREW_SPEED);
			} else {
				reset();
				break;
			}
			Timer.delay(0.02);
		}
	}
	private void closeArms(double second) {
		double armTime = Timer.getFPGATimestamp();
		while (true) {
			if (!isAutonomous() || !isEnabled()) return;
			boolean maxArmLimit = limit3.get();
			if (armTime + second > Timer.getFPGATimestamp()) {
				armMotor.set(Defines.ARM_SPEED);
				leftArmWheel.set(Relay.Value.kForward);
				rightArmWheel.set(Relay.Value.kReverse);
			} else if (maxArmLimit == true) {
				armMotor.set(Defines.ARM_OFF);
				leftArmWheel.set(Relay.Value.kOff);
				rightArmWheel.set(Relay.Value.kOff);
				reset();
				break;
			} else {
				reset();
				break;
			}
			Timer.delay(0.02);
		}
	}
	public void run(Autos autoMode) {
		switch(autoMode) {
			case AUTO_MOVE_TO_ZONE:
				forwardDrive(3000);
				break;
			case AUTO_GRAB_ONE_BIN_RED_SIDE:
				liftBin(3.5);
				wait(0.5);
				turn(90); //COULD 90 OR 180!
				wait(0.5);
				forwardDrive(2000);
				break;
			case AUTO_GRAB_ONE_BIN_BLUE_SIDE:
				liftBin(3.5);
				wait(0.5);
				turn(180);  //COULD 90 OR 180!
				wait(0.5);
				forwardDrive(2000);
				break;
			case AUTO_GRAB_TWO_BINS_RED_SIDE:
				liftBin(3.5);
				wait(0.5);
				forwardDrive(450);
				wait(0.5);
				closeArms(2);
				wait(0.5);
				turn(90);  //COULD BE 90 OR 180!
				wait(0.5);
				forwardDrive(2000);
				break;
			case AUTO_GRAB_TWO_BINS_BLUE_SIDE:
				liftBin(3.5);
				wait(0.5);
				forwardDrive(450);
				wait(0.5);
				closeArms(2);
				wait(0.5);
				turn(180); //COULD BE 90 OR 180!
				wait(0.5);
				forwardDrive(2000);
				break;
		}
	}
}
