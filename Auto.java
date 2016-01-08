package org.usfirst.frc.team5033.robot;

import org.usfirst.frc.team5033.robot.Defines.Autos;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
		/*this.limit2 = r.limit2;
		this.limit3 = r.limit3;*/
		this.leftArmWheel = r.leftArmWheel;
		this.rightArmWheel = r.rightArmWheel;
		this.screwMotor1 = r.screwMotor1;
		this.screwMotor2 = r.screwMotor2;
		this.armMotor = r.armMotor;
	}
	private void reset() {
		chassis.arcadeDrive(0, 0);
		gyro.reset();
		encoder.reset();
		screwMotor1.set(Defines.SCREW_OFF);
		screwMotor2.set(Defines.SCREW_OFF);
	}
	
	private void basicTurn(double desiredAngle) {
		reset();
		while(true) {
			if (!robot.isAutonomous() || !robot.isEnabled()) return;
			double angle = gyro.getAngle();
			SmartDashboard.putNumber("TurningAngle", angle);
			chassis.arcadeDrive(0, -.7);
			if(angle > desiredAngle) {
				reset();
				break;
			}
		}
	}
	
	double kP = 0.035;
	private void forwardDriveWithGyro(double distanceToGo) {
		reset();
		while (true) {
			double distance = encoder.get();
			double angle = gyro.getAngle();
			if (!robot.isAutonomous() || !robot.isEnabled()) return;
			SmartDashboard.putNumber("Distance", distance);
			chassis.arcadeDrive(-0.50, angle * kP);
			if (distance < distanceToGo) { 
				reset();
				return;
			}
			Timer.delay(0.02);
		}
	}
	private void forwardDrive(double distanceToGo) {
		reset();
		while(true) {
			double distance = encoder.get();
			if(!robot.isAutonomous() || !robot.isEnabled()) return;
			SmartDashboard.putNumber("Distance", distance);
			chassis.arcadeDrive(-0.50, 0);
			if(distance < distanceToGo) {
				reset();
				return;
			}
			Timer.delay(0.02);
		}
	}
	
	private void liftBin(double second) {
		double screwTime = Timer.getFPGATimestamp();
		while (true) {
			if (!robot.isAutonomous() || !robot.isEnabled()) return;
			SmartDashboard.putNumber("LiftBinTimer", screwTime);
			if (screwTime + second > Timer.getFPGATimestamp()) {
				screwMotor1.set(Defines.SCREW_SPEED);
				screwMotor2.set(Defines.SCREW_SPEED);
			} else {
				reset();
				return;
			}
			Timer.delay(0.02);
		}
	}
	
	public void run(Autos autoMode) {
		switch (autoMode) {
			case AUTO_MOVE_TO_ZONE:
				forwardDrive(-1000);
				break;
			case AUTO_DO_NOTHING:
				break;
			case AUTO_GRAB_ONE_BIN:
				forwardDrive(-100);
				liftBin(2.5);
				basicTurn(180);
				forwardDrive(-1500);
				break;	
			case AUTO_LIFT_BIN:
				forwardDrive(-100);
				liftBin(2.5);
				break;
		}
	}
}
