package org.usfirst.frc.team5033.robot;

import org.usfirst.frc.team5033.robot.Defines.AUTOS;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

public class Auto {
	private static void wait(Robot r, double waittime) {
		Timer.delay(waittime); 
	}
	private static void reset(Robot r) {
		r.robot.drive(0, 0);
		r.gyro.reset();
		r.encoder.reset();
		r.screwMotor1.set(Defines.SCREW_OFF);
		r.screwMotor2.set(Defines.SCREW_OFF);
	}
	public static void angleError(double setpointDegressZeroToThreeSixty, double experimentalDegrees) {
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
	public static void turn(Robot r, double deg) {
		while(true) {
			double deltaAngle = angleError(deg, gyro.getAngle());
			if(Math.abs(deg - deltaAngle) < MAX_ERROR) {
				break;
			} else {
				r.robot.drive(0, deltaAngle * kp_rotate);
			}
			Timer.delay(0.02);
		}
	}
	private static void forwardDrive(Robot r, double d) {
		while (true) {
			double distance = r.encoder.get();
			double angle = r.gyro.getAngle();
			if (!r.isAutonomous() || !r.isEnabled()) return;
			r.robot.drive(-0.25, angle * r.Kp);
			//r.robot.drive(-0.40, 0);
			if (distance < -d) {
				reset(r);
				return;
			}
			Timer.delay(0.02);
		}
	}
	private static void liftBin(Robot r, double sec) {
		double ScrewTime1 = Timer.getFPGATimestamp();
		while (true) {
			if (!r.isAutonomous() || !r.isEnabled()) return;
			if (ScrewTime1 + sec > Timer.getFPGATimestamp()) {
				r.screwMotor1.set(Defines.SCREW_SPEED);
				r.screwMotor2.set(Defines.SCREW_SPEED);
			} else {
				reset(r);
				break;
			}
			Timer.delay(0.02);
		}
	}
	private static void closeArms(Robot r, double sec) {
		double armTimer = Timer.getFPGATimestamp();
		while (true) {
			if (!r.isAutonomous() || !r.isEnabled()) return;
			boolean maxArmLimit = r.limit4.get();
			if (ArmTimeR + sec > Timer.getFPGATimestamp()) {
				r.armMotor.set(Defines.ARM_SPEED);
				r.leftArmWheel.set(Relay.Value.kForward);
				r.rightArmWheel.set(Relay.Value.kReverse);
			} else if (maxArmLimit == true) {
				r.armMotor.set(Defines.ARM_OFF);
				r.leftArmWheel.set(Relay.Value.kOff);
				r.rightArmWheel.set(Relay.Value.kOff);
				reset(r);
				break;
			} else {
				reset(r);
				break;
			}
			Timer.delay(0.02);
		}
	}
	public static void run(Robot r, AUTOS autoMode) {
		switch (autoMode) {
			case AUTO_MOVE_TO_ZONE:
				move(r, 3000);
				break;
			case AUTO_GRAB_ONE_BIN_RED_SIDE:
				liftBin(r, 3.5);
				wait(r, 0.5);
				turnRight(r, 70); //COULD BE LEFT OR RIGHT I DON'T KNOW!
				wait(r, 0.5);
				forwardDrive(r, 2000);
				break;
			case AUTO_GRAB_ONE_BIN_BLUE_SIDE:
				liftBin(r, 3.5);
				wait(r, 0.5);
				turnLeft(r, -70);  //COULD BE LEFT OR RIGHT I DON'T KNOW!
				wait(r, 0.5);
				forwardDrive(r, 2000);
				break;
			case AUTO_GRAB_TWO_BINS_RED_SIDE:
				liftBin(r, 3.5);
				wait(r, 0.5);
				forwardDrive(r, 450);
				wait(r, 0.5);
				closeArms(r, 2);
				wait(r, 0.5);
				turnRight(r, 70);  //COULD BE LEFT OR RIGHT I DON'T KNOW!
				wait(r, 0.5);
				forwardDrive(r, 2000);
				break;
			case AUTO_GRAB_TWO_BINS_BLUE_SIDE:
				liftBin(r, 3.5);
				wait(r, 0.5);
				forwardDrive(r, 450);
				wait(r, 0.5);
				closeArms(r, 2);
				wait(r, 0.5);
				turnLeft(r, -70); // - or + 70?  //COULD BE LEFT OR RIGHT I DON'T KNOW!
				wait(r, 0.5);
				forwardDrive(r, 2000);
				break;
		}
	}
}
