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
		r.screwmotor1.set(Defines.SCREW_OFF);
		r.screwmotor2.set(Defines.SCREW_OFF);
	}
	public static void turn(Robot r, double deg) {
		while (true) {
			double angle = r.gyro.getAngle();
			if (!r.isAutonomous() || !r.isEnabled()) return;
			if(angle < 70) {
				turnRight(r, 70);
			} else {
				turnLeft(r, -70);
				reset(r);
				break;
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
		}
	}
	private static void liftBin(Robot r, double sec) {
		double ScrewTime1 = Timer.getFPGATimestamp();
		while (true) {
			if (!r.isAutonomous() || !r.isEnabled()) return;
			Timer.delay(0.02);
			if (ScrewTime1 + sec > Timer.getFPGATimestamp()) {
				r.screwmotor1.set(Defines.SCREW_SPEED);
				r.screwmotor2.set(Defines.SCREW_SPEED);
			} else {
				reset(r);
				break;
			}
		}
	}
	private static void turnRight(Robot r, double deg) {
		while (true) {
			double angle = r.gyro.getAngle();
			if (!r.isAutonomous() || !r.isEnabled()) return;
			r.robot.drive(-.40, -1);
			Timer.delay(0.02);
			if (angle > deg) {
				reset(r);
				return;
			}
		}
	}
	private static void turnLeft(Robot r, double deg) {
		while (true) {
			double angle = r.gyro.getAngle();
			if (!r.isAutonomous() || !r.isEnabled()) return;
			r.robot.drive(-.40, 1);
			Timer.delay(0.02);
			if (angle < deg) {
				reset(r);
				return;
			}
		}
	}
	private static void closeArms(Robot r, double sec) {
		double armTimer = Timer.getFPGATimestamp();
		while (true) {
			if (!r.isAutonomous() || !r.isEnabled()) return;
			boolean maxarmlimit = r.limit4.get();
			Timer.delay(0.02);
			if (ArmTimeR + sec > Timer.getFPGATimestamp()) {
				r.armmotor.set(Defines.ARM_SPEED);
				r.leftarmwheel.set(Relay.Value.kForward);
				r.rightarmwheel.set(Relay.Value.kReverse);
			} else if (maxarmlimit == true) {
				r.armmotor.set(Defines.ARM_OFF);
				r.leftarmwheel.set(Relay.Value.kOff);
				r.rightarmwheel.set(Relay.Value.kOff);
				reset(r);
				break;
			} else {
				reset(r);
				break;
			}
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
