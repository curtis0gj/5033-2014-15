package org.usfirst.frc.team5033.robot;

import org.usfirst.frc.team5033.robot.Defines.AUTOS;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

public class Auto {
	private static void wait(Robot r, double waittime) {
		while(true) {
			if(!.risAutonomous() || !r.isEnabled())
				return;
			r.Timer.delay(waittime); //r or just timer?
				return;
		}
	
	private static void reset(Robot r) {
		r.robot.drive(0, 0);
		r.gyro.reset();
		r.encoder.reset();
		r.screwmotor1.set(Defines.SCREW_OFF);
		r.screwmotor2.set(Defines.SCREW_OFF);
	}
	
	private static void move(Robot r, int d) {
		while(true) {
			//Drive to auto zone.
    		double distance = r.encoder.get();
    		double angle = r.gyro.getAngle();
			if(!r.isAutonomous() || !r.isEnabled())
				return;
			r.robot.drive(-0.25, angle * r.Kp);
			//r.robot.drive(-0.40, 0);
    		if(distance < -d) {
    			reset(r);
    			return;
    		}
		}
	}
	private static void lift(Robot r, double sec) {
		double ScrewTime1 = Timer.getFPGATimestamp();
		while(true) {
			if(!r.isAutonomous() || !r.isEnabled())
				return;
			if(ScrewTime1 + sec > Timer.getFPGATimestamp()) {
				r.screwmotor1.set(Defines.SCREW_SPEED);
				r.screwmotor2.set(Defines.SCREW_SPEED);
			} else {
				reset(r);
				break;	
			}
		}
	}

	private static void turn(Robot r, int deg) {
		while(true) {
    		//Turn 90, stop and wait one second.
    		double angle = r.gyro.getAngle();
    		if(!r.isAutonomous() || !r.isEnabled())
    			return;
    		r.robot.drive(-.40, -1);
			if(angle > deg) {
				reset(r);
    			return;
    		}
    	}
	}
	private static void turn2(Robot r, int deg) {
		while(true) {
    		//Turn 90, stop and wait one second.
    		double angle = r.gyro.getAngle();
    		if(!r.isAutonomous() || !r.isEnabled())
    			return;
    		r.robot.drive(-.40, 1);
			if(angle < deg) {
				reset(r);
    			return;
    		}
    	}
	}
	private static void arms(Robot r, double sec) {
		double ArmTimeR = Timer.getFPGATimestamp();
    	while(true) {
    		//Close arms for two seconds and stop, wait one.
    		if(!r.isAutonomous() || !r.isEnabled())
    			return;
    		Boolean maxarmlimit = r.limit4.get();
    		if(ArmTimeR + sec > Timer.getFPGATimestamp()) {
    			r.armmotor.set(Defines.ARM_SPEED);
				r.leftarmwheel.set(Relay.Value.kForward);
	        	r.rightarmwheel.set(Relay.Value.kReverse);
    		} else if(maxarmlimit == true) {
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
		switch(autoMode) {
		case AUTO_MOVE:
    		move(r, 3000);
			break;
		case AUTO_ONE:
			lift(r, 3.5);
			wait(r, 0.5);
        	turn(r, 70);
        	wait(r, 0.5);
        	move(r, 2000);
			break;
		case AUTO_TWORED:
			lift(r, 3.5);
			wait(r, 0.5);
			move(r, 450);
			wait(r, 0.5);
			arms(r, 2);
			wait(r, 0.5);
			turn(r, 70);
			wait(r, 0.5);
			move(r, 2000);
			break;
		case AUTO_TWOBLUE:
			lift(r, 3.5);
			wait(r, 0.5);
			move(r, 450);
			wait(r, 0.5);
			arms(r, 2);
			wait(r, 0.5);
			turn2(r, -70); // - or + 70?
			wait(r, 0.5);
			move(r, 2000);
			break;
		}
	}
}
