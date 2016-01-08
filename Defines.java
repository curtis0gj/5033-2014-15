package org.usfirst.frc.team5033.robot;

public class Defines {
	public static final double SCREW_SPEED = 0.5;
	public static final double SCREW_OFF = 0.0;
	public static final double ARM_SPEED = 0.2;
	public static final double ARM_OFF = 0.0;
	public static final int STICK_THROTTLE = 2;
	public static final int LEFT_AXIS = 1;
	public static final int RIGHT_AXIS = 5;
	public static final int LEFT_TRIGGER = 5;
	public static final int RIGHT_TRIGGER = 6;

	public enum Autos {
		AUTO_GRAB_ONE_BIN,
		AUTO_MOVE_TO_ZONE,
		AUTO_LIFT_BIN,
		AUTO_DO_NOTHING
	}
}
