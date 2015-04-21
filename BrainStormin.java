
package org.usfirst.frc.team5033.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;

public class Robot extends SampleRobot {
	public RobotDrive chassis;
	public Joystick stick;
	public Joystick xbox;
	
	public Robot() {
		chassis = new RobotDrive(0, 1);
		stick = new Joystick(1);
		xbox = new Joystick(0);

	}
	public void autonomous() {

	}
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			Timer.delay(0.02);
		}
	}
	public void test() {
