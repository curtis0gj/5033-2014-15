package org.usfirst.frc.team5033.robot;


import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
	public RobotDrive robot;
	public Joystick stick;
	public Joystick xbox;
	public Encoder encoder;
	public Gyro gyro;
	public Relay leftarmwheel;
	public Relay rightarmwheel;
	public Victor screwmotor1;
	public Victor screwmotor2;
	public Victor armmotor;
	public DigitalInput limit;
	public DigitalInput limit2;
	public DigitalInput limit3;
	public DigitalInput limit4;
	public double Kp = 0.03;
	public boolean bottomscrewlimit = false;
	public boolean limitPressed2 = false;
	public boolean minarmlimit = false;
	public boolean maxarmlimit = false;
	public double x = 0;
	public SendableChooser autoChooser;
	public Defines.AUTOS autoMethod;

	public Robot() {
		robot = new RobotDrive(0, 1);
		stick = new Joystick(1);
		xbox = new Joystick(0);
		gyro = new Gyro(0);
		encoder = new Encoder(0, 1, false, EncodingType.k4X);
		leftarmwheel = new Relay(1);
		rightarmwheel = new Relay(2);
		limit = new DigitalInput(4);
		limit2 = new DigitalInput(5);
		limit3 = new DigitalInput(6);
		limit4 = new DigitalInput(7);
		screwmotor1 = new Victor(4);
		screwmotor2 = new Victor(5);
		armmotor = new Victor(6);
		gyro.reset();
		gyro.initGyro();
		encoder.setDistancePerPulse(0.3193143);
		encoder.getDistance();

		autoChooser = new SendableChooser();
		autoChooser.addDefault("Pick Up Two Blue", Defines.AUTOS.AUTO_TWOBLUE);
		autoChooser.addObject("Pick Up Two Red", Defines.AUTOS.AUTO_TWORED);
		autoChooser.addObject("Pick Up One", Defines.AUTOS.AUTO_ONE);
		autoChooser.addObject("Move Into Autozone", Defines.AUTOS.AUTO_MOVE);
		SmartDashboard.putData("Auto mode", autoChooser);
	}



	public void autonomous() {
		autoMethod = (Defines.AUTOS) autoChooser.getSelected();
		Auto.run(this, autoMethod);

		if (autoMethod == Defines.AUTOS.AUTO_ONE) {
			return;
		} else if (autoMethod == Defines.AUTOS.AUTO_MOVE) {
			return;
		} else if (autoMethod == Defines.AUTOS.AUTO_TWORED) {
			return;
		} else if (autoMethod == Defines.AUTOS.AUTO_TWOBLUE) {
			return;
		}
	}
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			double throttle = stick.getRawAxis(Defines.STICK_THROTTLE);
			double leftaxis = xbox.getRawAxis(Defines.LEFT_AXIS);
			double rightaxis = xbox.getRawAxis(Defines.RIGHT_AXIS);
			bottomscrewlimit = limit.get();
			minarmlimit = limit3.get();
			maxarmlimit = limit4.get();
			x = (-throttle + 1) / 2;

			robot.arcadeDrive(stick.getY() * x, -stick.getX() * x);

			if (-leftaxis > 0.5) {
				screwmotor1.set(Defines.SCREW_SPEED);
				screwmotor2.set(Defines.SCREW_SPEED);
			} else if (leftaxis > 0.5 && bottomscrewlimit == false) {
				screwmotor1.set(-Defines.SCREW_SPEED);
				screwmotor2.set(-Defines.SCREW_SPEED);
			} else {
				screwmotor1.set(Defines.SCREW_OFF);
				screwmotor2.set(Defines.SCREW_OFF);
			}
			if (-rightaxis > 0.5 && maxarmlimit == true) {
				armmotor.set(Defines.ARM_SPEED);
			} else if (rightaxis > 0.5 && minarmlimit == true) {
				armmotor.set(-Defines.ARM_SPEED);
			} else {
				armmotor.set(Defines.ARM_OFF);
			}
			if (xbox.getRawButton(Defines.RIGHT_TRIGGER)) {
				leftarmwheel.set(Relay.Value.kForward);
				rightarmwheel.set(Relay.Value.kReverse);
			} else if (xbox.getRawButton(Defines.LEFT_TRIGGER)) {
				leftarmwheel.set(Relay.Value.kReverse);
				rightarmwheel.set(Relay.Value.kForward);
			} else {
				leftarmwheel.set(Relay.Value.kOff);
				rightarmwheel.set(Relay.Value.kOff);
			}
		}
	}
	public void test() {

	}
}
