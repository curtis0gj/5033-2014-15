
package org.usfirst.frc.team5033.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	public RobotDrive chassis;
	public Joystick stick;
	public Joystick xbox;
	public Encoder encoder;
	public Gyro gyro;
	public Relay leftArmWheel;
	public Relay rightArmWheel;
	public Victor screwMotor1;
	public Victor screwMotor2;
	public Victor armMotor;
	public DigitalInput limit;
	/*public DigitalInput limit2;
	public DigitalInput limit3;*/
	public boolean bottomScrewLimit = false;
	/*public boolean minArmLimit = false;
	public boolean maxArmLimit = false;*/
	public double x = 0;
	public SendableChooser autoChooser;
	public Defines.Autos autoMethod;
	public Auto auto;
	
	public Robot() {
		chassis = new RobotDrive(0, 2);
		stick = new Joystick(1);
		xbox = new Joystick(0);
		gyro = new Gyro(0);
		encoder = new Encoder(0, 1, false, EncodingType.k4X);
		leftArmWheel = new Relay(1);
		rightArmWheel = new Relay(2);
		limit = new DigitalInput(4);
		
		/*limit2 = new DigitalInput(6);
		limit3 = new DigitalInput(7);*/
		
		screwMotor1 = new Victor(4);
		screwMotor2 = new Victor(5);
		armMotor = new Victor(6);
		
		gyro.reset();
		gyro.initGyro();
		encoder.setDistancePerPulse(0.3193143);
		encoder.getDistance();
		
		auto = new Auto(this);
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Do nothing.", Defines.Autos.AUTO_DO_NOTHING);
		autoChooser.addObject("Lift one bin and stay.", Defines.Autos.AUTO_LIFT_BIN);
		autoChooser.addObject("Pick up one bin.", Defines.Autos.AUTO_GRAB_ONE_BIN);
		autoChooser.addObject("Move into the autonomous zone.", Defines.Autos.AUTO_MOVE_TO_ZONE);
		SmartDashboard.putData("AUTONOMOUS MODES", autoChooser);
	}
	public void autonomous() {
		autoMethod = (Defines.Autos) autoChooser.getSelected();
		auto.run(autoMethod);
	}
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			double throttle = stick.getRawAxis(Defines.STICK_THROTTLE);
			
			/*double leftAxis = xbox.getRawAxis(Defines.LEFT_AXIS);
			double rightAxis = xbox.getRawAxis(Defines.RIGHT_AXIS);
			minArmLimit = limit2.get();
			maxArmLimit = limit3.get();*/
			
			bottomScrewLimit = limit.get();
			
			x = (-throttle + 2) / 4;
			chassis.arcadeDrive(stick.getY() * x, -stick.getX() * x);
			
			if (stick.getRawButton(3)) {
				screwMotor1.set(Defines.SCREW_SPEED);
				screwMotor2.set(Defines.SCREW_SPEED);
			} else if (stick.getRawButton(2) && bottomScrewLimit == false) {
				screwMotor1.set(-Defines.SCREW_SPEED);
				screwMotor2.set(-Defines.SCREW_SPEED);
			} else {
				screwMotor1.set(Defines.SCREW_OFF);
				screwMotor2.set(Defines.SCREW_OFF);	
			}
			
			/*if (-rightAxis > 0.5 && maxArmLimit == false) {
				armMotor.set(Defines.ARM_SPEED);
			} else if (rightAxis > 0.5 && minArmLimit == false) {
				armMotor.set(-Defines.ARM_SPEED);
			} else {
				armMotor.set(Defines.ARM_OFF);
			}
			if (xbox.getRawButton(Defines.RIGHT_TRIGGER)) {
				leftArmWheel.set(Relay.Value.kForward);
				rightArmWheel.set(Relay.Value.kReverse);
			} else if (xbox.getRawButton(Defines.LEFT_TRIGGER)) {
				leftArmWheel.set(Relay.Value.kReverse);
				rightArmWheel.set(Relay.Value.kForward);
			} else {
				leftArmWheel.set(Relay.Value.kOff);
				rightArmWheel.set(Relay.Value.kOff);
			}*/
			
			Timer.delay(0.02);
		}
	}
	public void test() {

	}
}

