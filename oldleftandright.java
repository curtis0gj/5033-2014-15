	private static void turnRight(Robot r, double deg) {
		while (true) {
			double angle = r.gyro.getAngle();
			if (!r.isAutonomous() || !r.isEnabled()) return;
			r.robot.drive(-.40, -1);
			if (angle > deg) {
				reset(r);
				return;
			}
			Timer.delay(0.02);
		}
	}
	private static void turnLeft(Robot r, double deg) {
		while (true) {
			double angle = r.gyro.getAngle();
			if (!r.isAutonomous() || !r.isEnabled()) return;
			r.robot.drive(-.40, 1);
			if (angle < deg) {
				reset(r);
				return;
			}
			Timer.delay(0.02);
		}
	}
	
	
				turnLeft(r, -70); 
				turnRight(r, 70);
