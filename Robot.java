package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {

	private Joystick joys;
	private SwerveDrive drive;
	private double joysX, joysY, joysZ, vX, vY, omega;

	@Override
	public void robotInit() {
		joys = new Joystick(0);
		drive = new SwerveDrive();
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
		joysY = joys.getY();
		joysX = joys.getX();
		joysZ = joys.getZ();
		vX = (Math.abs(joysY) > Constants.xBlind) ? joysY : 0;
		vY = (Math.abs(joysX) > Constants.yBlind) ? joysX : 0;
		omega = (Math.abs(joysZ) > Constants.zBlind) ? joysZ * (-1) : 0;

		drive.updateSwerve(vX, vY, omega);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}
}
