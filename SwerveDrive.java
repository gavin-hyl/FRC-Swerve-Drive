import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveDrive {
    
	private Translation2d FLpos = new Translation2d(Constants.Length / 2, Constants.Width / 2);
	private Translation2d FRpos = new Translation2d(Constants.Length / 2, -Constants.Width / 2);
	private Translation2d BLpos = new Translation2d(-Constants.Length / 2, Constants.Width / 2);
	private Translation2d BRpos = new Translation2d(-Constants.Length / 2, -Constants.Width / 2);
	private SwerveDriveKinematics SDK = new SwerveDriveKinematics(FLpos, FRpos, BLpos, BRpos);

	private SwerveDriveModule FLModule = new SwerveDriveModule(FLpos, Constants.encoders[0], Constants.offsetFL, Constants.steerMotors[0], Constants.driveMotors[0], Constants.swerveKp, Constants.swerveKi, Constants.swerveKp, Constants.swervePIDTolerance, Constants.swervePIDScale);
	private SwerveDriveModule FRModule = new SwerveDriveModule(FRpos, Constants.encoders[1], Constants.offsetFL, Constants.steerMotors[1], Constants.driveMotors[1], Constants.swerveKp, Constants.swerveKi, Constants.swerveKp, Constants.swervePIDTolerance, Constants.swervePIDScale);
	private SwerveDriveModule BLModule = new SwerveDriveModule(BLpos, Constants.encoders[2], Constants.offsetFL, Constants.steerMotors[2], Constants.driveMotors[2], Constants.swerveKp, Constants.swerveKi, Constants.swerveKp, Constants.swervePIDTolerance, Constants.swervePIDScale);
	private SwerveDriveModule BRModule = new SwerveDriveModule(BRpos, Constants.encoders[3], Constants.offsetFL, Constants.steerMotors[3], Constants.driveMotors[3], Constants.swerveKp, Constants.swerveKi, Constants.swerveKp, Constants.swervePIDTolerance, Constants.swervePIDScale);
	private SwerveDriveModule modules[] = {FLModule, FRModule, BLModule, BRModule};

	private SwerveModuleState[] states;
	private ChassisSpeeds cSpeeds;
	private double[] angles = new double[4];

    /**
	 * Calculates and updates the state all eight motors in the swerve drive.
	 * 
	 * @param vX desired speed in the front/back direction, forward is positive.
	 * @param vY desired speed in the left/right direction, left is positive.
	 * @param omega desired angular speed, up is positive.
	 */
    public void updateSwerve(double vX, double vY, double omega) {

		cSpeeds = new ChassisSpeeds(vX, vY, omega);

		states = SDK.toSwerveModuleStates(cSpeeds);

		for (int i = 0; i < 4; i++) {
			angles[i] = states[i].angle.getDegrees();
		}

		// Make sure that the motors are facing forward
		// (tricky things happen when the SDK believes the robot should be stationary)
		if (vX == vY && vY == omega && omega ==0){
			angleFL += 180;
			angleFR += 180;
			angleBL += 180;
			angleBR += 180;
		}

		for (int i = 0; i < 4; i++) {
			modules[i].setSteerMotor(angle);
		}
		
		// Refer to setDriveMotor as to why these have to be seperate.
		for (int i = 0; i < 4; i++) {
			modules[i].setDriveMotor(states[i].speedMetersPerSecond * motorMod);
		}
    }
}

private class SwerveDriveModule {

	private Translation2d modeulePosition;
	private CANCoder encoder;
	private double encoderOffset;
	private TalonFX steer;
	private TalonFX drive;
	private PIDController pid;

	public SwerveDriveModule(Translation2d pos, int encoderID, double encOffset, int steerID, int driveID, double kp, double ki, double kd, double pidTolerance, double pidScale) {
		modeulePosition = pos;
		encoder = new CANCoder(encoderID);
		encoderOffset = encOffset;
		steer = new TalonFX(steerID);
		drive = new TalonFX(driveID);
		pid = new PIDController(kp, ki, kd);
		pid.setTolerance(pidTolerance);
	}
	
	/**
	 * Powers the steer motor using the PID.
	 * 
	 * @param target The desired target angle.
	 */
	public void setSteerMotor(double target) {
		pid.setSetpoint(target);
		double currentPos = encoder.getPosition() + encoderOffset;
		double out = pid.calculate(currentPos);
		steer.set(ControlMode.PercentOutput, out / pidScale);
	}

	/**
	 * Powers the drive motor. The two functions (set drive and set steer) 
	 * must be seperate because of the time used by the PID and set functions
	 * may result in inaccurate steer motor PID calculations
	 * 
	 * @param powerPercentage The desired power (0.0 - 1.0)
	 */
	public void setDriveMotor(double powerPercentage) {
		drive.set(ControlMode.PercentOutput, powerPercentage);
	}
}