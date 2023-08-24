package frc.robot;

public class Constants {

    // d for drive, s for steer, enc for encoders
    public final static int dFL_ID = 1;
    public final static int sFL_ID = 2;
    public final static int encFL_ID = 9;
    public final static int dFR_ID = 3;
    public final static int sFR_ID = 4;
    public final static int encFR_ID = 10;
    public final static int dBL_ID = 7;
    public final static int sBL_ID = 8;
    public final static int encBL_ID = 11;
    public final static int dBR_ID = 5;
    public final static int sBR_ID = 6;
    public final static int encBR_ID = 12;

    public final static int[] driveMotors = {dFL_ID, dFR_ID, dBL_ID, dBR_ID};
    public final static int[] steerMotors = {sFL_ID, sFR_ID, sBL_ID, sBR_ID};
    public final static int[] encoders = {encFL_ID, encFR_ID, encBL_ID, encBR_ID};

    // To prevent the robot from reacting to joystick noise, blind areas are added around the origin
    // ranging from 0 to 1
    public final static double xBlind = 0.2;
    public final static double yBlind = 0.2;
    public final static double zBlind = 0.2;

    // Distances between the modules in meters
    // length refers to front/back distance, width refers to left/right distance
    public final static double Length = 0.8;
    public final static double Width = 0.8;
    
    // PID parameters
    public final static double swerveKp = -0.5;
    public final static double swerveKi = 150;
    public final static double swerveKd = -0.02;
    public final static double swervePIDScale = 100;
    public final static double swervePIDTolerance = 0;

    // Controls speed of the drive motors.
    public final static double motorMod = 0.3;

    // encoder offsets
    public final static double offsetFL = -135 + 180 -10;
    public final static double offsetFR = 23 + 180 -10;
    public final static double offsetBL = -120 + 10;
    public final static double offsetBR = -45;
}
