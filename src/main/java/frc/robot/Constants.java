package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    //Controller Ports
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;
    public static final int kDebugPort = 2;

    //Drive Train Constants
    public static final int kDriveTrainCurrentLimit = 80;
    public static final double kDriveTrainTrackWidthMeters = Units.inchesToMeters(25.5); //~ 27"
    public final static double kDriveCountsPerMeter = 6628.32202312992;
    public final static double kDriveMetersPerCount = 1/kDriveCountsPerMeter;
    public final static double kDriveTurningSensitivity = 0.5;

    //Drive feed forwards
    public static final double kDriveTrainKs = 0.2703;
    public static final double kDriveTrainKv = 1.1603;
    public static final double kDriveTrainKa = 0.40226;

    // Protruder
    public static final double kProtruderkP = 1/6;
    public static final double kProtruderkI = 0;
    public static final double kProtruderkD = 0;
    //Arm Angle Constants
    public static final double kArmGearRatio = 333.0 + (1.0/3.0);
    public static final double kPivotArmAngleKp = 1.0/20.0;
    public static final double  kPivotArmAngleKi = 0.0;
    public static final double  kPivotArmAngleKd = 0.0;
    public static final double kAcceptableAngleDelta = 1.0/2.0;

    //ClawSpinner
    public static final int kClawSpinnerBufferSize = 20;
    public static final double kClawSpinnerStalledCurrent = 6.0;
    public static final double kClawSpinnerSpeed = 0.7;
    public static final double kClawTimeFrame = 3.0;

}
