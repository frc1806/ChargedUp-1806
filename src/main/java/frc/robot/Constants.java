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
}
