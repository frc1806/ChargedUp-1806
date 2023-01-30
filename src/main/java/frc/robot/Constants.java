package frc.robot;

public class Constants {
    //Controller Ports
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;
    public static final int kDebugPort = 2;

    //Smart Dashboard Keys
    public static final String kDriveTrainKey = "DriveTrain";

    //Drive Train Constants
    public static final int kDriveTrainCurrentLimit = 80;
    public static final double kDriveTrainTrackWidthMeters = .6858; //~ 27"
    public final static double kDriveCountsPerMeter = 6628.32202312992;
    public final static double kDriveMetersPerCount = 1/kDriveCountsPerMeter;

    //Drive feed forwards
    public static final double kDriveTrainKs = 0.0;
    public static final double kDriveTrainKv = 0.0;
    public static final double kDriveTrainKa = 0.0;
}
