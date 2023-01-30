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
    public final static double kCountsPerInch = 168.359374;
    public final static double kDriveInchesPerCount = 1/kCountsPerInch;
}
