package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    //imortant
    public static final boolean isCompBot = true;
    public static final boolean isArmWiringPresent = true;
    public static final boolean areLimitSwitchesOnClaw = true;
    public static final boolean clawDIOsDebug = true;
    //GAME
    public static final double kFieldLength = 16.54175;
    public static final double kFieldWidth = 8.0137;
    //Controller Ports
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;
    public static final int kDebugPort = 2;

    //Drive Train Constants
    public static final int kDriveTrainCurrentLimit = 85;
    public static final double kDriveTrainTrackWidthMeters = Units.inchesToMeters(25.5); //~ 27"
    public final static double kDriveCountsPerMeter = 6628.32202312992;
    public final static double kDriveMetersPerCount = 1/kDriveCountsPerMeter;
    public final static double kDriveTurningSensitivity = 0.5;

    //Drive feed forwards
    public static final double kDriveTrainKs = 0.2703;
    public static final double kDriveTrainKv = 1.1603;
    public static final double kDriveTrainKa = 0.40226;

    //Powered Brake
    public static final double kDriveTrainMinimumMovingSpeed = .3048;
    public static final double kDriveTrainNormalPowerBrakePower = .025;
    public static final double kDriveTrainRampPowerBrakePower = 0.5;

    // Protruder
    public static final double kProtruderkP = 1.0/6.0;
    public static final double kProtruderkI = 0;
    public static final double kProtruderkD = 0;
    public static final double kProtruderFeedFowardGain = 0.1;
    public static final double kProtruderAcceptableDistanceDelta = 1.5;
    public static final double kProtruderDistanceAtFullRetract = 25; //TODO: Make number for actual robot and implement
    public static final double kProtruderDistanceAtFullExtend = 50; //TODO Measure
    public static final double kProtruderFirstStageExtension = 17;
    public static final double kProtruderSecondStageLength = 20;
    public static final double kProtruderSecondStageStallAmps = 5;
    public static final double kProtruderSecondStageStallTimeout = 0.2;
    public static final double kProtruderAcceptableFirstStageExtensionToPassThrough = 1.0;

    
    //Arm Angle Constants
    public static final double kArmGearRatio = 272.222222222222;
    public static final double kPivotArmAngleKp = 1.0/15.0;
    public static final double  kPivotArmAngleKi = 0.0;
    public static final double  kPivotArmAngleKd = 0.0;
    public static final double kAcceptableAngleDelta = 2.0;

    public static final double kPivotFeedForwardGain = 0.006;

    //ClawSpinner
    public static final int kClawSpinnerBufferSize = 20;
    public static final double kClawSpinnerStalledCurrent = 6.0;
    public static final double kClawSpinnerSpeed = 0.7;
    public static final double kClawTimeFrame = 3.0;

    //LEDs
    public static final int kTotalLEDCount = 24;

    //SlowAutoBalance
    public static final double kSlowBalanceTippedDegrees = 10.0;
    public static final double kSlowBalanceLevelDegrees = 3.5;
}
