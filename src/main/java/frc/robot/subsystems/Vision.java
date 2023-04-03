package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;


public class Vision extends SubsystemBase{

    private NetworkTable limelightTable;
    private double currentTimestamp;
    private  double lastAllianceUpdate;
    private Alliance currentAlliance;
    private String limelightHostname;

    //SIMULATION
    private final Translation2d testNodePose = new Translation2d(15.77, 4.98);

    public Vision(String hostname) { 
        limelightHostname = hostname;
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightHostname);
        currentTimestamp = 0.0;
        lastAllianceUpdate = Double.NEGATIVE_INFINITY;
        currentAlliance = Alliance.Invalid;
    }

    public double getValidTargetUpdateTimestamp(){
        return limelightTable.getEntry("tv").getLastChange();
    }

    public boolean hasLimelightUpdatedRecently(){
        return getValidTargetUpdateTimestamp()/ 1000000 > currentTimestamp - 1.0;
    }

    public Alliance getCurrentAlliance(){
        return currentAlliance;
    }

    /**
     * Update the limelight pose relative to the center of the drive base on the floor.
     * @param metersForwardOfCenter meters forward or backward of the centerline (where the cross member bar is). Forward is positive.
     * @param metersLeftOrRight meters left or right of the middle of the robot (the centerline between the left and right wheel sets). Right is positive.
     * @param metersUpOrDown meters up or down. Negative means the limelight has clipped into the floor, so don't use negative values for this.
     * @param yaw rotation of the limelight relative to forwards on the robot being 0. In Degrees.
     * @param pitch whether the limelight is angled up or down. In Degrees.
     * @param roll limelight skew. Let's try to mount the limelight so this is always 0. In Degrees.
     */
    public void updateLimelightPose(double metersForwardOfCenter, double metersLeftOrRight, double metersUpOrDown, double yaw, double pitch, double roll){
        LimelightHelpers.setCameraPose_RobotSpace(limelightHostname, metersForwardOfCenter, metersLeftOrRight, metersUpOrDown, roll, pitch, yaw);
    }


    public double getAprilTagId(){
        return LimelightHelpers.getFiducialID(limelightHostname);
    }

    public double getTargetYaw(){
        return LimelightHelpers.getTX(limelightHostname);
    }

    /**
     * Get robot pose based on apriltags
     * @return
     */
    public Pose2d getBotPose(){
        switch(currentAlliance){
            case Blue:

                return LimelightHelpers.getBotPose2d_wpiBlue(limelightHostname);
            case Red:
                return LimelightHelpers.getBotPose2d_wpiRed(limelightHostname);
            default:
            case Invalid:
                DriverStation.reportError("VisionSubystem.java: Could not get bot pose. Invalid Alliance.", true);
                return null;

            
        }
    }

    @Override
    public void periodic() {
        outputToSmartDashboard();
        currentTimestamp = Timer.getFPGATimestamp();
        //Check alliance every 5 seconds, hopesfully this will update in disabled. If not it's on the dashboard so we can thumbs-down.
        if(lastAllianceUpdate + 2.5 < currentTimestamp){
            currentAlliance = DriverStation.getAlliance();
            lastAllianceUpdate = currentTimestamp;
        }

    }

    @Override
    public void simulationPeriodic(){

        double angleToTestGoal = -testNodePose.minus(RobotContainer.S_DRIVETRAIN.getPose().getTranslation()).getAngle().getDegrees() + RobotContainer.S_DRIVETRAIN.getPose().getRotation().minus(new Rotation2d(Units.degreesToRadians(180.0))).getDegrees();
        if(RobotContainer.S_DRIVETRAIN.getPose().getTranslation().getDistance(testNodePose) < 8.0 && Math.abs(angleToTestGoal) < 27.0)
        {
            limelightTable.getEntry("tv").setBoolean(true);
            limelightTable.getEntry("tx").setDouble(angleToTestGoal);
            //limelightTable.getEntry("tx").setDouble(RobotContainer.S_DRIVETRAIN.getPose().relativeTo(testNodePose).getRotation().getDegrees());
        }
        else
        {
            limelightTable.getEntry("tv").setBoolean(false);
            limelightTable.getEntry("tx").setDouble(0.0);
        }
    }

    public boolean hasAprilTagTarget(){
        return LimelightHelpers.getFiducialID(limelightHostname) != -1  && LimelightHelpers.getTV(limelightHostname);
    }

    /* TODO: Port to new smart dashboard tab setup
    @Override
    public void setupDriverTab() {

               
        Robot.getMainCompetitionTab().addNumber("April Tag ID", new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                return getAprilTagId();
            }
               
           }).withWidget(BuiltInWidgets.kNumberBar).withPosition(3,0).withSize(1, 1);

           Robot.getMainCompetitionTab().addNumber("Yaw Value", new DoubleSupplier() {

            @Override
            public double getAsDouble() {
                return getTarget();
            }
               
           }).withWidget(BuiltInWidgets.kNumberBar).withPosition(3,2).withSize(1, 1);

           Robot.getMainCompetitionTab().addBoolean("Limelight Connection", new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                // TODO Auto-generated method stub
                return hasLimelightUpdatedRecently();
            }
            
           }).withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 4).withSize(1, 1);
     


    }
    */

    public void outputToSmartDashboard() {

        SmartDashboard.putBoolean("Limelight Connection", hasLimelightUpdatedRecently());
        SmartDashboard.putString("Vision Subsystem current Alliance", currentAlliance.name());
        if(getBotPose() != null){
            SmartDashboard.putNumber("Limelight X:", getBotPose().getX());
            SmartDashboard.putNumber("Limelight Y:", getBotPose().getY());
            SmartDashboard.putNumber("Limelight Rotation: ", getBotPose().getRotation().getDegrees());   
        }
    }



    
}
