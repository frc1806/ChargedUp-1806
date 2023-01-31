package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class VisionSubsystem extends SubsystemBase{

    private NetworkTable limelightTable;
    private double currentTimestamp;

    //SIMULATION
    private final Translation2d testNodePose = new Translation2d(15.77, 4.98);

    public VisionSubsystem(String hostname) { 
        limelightTable = NetworkTableInstance.getDefault().getTable(hostname);
        currentTimestamp = 0.0;
    }

    public double getValidTargetUpdateTimestamp(){
        return limelightTable.getEntry("tv").getLastChange();
    }

    public boolean hasLimelightUpdatedRecently(){
        return getValidTargetUpdateTimestamp()/ 1000000 > currentTimestamp - 1.0;
    }


    public double getAprilTagId(){
        return limelightTable.getEntry("tid").getDouble(0);
    }

    public double getTarget(){
        return limelightTable.getEntry("tx").getDouble(0);
    }


    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void periodic() {
        currentTimestamp = Timer.getFPGATimestamp();
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
    }



    
}
