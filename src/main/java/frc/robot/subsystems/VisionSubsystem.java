package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;



import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase{

    private NetworkTable limelightTable;
    private double currentTimestamp;

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
