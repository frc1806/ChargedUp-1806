package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.ShuffleboardTabBase;

public class DrivetrainTab extends ShuffleboardTabBase {
    
    private GenericEntry mLeftDrivePower, mRightDrivePower;
    private GenericEntry mLeftLeaderOutput, mRightLeaderOutput, mLeftFollowerOutput, mRightFollowerOutput;
    private GenericEntry mLeftLeaderTemp, mRightLeaderTemp, mLeftFollowerTemp, mRightFollowerTemp;
    private GenericEntry mLeftLeaderAmps, mRightLeaderAmps, mLeftFollowerAmps, mRightFollowerAmps;
    

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("DriveTrain");

        mLeftDrivePower = mTab.add("Left Drive Power", RobotContainer.S_DRIVETRAIN.getLeftDrivePower())
            .withPosition(0, 0)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mRightDrivePower = mTab.add("Right Drive Power",RobotContainer.S_DRIVETRAIN.getRightDrivePower())
            .withPosition(8, 0)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        mLeftLeaderOutput = mTab.add("Left Leader Output",RobotContainer.S_DRIVETRAIN.getLeftLeader().getAppliedOutput())
            .withPosition(0, 1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        mRightLeaderOutput = mTab.add("Right Leader Output",RobotContainer.S_DRIVETRAIN.getRightLeader().getAppliedOutput())
            .withPosition(6, 1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        mLeftFollowerOutput = mTab.add("Left Follower Output",RobotContainer.S_DRIVETRAIN.getLeftFollower().getAppliedOutput())
            .withPosition(2, 1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mRightFollowerOutput = mTab.add("Right Follower Output",RobotContainer.S_DRIVETRAIN.getRightFollower().getAppliedOutput())
            .withPosition(8, 1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mLeftLeaderTemp = mTab.add("Left Leader Temp",RobotContainer.S_DRIVETRAIN.getLeftLeader().getMotorTemperature())
            .withPosition(0, 2)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mRightLeaderTemp = mTab.add("Right Leader Temp",RobotContainer.S_DRIVETRAIN.getRightLeader().getMotorTemperature())
            .withPosition(6, 2)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mLeftFollowerTemp = mTab.add("Left Follower Temp",RobotContainer.S_DRIVETRAIN.getLeftFollower().getMotorTemperature())
            .withPosition(2, 2)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mRightFollowerTemp = mTab.add("Right Follower Temp",RobotContainer.S_DRIVETRAIN.getRightFollower().getMotorTemperature())
            .withPosition(8, 2)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mLeftLeaderAmps = mTab.add("Left Leader Amps",RobotContainer.S_DRIVETRAIN.getLeftLeader().getOutputCurrent())
            .withPosition(0, 3)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mRightLeaderAmps = mTab.add("Right Leader Amps",RobotContainer.S_DRIVETRAIN.getRightLeader().getOutputCurrent())
            .withPosition(6, 3)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mLeftFollowerAmps = mTab.add("Left Follower Amps",RobotContainer.S_DRIVETRAIN.getLeftFollower().getOutputCurrent())
            .withPosition(2, 3)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mRightFollowerAmps = mTab.add("Right Follower Amps",RobotContainer.S_DRIVETRAIN.getRightFollower().getOutputCurrent())
            .withPosition(8, 3)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
    }

    @Override
    public void update() {
        mLeftDrivePower.setDouble(RobotContainer.S_DRIVETRAIN.getLeftDrivePower());
        mRightDrivePower.setDouble(RobotContainer.S_DRIVETRAIN.getRightDrivePower());
        mLeftLeaderOutput.setDouble(RobotContainer.S_DRIVETRAIN.getLeftLeader().getAppliedOutput());
        mRightLeaderOutput.setDouble(RobotContainer.S_DRIVETRAIN.getRightLeader().getAppliedOutput());
        mLeftFollowerOutput.setDouble(RobotContainer.S_DRIVETRAIN.getLeftFollower().getAppliedOutput());
        mRightFollowerOutput.setDouble(RobotContainer.S_DRIVETRAIN.getRightFollower().getAppliedOutput());
        mLeftLeaderTemp.setDouble(RobotContainer.S_DRIVETRAIN.getLeftLeader().getMotorTemperature());
        mRightLeaderTemp.setDouble(RobotContainer.S_DRIVETRAIN.getRightLeader().getMotorTemperature());
        mLeftFollowerTemp.setDouble(RobotContainer.S_DRIVETRAIN.getLeftFollower().getMotorTemperature());
        mRightFollowerTemp.setDouble(RobotContainer.S_DRIVETRAIN.getRightFollower().getMotorTemperature());
        mLeftLeaderAmps.setDouble(RobotContainer.S_DRIVETRAIN.getLeftLeader().getOutputCurrent());
        mRightLeaderAmps.setDouble(RobotContainer.S_DRIVETRAIN.getRightLeader().getOutputCurrent());
        mLeftFollowerAmps.setDouble(RobotContainer.S_DRIVETRAIN.getLeftFollower().getOutputCurrent());
        mRightFollowerAmps.setDouble(RobotContainer.S_DRIVETRAIN.getRightFollower().getOutputCurrent());
    }
    
}
