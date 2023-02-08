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
    private GenericEntry mLeftLeaderAmps, mRightLeaderAmpsmLeftFollowerAmps, mRightFollowerAmps;
    

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("DriveTrain");

        mLeftDrivePower = mTab.add("Left Drive Power", RobotContainer.S_DRIVETRAIN.getLeftDrivePower())
            .withPosition(0, 0)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mRightDrivePower = mTab.add("Right Follower Output",RobotContainer.S_DRIVETRAIN.getLeftFollower())
            .withPosition(0, 0)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        mLeftLeaderOutput = mTab.add("Left Follower Output",RobotContainer.S_DRIVETRAIN.getLeftFollower())
            .withPosition(2, 1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        mRightLeaderOutput = mTab.add("Left Follower Output",RobotContainer.S_DRIVETRAIN.getLeftFollower())
            .withPosition(0, 0)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        mLeftFollowerOutput = mTab.add("Left Follower Output",RobotContainer.S_DRIVETRAIN.getLeftFollower())
            .withPosition(0, 0)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
        
        mRightFollowerOutput = mTab.add("Left Follower Output",RobotContainer.S_DRIVETRAIN.getLeftFollower())
            .withPosition(0, 0)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();
    }

    @Override
    public void update() {
        mTab.add("Drive Train Subsystem",RobotContainer.S_DRIVETRAIN)
            .withPosition(0, 0);
    }
    
}
