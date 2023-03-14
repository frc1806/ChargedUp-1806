package frc.robot.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.subsystems.DriverControls;

public class MainCompetitionTab extends ShuffleboardTabBase{
    private GenericEntry leftDrivePower;
    private GenericEntry rightDrivePower;
    private GenericEntry currentMode;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Competition Tab");
        
        mTab.add("Driver Controls", DriverControls.controllerConfigChooser)
            .withPosition(0, 0)
            .withSize(2,1);
        

        
        leftDrivePower = mTab.add("Left Drive Power", RobotContainer.S_DRIVETRAIN.getLeftDrivePower())
            .withPosition(0, 1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        rightDrivePower = mTab.add("Right Drive Power", RobotContainer.S_DRIVETRAIN.getRightDrivePower())
            .withPosition(8, 1)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kNumberBar)
            .getEntry();

        currentMode = mTab.add("Game Piece Mode", RobotContainer.GetCurrentGamePieceMode().name())
            .withPosition(8, 2)
            .withSize(2,1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        
    }

    public void addAutoChooser(SendableChooser<CommandBase> autoChooser){
        mTab.add("Auto Chooser", autoChooser)
            .withPosition(8,0)
            .withSize(2,1);
    }

    @Override
    public void update() {
        leftDrivePower.setDouble(RobotContainer.S_DRIVETRAIN.getLeftDrivePower());
        rightDrivePower.setDouble(RobotContainer.S_DRIVETRAIN.getRightDrivePower());
        currentMode.setString(RobotContainer.GetCurrentGamePieceMode().name());
    }
    
}
