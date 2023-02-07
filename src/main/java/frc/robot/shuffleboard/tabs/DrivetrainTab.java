package frc.robot.shuffleboard.tabs;

import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.ShuffleboardTabBase;

public class DrivetrainTab extends ShuffleboardTabBase {

    private ComplexWidget mDriveTrainWidget;

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("DriveTrain Tab");
        
        mDriveTrainWidget = mTab.add("Drive Train Subsystem",RobotContainer.S_DRIVETRAIN)
            .withPosition(0, 0);
    }

    @Override
    public void update() {
    }
    
}
