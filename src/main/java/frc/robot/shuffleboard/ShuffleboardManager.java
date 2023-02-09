package frc.robot.shuffleboard;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.tabs.DebugTab;
import frc.robot.shuffleboard.tabs.DrivetrainTab;
import frc.robot.shuffleboard.tabs.IntakeTab;
import frc.robot.shuffleboard.tabs.MainCompetitionTab;
import frc.robot.shuffleboard.tabs.ProtruderTab;
import frc.robot.shuffleboard.tabs.VisionTab;
import frc.robot.util.SWATXboxController;

public class ShuffleboardManager {
    private List<ShuffleboardTabBase> mShuffleboardTabs;
    private List<ShuffleboardTabBase> mDebugTabs;
    private boolean testAdded;
    private static SWATXboxController mController;

    public ShuffleboardManager(){
        mShuffleboardTabs = Arrays.asList(new MainCompetitionTab(), new DebugTab());
        mDebugTabs = Arrays.asList(new DrivetrainTab(), new IntakeTab(), new ProtruderTab(), new VisionTab());
        testAdded = false;
        mController = RobotContainer.S_DRIVECONTROLS.getDriverController();
    }

    public void registerTabs(){
        for (ShuffleboardTabBase tab : mShuffleboardTabs) {
            tab.createEntries();
        }

        if (mController.getStartButton() && mController.getBackButton() || testAdded){
            testAdded=true;
            for (ShuffleboardTabBase dTab : mDebugTabs){
                dTab.createEntries();
            }
        }
    }

    public void updateAllTabs(){
        for (ShuffleboardTabBase tab : mShuffleboardTabs) {
            tab.update();
        }

        if (mController.getStartButton() && mController.getBackButton() || testAdded){
            if(testAdded==false){
                testAdded=true;
                for (ShuffleboardTabBase dTab : mDebugTabs){
                    dTab.createEntries();
                }
            }
            for (ShuffleboardTabBase dTab : mDebugTabs){
                dTab.update();
            }
        }
    }
}