package frc.robot.shuffleboard;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.tabs.DrivetrainTab;
import frc.robot.shuffleboard.tabs.ArmTab;
import frc.robot.shuffleboard.tabs.MainCompetitionTab;
import frc.robot.shuffleboard.tabs.OperatorTab;
import frc.robot.shuffleboard.tabs.VisionTab;
import frc.robot.util.SWATXboxController;

public class ShuffleboardManager {
    private List<ShuffleboardTabBase> mShuffleboardTabs;
    private MainCompetitionTab mMainCompetitionTab;
    private List<ShuffleboardTabBase> mDebugTabs;
    private boolean testAdded;
    private static SWATXboxController mController;
    private static SWATXboxController mDebugController;

    public ShuffleboardManager(){
        mMainCompetitionTab = new MainCompetitionTab();
        mShuffleboardTabs = Arrays.asList(mMainCompetitionTab, new OperatorTab());
        mDebugTabs = Arrays.asList(new DrivetrainTab(), new ArmTab(), new VisionTab());
        testAdded = false;
        mController = RobotContainer.S_DRIVECONTROLS.getDriverController();
        mDebugController = RobotContainer.S_DRIVECONTROLS.getDebugController();
    }

    public void registerTabs(){
        for (ShuffleboardTabBase tab : mShuffleboardTabs) {
            tab.createEntries();
        }

        if (mController.getStartButton() && mController.getBackButton() || mDebugController.getStartButton() && mDebugController.getBackButton() || testAdded){
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

        if (mController.getStartButton() && mController.getBackButton() || mDebugController.getStartButton() && mDebugController.getBackButton() || testAdded){
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

    public void addAutoChooser(SendableChooser<CommandBase> chooser){
        mMainCompetitionTab.addAutoChooser(chooser);
    }
}