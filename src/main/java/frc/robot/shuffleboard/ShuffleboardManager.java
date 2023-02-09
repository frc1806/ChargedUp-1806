package frc.robot.shuffleboard;

import java.util.Arrays;
import java.util.List;

import frc.robot.shuffleboard.tabs.DebugTab;
import frc.robot.shuffleboard.tabs.DrivetrainTab;
import frc.robot.shuffleboard.tabs.IntakeTab;
import frc.robot.shuffleboard.tabs.MainCompetitionTab;
import frc.robot.shuffleboard.tabs.ProtruderTab;
import frc.robot.shuffleboard.tabs.VisionTab;

public class ShuffleboardManager {

    private List<ShuffleboardTabBase> mShuffleboardTabs;

    public ShuffleboardManager(){
        mShuffleboardTabs = Arrays.asList(new MainCompetitionTab(), new DebugTab(), new DrivetrainTab(), new IntakeTab(), new ProtruderTab(), new VisionTab());
    }

    public void registerTabs(){
        for (ShuffleboardTabBase tab : mShuffleboardTabs) {
            tab.createEntries();
        }
    }

    public void updateAllTabs(){
        for (ShuffleboardTabBase tab : mShuffleboardTabs) {
            tab.update();
        }
    }
}
