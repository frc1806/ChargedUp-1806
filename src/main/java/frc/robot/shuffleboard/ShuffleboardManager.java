package frc.robot.shuffleboard;

import frc.robot.shuffleboard.tabs.MainCompetitionTab;

public class ShuffleboardManager {
    private ShuffleboardTabBase mMainCompetitionTab;

    public ShuffleboardManager(){
        mMainCompetitionTab = new MainCompetitionTab();
    }

    public void registerTabs(){
        mMainCompetitionTab.createEntries();
    }

    public void updateAllTabs(){
        mMainCompetitionTab.update();
    }
}
