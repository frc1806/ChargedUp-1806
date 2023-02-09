package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//Inspired off of 1678's ShuffleboardTabBase from C2022 but added more features that 1806 would like to use (TODO)

public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab mTab;

    public abstract void createEntries();

    public abstract void update();

    public ShuffleboardTab getTab() {
        return mTab;
    }
}