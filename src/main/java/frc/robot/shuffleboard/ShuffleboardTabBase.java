package frc.robot.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//Inspired off of 1678's ShuffleboardTabBase from C2022 but added more features that 1806 would like to use (TODO)

public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab mTab;

    public abstract void createEntries();

    public abstract void update();

    public ShuffleboardTab getTab() {
        return mTab;
    }

    public GenericEntry createStringEntry(String title, String value){
        return mTab.add(title, value).getEntry();
    }

    public GenericEntry createNumberEntry(String title, Double value){
        return mTab.add(title, value).getEntry();
    }

    public GenericEntry createBooleanEntry(String title, Boolean value){
        return mTab.add(title, value).getEntry();
    }

    public GenericEntry createStringEntry(String title, String value, Integer SizeX, Integer SizeY, BuiltInWidgets Widget){
        return mTab.add(title, value).withSize(SizeX, SizeY).withWidget(Widget).getEntry();
    }

    public GenericEntry createNumberEntry(String title, Double value, Integer SizeX, Integer SizeY, BuiltInWidgets Widget){
        return mTab.add(title, value).withSize(SizeX, SizeY).withWidget(Widget).getEntry();
    }

    public GenericEntry createBooleanEntry(String title, Boolean value, Integer SizeX, Integer SizeY, BuiltInWidgets Widget){
        return mTab.add(title, value).withSize(SizeX, SizeY).withWidget(Widget).getEntry();
    }

    public GenericEntry createStringEntry(String title, String value, Integer PosX, Integer PosY, Integer SizeX, Integer SizeY){
        return mTab.add(title, value).withPosition(PosX, PosY).withSize(SizeX, SizeY).getEntry();
    }

    public GenericEntry createNumberEntry(String title, Double value, Integer PosX, Integer PosY, Integer SizeX, Integer SizeY){
        return mTab.add(title, value).withPosition(PosX, PosY).withSize(SizeX, SizeY).getEntry();
    }
    public GenericEntry createBooleanEntry(String title, Boolean value, Integer PosX, Integer PosY, Integer SizeX, Integer SizeY){
        return mTab.add(title, value).withPosition(PosX, PosY).withSize(SizeX, SizeY).getEntry();
    }

    public GenericEntry createStringEntry(String title, String value, Integer PosX, Integer PosY, Integer SizeX, Integer SizeY, BuiltInWidgets Widget){
        return mTab.add(title, value).withPosition(PosX, PosY).withPosition(SizeX, SizeY).withWidget(Widget).getEntry();
    }

    public GenericEntry createNumberEntry(String title, Double value, Integer PosX, Integer PosY, Integer SizeX, Integer SizeY, BuiltInWidgets Widget){
        return mTab.add(title, value).withPosition(PosX, PosY).withPosition(SizeX, SizeY).withWidget(Widget).getEntry();
    }

    public GenericEntry createBooleanEntry(String title, Boolean value, Integer PosX, Integer PosY, Integer SizeX, Integer SizeY, BuiltInWidgets Widget){
        return mTab.add(title, value).withPosition(PosX, PosY).withPosition(SizeX, SizeY).withWidget(Widget).getEntry();
    }
}