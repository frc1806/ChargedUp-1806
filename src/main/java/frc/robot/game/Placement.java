package frc.robot.game;

import frc.robot.Constants;

public class Placement {

    Double extendDistance, pivotAngle;
    String name;

    public Placement(String name, Double extendDistance, Double pivotAngle){
        this.extendDistance = extendDistance;
        this.pivotAngle = pivotAngle;
        this.name = name;
    }

    // TODO (preset placements)
    public static Placement HOME = new Placement("Home", Constants.kProtruderDistanceAtFullRetract, 180.0);
    public static Placement LOW_PLACEMENT_CUBE = new Placement("Low Placement Cube", 10.0,180.0);
    public static Placement LOW_PLACEMENT_CONE = new Placement("Low Placement Cone", 10.0,180.0);
    public static Placement MED_PLACEMENT_CUBE = new Placement("Medium Placement Cube", 10.0,180.0);
    public static Placement MED_PLACEMENT_CONE = new Placement("Medium Placement Cone", 10.0,280.0);
    public static Placement HIGH_PLACEMENT_CUBE = new Placement("High Placement Cube", 10.0,180.0);
    public static Placement HIGH_PLACEMENT_CONE = new Placement("High Placement Cone", 10.0,295.0);

    public Double getExtendDistance(){
        return extendDistance;
    }

    public Double getPivotAngle(){
        return pivotAngle;
    }

    public String getPlacementName(){
        return name;
    }
}
