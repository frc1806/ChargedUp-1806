package frc.robot.game;

public class Placement {

    Double extendDistance, pivotAngle;

    public Placement(Double extendDistance, Double pivotAngle){
        this.extendDistance = extendDistance;
        this.pivotAngle = pivotAngle;
    }

    // TODO (preset placements)
    public static Placement Home = new Placement(0.0,0.0);
    public static Placement LOW_PLACEMENT_CUBE = new Placement(0.0,0.0);
    public static Placement LOW_PLACEMENT_CONE = new Placement(0.0,0.0);
    public static Placement MED_PLACEMENT_CUBE = new Placement(0.0,0.0);
    public static Placement MED_PLACEMENT_CONE = new Placement(0.0,0.0);
    public static Placement HIGH_PLACEMENT_CUBE = new Placement(0.0,0.0);
    public static Placement HIGH_PLACEMENT_CONE = new Placement(0.0,0.0);

    public Double getExtendDistance(){
        return extendDistance;
    }

    public Double getPivotAngle(){
        return pivotAngle;
    }
}
