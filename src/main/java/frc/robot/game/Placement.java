package frc.robot.game;

public class Placement {

    Double extendDistance, extendSpeed, pivotAngle, pivotSpeed;

    public Placement(Double extendDistance, Double extendSpeed, Double pivotAngle, Double pivotSpeed){
        this.extendDistance = extendDistance;
        this.extendSpeed = extendSpeed;
        this.pivotAngle = pivotAngle;
        this.pivotSpeed = pivotSpeed;
    }

    // TODO (preset placements)
    public static Placement LOW_PLACEMENT_CUBE = new Placement(0.0,0.0,0.0,0.0);
    public static Placement LOW_PLACEMENT_CONE = new Placement(0.0,0.0,0.0,0.0);
    public static Placement MED_PLACEMENT_CUBE = new Placement(0.0,0.0,0.0,0.0);
    public static Placement MED_PLACEMENT_CONE = new Placement(0.0,0.0,0.0,0.0);
    public static Placement HIGH_PLACEMENT_CUBE = new Placement(0.0,0.0,0.0,0.0);
    public static Placement HIGH_PLACEMENT_CONE = new Placement(0.0,0.0,0.0,0.0);

    public Double getExtendDistance(){
        return extendDistance;
    }

    public Double getExtendSpeed(){
        return extendSpeed;
    }

    public Double getPivotAngle(){
        return pivotAngle;
    }

    public Double getPivotSpeed(){
        return pivotSpeed;
    }
}
