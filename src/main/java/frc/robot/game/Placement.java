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
    public static Placement HOME = new Placement("Home", Constants.kProtruderDistanceAtFullRetract, 185.0);
    public static Placement LOW_PLACEMENT_CUBE = new Placement("Low Placement Cube", Constants.kProtruderDistanceAtFullRetract + 10.0,240.0);
    public static Placement LOW_PLACEMENT_CONE = new Placement("Low Placement Cone", Constants.kProtruderDistanceAtFullRetract + 10.0,240.0);
    public static Placement MED_PLACEMENT_CUBE = new Placement("Medium Placement Cube", Constants.kProtruderDistanceAtFullRetract + 22.0,252.0);
    public static Placement MED_PLACEMENT_CONE = new Placement("Medium Placement Cone", Constants.kProtruderDistanceAtFullRetract + 22.0,252.0);
    public static Placement HIGH_PLACEMENT_CUBE = new Placement("High Placement Cube", Constants.kProtruderDistanceAtFullRetract + 32.0,273.0);
    public static Placement HIGH_PLACEMENT_CONE = new Placement("High Placement Cone", Constants.kProtruderDistanceAtFullRetract + 32.0,275.0);
    public static Placement FEEDER_STATION = new Placement("Feeder Station",  Constants.kProtruderDistanceAtFullRetract + 17.0,65.0);
    //public static Placement GROUND_INTAKE = new Placement("Ground Intake", Constants.kProtruderDistanceAtFullRetract + 14.0 ,122.0); Pre cymbal move
    public static Placement GROUND_INTAKE_CUBE = new Placement("Ground Intake Cube", Constants.kProtruderDistanceAtFullRetract + 15.5 ,122.0);
    public static Placement GROUND_INTAKE_CONE = new Placement("Ground Intake Cone", Constants.kProtruderDistanceAtFullRetract + 14.5 ,127.0); 
    public static Placement FRONT_GROUND_INTAKE = new Placement("FrontGroundIntake", Constants.kProtruderDistanceAtFullRetract + 14.5, 220.0);

    public static Placement LAUNCH_A_CUBE_START = new Placement("LaunchStart", Constants.kProtruderDistanceAtFullRetract, 273.0);
    public static Placement LAUNCH_A_CUBE_END = new Placement("LaunchEnd", Constants.kProtruderDistanceAtFullRetract + 32.0, 273.0);

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
