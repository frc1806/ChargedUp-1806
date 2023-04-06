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

    public static Placement LOW_PLACEMENT_CUBE = new Placement("Low Placement Cube", Constants.kProtruderDistanceAtFullRetract + 11.5,218.0);
    // TODO (preset placements)
    public static Placement HOME = new Placement("Home", Constants.kProtruderDistanceAtFullRetract, 185.0);
    public static Placement LOW_PLACEMENT_CONE = new Placement("Low Placement Cone", Constants.kProtruderDistanceAtFullRetract + 13.0,224.0);
    public static Placement MED_PLACEMENT_CUBE = new Placement("Medium Placement Cube", Constants.kProtruderDistanceAtFullRetract + 23.0,257.0);
    public static Placement MED_PLACEMENT_CONE = new Placement("Medium Placement Cone", Constants.kProtruderDistanceAtFullRetract + 20.5,271.0);
    public static Placement HIGH_PLACEMENT_CUBE = new Placement("High Placement Cube", Constants.kProtruderDistanceAtFullRetract + 37.0,274.0);
    public static Placement HIGH_PLACEMENT_CONE = new Placement("High Placement Cone", Constants.kProtruderDistanceAtFullRetract + 38.0,280.0);
    public static Placement FEEDER_STATION = new Placement("Feeder Station Cube",  Constants.kProtruderDistanceAtFullRetract + 8.0,66.0);
    public static Placement FEEDER_STATION_CONE = new Placement("Feeder Station Cone",  Constants.kProtruderDistanceAtFullRetract + 8.0,62.0);
    //public static Placement GROUND_INTAKE = new Placement("Ground Intake", Constants.kProtruderDistanceAtFullRetract + 14.0 ,122.0); Pre cymbal move
    public static Placement GROUND_INTAKE_CUBE = new Placement("Ground Intake Cube", Constants.kProtruderDistanceAtFullRetract + 20.0 ,121.0);
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
