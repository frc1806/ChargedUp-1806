package frc.robot.game;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class PosesOfInterest {

    //Cone Nodes
    private static Translation2d blueRRightConeNode = new Translation2d(1.0, 0.50);
    private static Translation2d blueRLeftConeNode = new Translation2d(1.0, 1.6);
    private static Translation2d blueCRightConeNode = new Translation2d(1.0, 2.2);
    private static Translation2d blueCLeftConeNode = new Translation2d(1.0, 3.32);
    private static Translation2d blueLRightConeNode = new Translation2d(1.0, 3.86);
    private static Translation2d blueLLeftConeNode = new Translation2d(1.0, 5.0);

    //Cube Nodes
    private static Translation2d blueRCubeNode = new Translation2d(1.0, 1.05);
    private static Translation2d blueCCubeNode = new Translation2d(1.0, 2.7);
    private static Translation2d blueLCubeNode = new Translation2d(1.0, 4.4);

    //Feeder Stations
    private static Translation2d blueLeftFeeder = new Translation2d(16, 7.41);
    private static Translation2d blueRightFeeder = new Translation2d(16, 6.14);

    private static Translation2d ConvertBlueToRed(Translation2d input){
        return new Translation2d(input.getX(), Constants.kFieldWidth - input.getY());
    }

    private static ArrayList<Translation2d> blueConeNodes = new ArrayList<>();
    static {
        blueConeNodes.add(blueRRightConeNode);
        blueConeNodes.add(blueRLeftConeNode);
        blueConeNodes.add(blueCRightConeNode);
        blueConeNodes.add(blueCLeftConeNode);
        blueConeNodes.add(blueLRightConeNode);
        blueConeNodes.add(blueLLeftConeNode);
    }

    private static ArrayList<Translation2d> blueCubeNodes = new ArrayList<>();
    static {
        blueCubeNodes.add(blueRCubeNode);
        blueCubeNodes.add(blueCCubeNode);
        blueCubeNodes.add(blueLCubeNode);
    }

    private static ArrayList<Translation2d> blueFeederStations = new ArrayList<>();
    static {
        blueFeederStations.add(blueLeftFeeder);
        blueFeederStations.add(blueRightFeeder);
    }

    private static ArrayList<Translation2d> redConeNodes = new ArrayList<>();
    static {
        redConeNodes.add(ConvertBlueToRed(blueRRightConeNode));
        redConeNodes.add(ConvertBlueToRed(blueRLeftConeNode));
        redConeNodes.add(ConvertBlueToRed(blueCRightConeNode));
        redConeNodes.add(ConvertBlueToRed(blueCLeftConeNode));
        redConeNodes.add(ConvertBlueToRed(blueLRightConeNode));
        redConeNodes.add(ConvertBlueToRed(blueLLeftConeNode));
    }

    private static ArrayList<Translation2d> redCubeNodes = new ArrayList<>();
    static {
        redCubeNodes.add(ConvertBlueToRed(blueRCubeNode));
        redCubeNodes.add(ConvertBlueToRed(blueCCubeNode));
        redCubeNodes.add(ConvertBlueToRed(blueLCubeNode));
    }

    private static ArrayList<Translation2d> redFeederStations = new ArrayList<>();
    static {
        redFeederStations.add(ConvertBlueToRed(blueLeftFeeder));
        redFeederStations.add(ConvertBlueToRed(blueRightFeeder));
    }

    private static Translation2d GetClosestTranslationInList(Translation2d currentTranslation, ArrayList<Translation2d> list){
        double minimumDistance = Double.MAX_VALUE;
        Translation2d closestNode = null;
        for(Translation2d potentialClosestNode: list){
            double distance = potentialClosestNode.getDistance(currentTranslation);
            if(distance < minimumDistance){
                minimumDistance = distance;
                closestNode = potentialClosestNode;
            }
        }
        return closestNode;
    }

    public static Translation2d GetClosestConeNode(Alliance currentAlliance, Pose2d currentPose){
        switch(currentAlliance){
            case Blue:
                    return GetClosestTranslationInList(currentPose.getTranslation(), blueConeNodes);
            case Invalid:
            case Red:
            default:
                    return GetClosestTranslationInList(currentPose.getTranslation(), redConeNodes);            
        }
    }

    public static Translation2d GetClosestCubeNode(Alliance currentAlliance, Pose2d currentPose){
        switch(currentAlliance){
            case Blue:
                    return GetClosestTranslationInList(currentPose.getTranslation(), blueCubeNodes);
            case Invalid:
            case Red:
            default:
                    return GetClosestTranslationInList(currentPose.getTranslation(), redCubeNodes);            
        }
    }

    public static Translation2d GetClosestFeederStation(Alliance currentAlliance, Pose2d currentPose){
        switch(currentAlliance){
            case Blue:
                    return GetClosestTranslationInList(currentPose.getTranslation(), blueFeederStations);
            case Invalid:
            case Red:
            default:
                    return GetClosestTranslationInList(currentPose.getTranslation(), redFeederStations);            
        }
    }
}
