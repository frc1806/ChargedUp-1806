package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class TelescopingRotatingArmFeedForwards {

    /**
     * Calculates the feed foward for the pivot based on both the arm angle and telescoping arm's center of gravity relative to the center.
     * @param armCenterOfGravityDistanceFromAxleMeters the distance of the center of gravity of the telescoping arm and pivot assembly from the center of the axle.
     * @param armAngleDegrees angle of the arm in degrees, up is 0.
     * @param gain a gain value to be tuned
     * @return A number, keep in mind motor units and whether this is being used in a SetVoltage or Set when deciding your gain
     */
    public static double CalculateArmFeedForward(double armCenterOfGravityDistanceFromAxleMeters, double armAngleDegrees, double gain){
        return armCenterOfGravityDistanceFromAxleMeters * 9.8 * Math.cos(Units.degreesToRadians(armAngleDegrees)) * gain; //distance * gravity * angle *
    }

    /**
     * Calculates the feed forward for the telescoping part of the telescoping arm
     * @param armAngleDegrees angle of the arm in degrees, up is 0.
     * @param gain a gain value to be tuned
     * @return a number representing the feed forward for the given angle and gain.
     */
    public static double CalculateTelescopeFeedForward(double armAngleDegrees, double gain){
        return Math.cos(Units.degreesToRadians(armAngleDegrees)) * gain;
    }
    
}
