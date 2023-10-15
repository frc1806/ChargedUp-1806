// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class TrapezoidalDiffDriveTurnPID {

    ProfiledPIDController steeringController;
    DriveTrain driveTrain;

    public TrapezoidalDiffDriveTurnPID(DriveTrain drivetrain){
        this.driveTrain = drivetrain;
        steeringController = new ProfiledPIDController(0.25, 0.0, 0.01, new Constraints(Math.PI, Math.PI*8));
        //steeringController.setIntegratorRange(-0.15, 0.15);
        steeringController.setTolerance(Units.degreesToRadians(0.1));
        steeringController.enableContinuousInput(0, 2*Math.PI);
    }

    public void initialize(){
        steeringController.reset(driveTrain.getPose().getRotation().getRadians());
    }
    public double calculate( double goal){
        return applyMinimumOutput(steeringController.calculate(driveTrain.getPose().getRotation().getRadians(), goal), 0.005, 0.03);
    }

    private double applyMinimumOutput(double value, double deadZone, double minimumOutput)
	{
		if(Math.abs(value) < deadZone)
		{
			return 0.0;
		}
		//Apply Minimum output
		if(Math.abs(value) > 0)
		{
			value = (value>0?1:-1) * (Math.abs(value) * (1.0-minimumOutput) + minimumOutput);
		}
		return value;
		
	}
}
