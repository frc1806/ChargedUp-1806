package frc.robot.commands.protruder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Protruder;

public class DetectProtruderUnprotrude extends CommandBase {
    /** Creates a new DetectProtruderUnprotrude. */

    private Protruder mProtruder;
    private double protruderSnapshot;

    public DetectProtruderUnprotrude(Protruder protruder) {
        mProtruder = protruder;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        protruderSnapshot = mProtruder.getTargetDistance();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mProtruder.getDistance() < protruderSnapshot - Constants.kProtruderPushBackInches;
    }
    
}
