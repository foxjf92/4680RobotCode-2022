package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberHigh;

public class ReleaseHighClimber extends CommandBase {

    public ReleaseHighClimber() {
        addRequirements(RobotContainer.climberHigh);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ClimberHigh.release();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
