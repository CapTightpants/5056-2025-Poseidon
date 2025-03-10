package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars.Throttles;
import frc.robot.subsystems.CoralArm;

public class CommandCoralOuttake extends Command {
    private final CoralArm m_coralArm;

    public CommandCoralOuttake(CoralArm coralArm) {
        m_coralArm = coralArm;
        addRequirements(m_coralArm);
    }

    @Override
    public void execute() {
        m_coralArm.IntakeCoral(1 * Throttles.kCoralIntakeThrottle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_coralArm.IntakeCoral(0);
    }
}

