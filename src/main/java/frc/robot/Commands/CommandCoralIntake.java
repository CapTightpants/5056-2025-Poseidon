package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;
import frc.robot.Vars.Throttles;

public class CommandCoralIntake extends Command {
    private final CoralArm m_coralArm;
    
    public CommandCoralIntake(CoralArm coralArm) {
        m_coralArm = coralArm;

    addRequirements(m_coralArm);
    }

    @Override
    public void execute() {
        m_coralArm.IntakeCoral(-1 * Throttles.kCoralIntakeThrottle);
    }

    @Override
    public void end(boolean interrupted) {
        m_coralArm.IntakeCoral(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
