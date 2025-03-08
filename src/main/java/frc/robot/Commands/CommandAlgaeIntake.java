package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.Vars.Throttles;

public class CommandAlgaeIntake extends Command {
    private final AlgaeArm m_algaeArm;
    
    public CommandAlgaeIntake(AlgaeArm algaeArm) {
        m_algaeArm = algaeArm;

    addRequirements(m_algaeArm);
    }

    @Override
    public void execute() {
        m_algaeArm.IntakeAlgae(1 * Throttles.kAlgaeIntakeThrottle);
    }

    @Override
    public void end(boolean interrupted) {
        m_algaeArm.IntakeAlgae(0);
    }
    
}
