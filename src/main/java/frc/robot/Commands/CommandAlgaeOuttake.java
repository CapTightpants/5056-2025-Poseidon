package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars.Throttles;
import frc.robot.subsystems.AlgaeArm;

public class CommandAlgaeOuttake extends Command {
        private final AlgaeArm m_algaeArm;

    private CommandAlgaeOuttake(AlgaeArm algaeArm) {
        m_algaeArm = algaeArm;
        addRequirements(m_algaeArm);
    }

    @Override
    public void execute() {
        m_algaeArm.IntakeAlgae(-1 * Throttles.kCoralIntakeThrottle);
    }
}
