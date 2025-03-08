package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Lift;

public class CommandMultiPosition extends Command {
    private final Lift m_lift;
    private final CoralArm m_coral;
    private final AlgaeArm m_algaeArm;
    private final kLiftPosition m_position;
    private boolean isFinished = false;

    public CommandMultiPosition(Lift lift, CoralArm coralArm, AlgaeArm algaeArm, kLiftPosition position) {
        m_lift = lift;
        m_coral = coralArm;
        m_algaeArm = algaeArm;
        m_position = position;

        addRequirements(m_lift, m_coral, m_algaeArm);
    }

    @Override
    public void execute() {
        if ((m_position == kLiftPosition.Base) && (m_lift.getLiftPosition(m_position) != true)) {
            Commands.sequence(
                new CommandPositionLift(m_lift, m_position),
                new CommandPositionCoral(m_coral, m_position),
                new CommandPositionAlgae(m_algaeArm, m_position)
            );
        }

        if (((m_coral.getCoralPosition(kLiftPosition.Base)) || (m_algaeArm.getAlgaePosition(kLiftPosition.Base))) && ((m_position != kLiftPosition.processor) || (m_position != kLiftPosition.Start))) {
            Commands.sequence(
                new CommandPositionCoral(m_coral, m_position),
                new CommandPositionAlgae(m_algaeArm, m_position),
                new CommandPositionLift(m_lift, m_position)
            );
        }

        else {
            new CommandPositionAlgae(m_algaeArm, m_position);
            new CommandPositionCoral(m_coral, m_position);
            new CommandPositionLift(m_lift, m_position);
        }
        // m_algaeArm.AlgaeArmPosition(m_position);
        // m_coral.CoralArmPosition(m_position);
        // m_lift.setLiftPosition(m_position);
        isFinished = true;
    }

    @Override public boolean isFinished() {
        return isFinished;
    }

    @Override public void end(boolean interrupted) {
        isFinished = false;
    }
}
