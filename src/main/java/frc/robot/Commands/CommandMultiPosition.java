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
    // private final AlgaeArm m_algaeArm;
    private final kLiftPosition m_position;

    private Command commandOutOfBase;
    private Command commandToBase;
    private Command commandOtherPositions;

    public CommandMultiPosition(Lift lift, CoralArm coralArm, kLiftPosition position) {
        m_lift = lift;
        m_coral = coralArm;
        // m_algaeArm = algaeArm;
        m_position = position;

        commandToBase = Commands.sequence(
            new CommandPositionLift(m_lift, m_position),
            new CommandPositionCoral(m_coral, m_position)
            // Commands.parallel(
            //     new CommandPositionAlgae(m_algaeArm, m_position)
            // )
        );

        commandOutOfBase = Commands.sequence(
            // Commands.parallel(
                // new CommandPositionAlgae(m_algaeArm, m_position)
                // ),
            new CommandPositionCoral(m_coral, m_position),
            new CommandPositionLift(m_lift, m_position)
        );

        commandOtherPositions = Commands.parallel(
            // new CommandPositionAlgae(m_algaeArm, m_position),
            new CommandPositionCoral(m_coral, m_position),
            new CommandPositionLift(m_lift, m_position)
        );

        addRequirements(m_lift, m_coral);
    }

    @Override
    public void execute() {
        if ((m_position == kLiftPosition.Station) && (!m_lift.getLiftPosition(m_position))) {
            commandToBase.schedule();
        }
        else if (((m_coral.getCoralPosition(kLiftPosition.Station)) || ((m_position != kLiftPosition.Processor) && (m_position != kLiftPosition.Start)))) {
            commandOutOfBase.schedule();
        }
        else {
            commandOtherPositions.schedule();
        }

        // m_algaeArm.AlgaeArmPosition(m_position);
        // m_coral.CoralArmPosition(m_position);
        // m_lift.setLiftPosition(m_position);
        
    }

    @Override public boolean isFinished() {
        return true;
    }
}
